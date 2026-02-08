import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import TransformStamped, PoseStamped
from tf2_ros import TransformBroadcaster
from tf_transformations import euler_from_quaternion, quaternion_from_euler
from nav_msgs.msg import Path
import numpy as np
import math


class SensorReader(Node):

    def __init__(self):
        super().__init__('ekf_sensor_fusion')

        # ---------------- ROS PARAM ----------------
        self.set_parameters([
            rclpy.parameter.Parameter(
                'use_sim_time',
                rclpy.Parameter.Type.BOOL,
                True
            )
        ])

        # ---------------- PUBLISHERS ----------------
        self.path_pub = self.create_publisher(Path, '/trajectory', 10)
        self.path = Path()
        self.path.header.frame_id = 'odom'

        self.br = TransformBroadcaster(self)

        # ---------------- SUBSCRIBERS ----------------
        self.create_subscription(
            Imu,
            '/imu',
            self.imu_cb,
            qos_profile_sensor_data
        )

        self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_cb,
            10
        )

        # ---------------- ROBOT PARAMETERS ----------------
        self.r = 0.033   # wheel radius [m]
        self.L = 0.16    # wheel base [m]

        # State: x = [x, y, yaw, v, w]^T
        # ---------- STATE ----------
        self.x = np.zeros((5, 1))

        # ---------- COVARIANCE ----------
        # High uncertainty on velocity states at start
        self.P = np.diag([
            0.01,  # x
            0.01,  # y
            0.01,  # yaw
            1.0,   # v
            1.0    # w
        ])

        # ---------- PROCESS NOISE ----------
        # Motion uncertainty mainly comes from v and w
        self.Q = np.diag([
            0.001,  # x
            0.001,  # y
            0.002,  # yaw
            0.05,   # v
            0.05    # w
        ])

        # ---------- MEASUREMENT NOISE ----------
        # IMU orientation (yaw)
        self.R = np.array([[0.05]])

        # ---------------- TIME & FLAGS ----------------
        self.last_time = self.get_clock().now()

        # For async callbacks
        self.initialized = False

        # Measurement buffers
        self.yaw_meas = None
        self.w_meas = None

        self.get_logger().info('EKF sensor fusion node initialized')


    # ---------------- IMU UPDATE ----------------
   # ---------------- IMU UPDATE (yaw) ----------------
    def imu_cb(self, msg):
        q = msg.orientation
        _, _, yaw_meas = euler_from_quaternion([q.x, q.y, q.z, q.w])

        z = np.array([[yaw_meas]])
        H = np.array([[0, 0, 1, 0, 0]])

        y = z - H @ self.x
        # wrap yaw innovation
        y[0,0] = math.atan2(math.sin(y[0,0]), math.cos(y[0,0]))

        S = H @ self.P @ H.T + self.R
        K = self.P @ H.T @ np.linalg.inv(S)

        self.x = self.x + K @ y
        self.x[2,0] = math.atan2(math.sin(self.x[2,0]), math.cos(self.x[2,0]))

        self.P = (np.eye(5) - K @ H) @ self.P

    def encoder_update(self, v_meas):
        z = np.array([[v_meas]])
        H = np.array([[0, 0, 0, 1, 0]])
        Rv = np.array([[0.2]])  # encoder noise

        y = z - H @ self.x
        S = H @ self.P @ H.T + Rv
        K = self.P @ H.T @ np.linalg.inv(S)

        self.x = self.x + K @ y
        self.P = (np.eye(5) - K @ H) @ self.P

    


    # ---------------- ODOM PREDICTION ----------------
    def joint_cb(self, msg):
        wL, wR = msg.velocity[0], msg.velocity[1]

        now = self.get_clock().now()
        if not self.initialized:
            self.initialized = True
            return

        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now
        if dt <= 0:
            return

        vL = self.r * wL
        vR = self.r * wR
        v_meas = 0.5 * (vR + vL)
        w_meas = (vR - vL) / self.L

        # encoder update (v)
        self.encoder_update(v_meas)

        # set w as control input (หรือจะทำ gyro update แยกก็ได้)
        self.x[4,0] = w_meas


        # EKF prediction (ใช้ state!)
        yaw = self.x[2,0]
        v   = self.x[3,0]
        w   = self.x[4,0]

        self.x[0,0] += v * math.cos(yaw) * dt
        self.x[1,0] += v * math.sin(yaw) * dt
        self.x[2,0] += w * dt
        self.x[2,0] = math.atan2(math.sin(self.x[2,0]), math.cos(self.x[2,0]))

        F = np.array([
            [1, 0, -v*math.sin(yaw)*dt, math.cos(yaw)*dt, 0],
            [0, 1,  v*math.cos(yaw)*dt, math.sin(yaw)*dt, 0],
            [0, 0, 1,                   0,                dt],
            [0, 0, 0,                   1,                0],
            [0, 0, 0,                   0,                1]
        ])

        self.P = F @ self.P @ F.T + self.Q

        self.publish_tf_and_path(now)

        print(f"x: {self.x[0,0]}  y: {self.x[1,0]} rot: {self.x[2,0]}")

    # ---------------- TF + PATH ----------------
    def publish_tf_and_path(self, now):
        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'

        t.transform.translation.x = self.x[0,0]
        t.transform.translation.y = self.x[1,0]
        t.transform.translation.z = 0.0

        q = quaternion_from_euler(0, 0, self.x[2,0])
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.br.sendTransform(t)

        pose = PoseStamped()
        pose.header = t.header
        pose.pose.position.x = self.x[0,0]
        pose.pose.position.y = self.x[1,0]
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]

        self.path.poses.append(pose)
        self.path.header.stamp = now.to_msg()
        self.path_pub.publish(self.path)


def main():
    rclpy.init()
    test_ekf= SensorReader()
    rclpy.spin(test_ekf)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
