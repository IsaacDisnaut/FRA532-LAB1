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
import pandas as pd
import os

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
        self.metrics_list = []
        self.log_file = 'ekf_fusion_results.csv'
        self.total_dist = 0.0
        self.last_pos = None
        # ---------------- PUBLISHERS ----------------
        self.path_pub = self.create_publisher(Path, '/trajectory', 10)
        self.path = Path()
        self.path.header.frame_id = 'odom'

        self.br = TransformBroadcaster(self)
        self.raw_odom_x = 0.0
        self.raw_odom_y = 0.0
        self.raw_odom_yaw = 0.0

        # ตัวแปรเก็บค่าล่าสุดเพื่อทำ Metrics
        self.latest_imu_innovation = 0.0
        self.latest_enc_innovation = 0.0
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
            0.1,  # x
            0.1,  # y
            0.01,  # yaw
            1.0,   # v
            1.0    # w
        ])

        # ---------- PROCESS NOISE ----------
        # Motion uncertainty mainly comes from v and w
        self.Q = np.diag([
            0.9,  # x
            0.9,  # y
            0.9,  # yaw
            0.9,   # v
            0.9    # w
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
        y[0,0] = math.atan2(math.sin(y[0,0]), math.cos(y[0,0]))
        
        # เก็บค่า Innovation เพื่อดู Robustness
        self.latest_imu_innovation = float(abs(y[0,0]))

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
        # ... [Initial check และ dt เหมือนเดิม] ...
        if not self.initialized:
            self.last_time = now
            self.initialized = True
            return
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now

        # 1. Prediction Step
        # ... [ส่วนทำนาย State x และ P เหมือนเดิม] ...
        yaw = self.x[2,0]
        v = self.x[3,0]
        w = self.x[4,0]
        self.x[0,0] += v * math.cos(yaw) * dt
        self.x[1,0] += v * math.sin(yaw) * dt
        self.x[2,0] += w * dt
        self.x[2,0] = math.atan2(math.sin(self.x[2,0]), math.cos(self.x[2,0]))

        F = np.eye(5)
        F[0, 2] = -v * math.sin(yaw) * dt
        F[0, 3] = math.cos(yaw) * dt
        F[1, 2] =  v * math.cos(yaw) * dt
        F[1, 3] = math.sin(yaw) * dt
        F[2, 4] = dt
        self.P = F @ self.P @ F.T + self.Q

        # 2. Measurement Update (Encoder)
        vL, vR = self.r * wL, self.r * wR
        v_meas = 0.5 * (vR + vL)
        w_meas = (vR - vL) / self.L

        z = np.array([[v_meas], [w_meas]])
        H = np.array([[0, 0, 0, 1, 0], [0, 0, 0, 0, 1]])
        R_enc = np.diag([0.1, 0.1])

        y = z - H @ self.x
        self.latest_enc_innovation = float(np.linalg.norm(y)) # Robustness metric

        S = H @ self.P @ H.T + R_enc
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        self.P = (np.eye(5) - K @ H) @ self.P
        self.x[2,0] = math.atan2(math.sin(self.x[2,0]), math.cos(self.x[2,0]))

        # ---- [เพิ่มส่วนการคำนวณ Metrics] ----
        
        # A. Raw Odometry (สำหรับคำนวณ Drift)
        self.raw_odom_x += v_meas * math.cos(self.raw_odom_yaw) * dt
        self.raw_odom_y += v_meas * math.sin(self.raw_odom_yaw) * dt
        self.raw_odom_yaw += w_meas * dt
        
        # B. คำนวณ Drift (ระยะห่างระหว่าง EKF กับ Raw Odom)
        drift = math.sqrt((self.x[0,0] - self.raw_odom_x)**2 + (self.x[1,0] - self.raw_odom_y)**2)
        
        # C. คำนวณ Accuracy (ใช้ค่าความมั่นใจจาก Matrix P)
        accuracy_trace = np.trace(self.P)

        # D. ระยะทางสะสม
        if self.last_pos is not None:
            self.total_dist += math.sqrt((self.x[0,0]-self.last_pos[0])**2 + (self.x[1,0]-self.last_pos[1])**2)
        self.last_pos = (self.x[0,0], self.x[1,0])

        # บันทึกข้อมูล
        self.metrics_list.append({
            'timestamp': now.nanoseconds * 1e-9,
            'accuracy_P_trace': accuracy_trace,   # ยิ่งน้อย ยิ่งแม่น (Accuracy)
            'drift_error': drift,                 # ระยะเพี้ยนจาก Odom เปล่า (Drift)
            'imu_innovation': self.latest_imu_innovation, # ความเสถียร (Robustness)
            'enc_innovation': self.latest_enc_innovation, # ความเสถียร (Robustness)
            'dist_travelled': self.total_dist
        })

        if len(self.metrics_list) % 100 == 0:
            pd.DataFrame(self.metrics_list).to_csv(self.log_file, index=False)
        self.get_logger().info(
    f'x={self.x[0,0]:.3f}, y={self.x[1,0]:.3f}, yaw={self.x[2,0]:.3f}, v={self.x[3,0]:.3f}, w={self.x[4,0]:.3f}'
)

        self.publish_tf_and_path(now)

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

if __name__ == '__main__':
    main()
