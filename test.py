import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from tf_transformations import euler_from_quaternion
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import math


class SensorReader(Node):

    def __init__(self):
        super().__init__('sensor_reader')

        self.path_pub = self.create_publisher(Path, '/trajectory',10)
        self.path = Path()
        self.path.header.frame_id = 'odom'
        # IMU subscription
        self.create_subscription(
            Imu,
            '/imu',
            self.imu_cb,
            qos_profile_sensor_data
        )

        # JointState subscription
        self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_cb,
            10
        )
        self.br = TransformBroadcaster(self)
        self.wheel_radius=0.033
        self.shaft=0.16

        self.x=0.0
        self.y=0.0
        self.yaw=0.0
        self.last_time=self.get_clock().now()
        self.wL = 0.0
        self.wR = 0.0

    def imu_cb(self, msg):
        orient_z = msg.orientation.z
        ang_vel_z = msg.angular_velocity.z
        q = msg.orientation
        _,_, self.yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        

        print(
            f"IMU | orientation.z={orient_z:.6f}, "
            f"angular_velocity.z={ang_vel_z:.6f},"
            f"YAW={self.yaw:.6f}"
        )

    def joint_cb(self, msg):
        self.wL = msg.velocity[0]
        self.wR = msg.velocity[1]

        now = self.get_clock().now()
        dt=(now-self.last_time).nanoseconds*1e-9
        self.last_time=now

        vL = self.wheel_radius*self.wL
        vR = self.wheel_radius*self.wR
        v=0.5*(vR+vL)

        self.x += v*math.cos(self.yaw)*dt
        self.y += v*math.sin(self.yaw)*dt

        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'

        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0


        from tf_transformations import quaternion_from_euler
        qz= quaternion_from_euler(0,0,self.yaw)
        t.transform.rotation.x = qz[0]
        t.transform.rotation.y = qz[1]
        t.transform.rotation.z = qz[2]
        t.transform.rotation.w = qz[3]

        self.br.sendTransform(t)


        pose = PoseStamped()
        pose.header.stamp = now.to_msg()
        pose.header.frame_id = 'odom'

        pose.pose.position.x = self.x
        pose.pose.position.y = self.y
        pose.pose.position.z = 0.0

        pose.pose.orientation.x = qz[0]
        pose.pose.orientation.y = qz[1]
        pose.pose.orientation.z = qz[2]
        pose.pose.orientation.w = qz[3]

        self.path.poses.append(pose)
        self.path.header.stamp = now.to_msg()

        self.path_pub.publish(self.path)
        # for name, pos, vel in zip(
        #     msg.name,
        #     msg.position,
        #     msg.velocity
        # ):
            # print(
            #     f"Joint | {name}: "
            #     f"pos={pos:.3f}, vel={vel:.3f}"
            # )


def main():
    rclpy.init()
    node = SensorReader()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
