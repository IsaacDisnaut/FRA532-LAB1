import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import TransformStamped, PoseStamped
from tf2_ros import TransformBroadcaster
from tf_transformations import euler_from_quaternion, quaternion_from_euler
from nav_msgs.msg import Path
import math
import numpy as np
import pandas as pd # สำหรับบันทึก CSV
import os

class SensorReader(Node):

    def __init__(self):
        super().__init__('sensor_reader')

        self.path_pub = self.create_publisher(Path, '/trajectory', 10)
        self.path = Path()
        self.path.header.frame_id = 'odom'
        
        # Subscriptions
        self.create_subscription(Imu, '/imu', self.imu_cb, qos_profile_sensor_data)
        self.create_subscription(JointState, '/joint_states', self.joint_cb, 10)
        
        self.br = TransformBroadcaster(self)
        self.wheel_radius = 0.033
        self.shaft = 0.16

        # --- States ---
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        
        # สำหรับคำนวณ Drift (ตำแหน่งจาก Encoder อย่างเดียว)
        self.raw_x = 0.0
        self.raw_y = 0.0
        self.raw_yaw = 0.0

        self.last_time = self.get_clock().now()
        self.initialized = False
        self.wL = 0.0
        self.wR = 0.0

        # --- Metrics Logging ---
        self.metrics_list = []
        self.log_file = 'dead_reckoning_results.csv'
        self.total_dist = 0.0
        self.last_pos = None
        self.imu_yaw_raw = 0.0
        
        self.get_logger().info(f'Odometry Logger initialized. Saving to {self.log_file}')

    def imu_cb(self, msg):
        q = msg.orientation
        _, _, self.imu_yaw_raw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        # ในระบบนี้เราใช้ Yaw จาก IMU โดยตรงเป็นหลัก
        self.yaw = self.imu_yaw_raw

    def joint_cb(self, msg):
        # ป้องกันค่าเริ่มต้น
        if len(msg.velocity) < 2: return
        
        self.wL = msg.velocity[0]
        self.wR = msg.velocity[1]

        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now
        
        if not self.initialized:
            self.initialized = True
            return
        if dt <= 0: return

        # 1. คำนวณความเร็ว
        vL = self.wheel_radius * self.wL
        vR = self.wheel_radius * self.wR
        v = 0.5 * (vR + vL)
        w_enc = (vR - vL) / self.shaft # ความเร็วเชิงมุมจากล้ออย่างเดียว

        # 2. Update Position (IMU + Encoders)
        self.x += v * math.cos(self.yaw) * dt
        self.y += v * math.sin(self.yaw) * dt

        # 3. Update Raw Odometry (สำหรับคำนวณ Drift)
        self.raw_x += v * math.cos(self.raw_yaw) * dt
        self.raw_y += v * math.sin(self.raw_yaw) * dt
        self.raw_yaw += w_enc * dt

        drift = math.sqrt((self.x - self.raw_x)**2 + (self.y - self.raw_y)**2)
        
        # Robustness Proxy: ความต่างของการหมุน (IMU vs Encoders)
        # ถ้าค่านี้สูงแปลว่าล้ออาจจะลื่น (Slip) หรือ IMU มี Noise
        yaw_diff = abs(math.atan2(math.sin(self.yaw - self.raw_yaw), math.cos(self.yaw - self.raw_yaw)))
        
        if self.last_pos is not None:
            self.total_dist += math.sqrt((self.x - self.last_pos[0])**2 + (self.y - self.last_pos[1])**2)
        self.last_pos = (self.x, self.y)

        self.metrics_list.append({
            'timestamp': now.nanoseconds * 1e-9,
            'accuracy_proxy': yaw_diff,       # บอกความสอดคล้องของเซนเซอร์
            'drift_error': drift,             # บอกการสะสมความเพี้ยน
            'robustness_v': v,                # ดูความนิ่งของความเร็ว
            'dist_travelled': self.total_dist
        })
        if len(self.metrics_list) % 100 == 0:
            pd.DataFrame(self.metrics_list).to_csv(self.log_file, index=False)
        self.publish_data(now)

    def publish_data(self, now):
        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        qz = quaternion_from_euler(0, 0, self.yaw)
        t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w = qz
        self.br.sendTransform(t)

        pose = PoseStamped()
        pose.header = t.header
        pose.pose.position.x, pose.pose.position.y = self.x, self.y
        pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w = qz
        self.path.poses.append(pose)
        self.path_pub.publish(self.path)

def main():
    rclpy.init()
    node = SensorReader()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.metrics_list:
            pd.DataFrame(node.metrics_list).to_csv(node.log_file, index=False)
            print(f"Final data saved to {node.log_file}")
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()

if __name__ == '__main__':
    main()
