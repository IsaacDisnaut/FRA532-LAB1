import rclpy
from rclpy.node import Node
import tf2_ros
import numpy as np
from tf_transformations import euler_from_quaternion
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from scipy.spatial import cKDTree
import math
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Path, OccupancyGrid
from tf2_ros import StaticTransformBroadcaster

# --- ฟังก์ชัน ICP คงเดิม ---
def icp_2d(X, Y, init_R=None, init_t=None, max_iter=30, tol=1e-5):
    R_total = np.eye(2)
    t_total = np.zeros((2, 1))
    X_work = X.copy()
    
    if init_R is not None and init_t is not None:
        X_work = (init_R @ X_work.T).T + init_t.T
        R_total = init_R @ R_total
        t_total = init_R @ t_total + init_t

    tree = cKDTree(Y)
    prev_err = np.inf

    for _ in range(max_iter):
        dists, idx = tree.query(X_work)
        Y_corr = Y[idx]
        x0 = np.mean(X_work, axis=0)
        y0 = np.mean(Y_corr, axis=0)
        H = (X_work - x0).T @ (Y_corr - y0)
        U, _, Vt = np.linalg.svd(H)
        R = Vt.T @ U.T
        if np.linalg.det(R) < 0:
            Vt[1, :] *= -1
            R = Vt.T @ U.T
        t = (y0 - R @ x0).reshape(2, 1)
        X_work = (R @ X_work.T).T + t.T
        R_total = R @ R_total
        t_total = R @ t_total + t
        err = np.mean(dists**2)
        if abs(prev_err - err) < tol:
            break
        prev_err = err
    return R_total, t_total, X_work, prev_err

class ICPMapper(Node):
    def __init__(self):
        super().__init__('icp_mapper_node')
        
        # TF & Buffer
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Map & Path Publishers
        map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', map_qos)
        self.path_pub = self.create_publisher(Path, '/icp_path', 5)
        
        # Subscription
        self.create_subscription(LaserScan, '/scan', self.scan_cb, qos_profile_sensor_data)
        
        # State Variables
        self.prev_points = None
        self.prev_ekf_pose = None 
        self.current_global_R = np.eye(2)
        self.current_global_t = np.zeros((2, 1))
        self.current_yaw = 0.0
        
        # Map Config
        self.res = 0.05
        self.map_width = 800
        self.map_height = 800
        self.origin_x = -20.0
        self.origin_y = -20.0
        self.grid = np.full((self.map_height, self.map_width), -1, dtype=np.int8)
        
        # Path Init
        self.path_msg = Path()
        self.path_msg.header.frame_id = 'map'
        
        # TF Static
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        self.send_static_tf()

        # Update Thresholds (เพื่อไม่ให้แมพทำงานหนักเกินไป)
        self.last_map_t = self.current_global_t.copy()
        self.last_map_yaw = 0.0

    def send_static_tf(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'odom'
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.w = 1.0
        self.static_tf_broadcaster.sendTransform(t)

    def bresenham(self, x0, y0, x1, y1):
        points = []
        dx, dy = abs(x1 - x0), abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        while True:
            points.append((x0, y0))
            if x0 == x1 and y0 == y1: break
            e2 = 2 * err
            if e2 > -dy: err -= dy; x0 += sx
            if e2 < dx: err += dx; y0 += sy
        return points

    def update_map(self, robot_pos, world_pts):
        rx = int((robot_pos[0] - self.origin_x) / self.res)
        ry = int((robot_pos[1] - self.origin_y) / self.res)

        for p in world_pts:
            mx = int((p[0] - self.origin_x) / self.res)
            my = int((p[1] - self.origin_y) / self.res)

            if 0 <= mx < self.map_width and 0 <= my < self.map_height:
                # Ray tracing for free space
                cells = self.bresenham(rx, ry, mx, my)
                for cx, cy in cells[:-1]:
                    if 0 <= cx < self.map_width and 0 <= cy < self.map_height:
                        if self.grid[cy, cx] == -1: # เฉพาะพื้นที่ไม่รู้จัก
                            self.grid[cy, cx] = 0
                
                # Mark obstacle
                self.grid[my, mx] = 100

    def scan_cb(self, msg: LaserScan):
        # 1. Convert Scan to Points
        angles = msg.angle_min + np.arange(len(msg.ranges)) * msg.angle_increment
        ranges = np.array(msg.ranges)
        valid = np.isfinite(ranges) & (ranges > msg.range_min) & (ranges < msg.range_max)
        curr_pts = np.vstack((ranges[valid] * np.cos(angles[valid]), 
                              ranges[valid] * np.sin(angles[valid]))).T

        curr_ekf = self.get_ekf_pose()
        if curr_ekf is None or self.prev_points is None:
            self.prev_points = curr_pts
            self.prev_ekf_pose = curr_ekf
            return
        if self.prev_ekf_pose is None or curr_ekf is None:
            self.get_logger().info('Waiting for first EKF pose...')
            self.prev_ekf_pose = curr_ekf  # เก็บค่าแรกไว้เป็นฐาน
            return  # จบการทำงาน callback นี้ไปก่อน รอข้อมูลรอบหน้า

        # 2. Initial Guess from EKF
        dx, dy = curr_ekf[0] - self.prev_ekf_pose[0], curr_ekf[1] - self.prev_ekf_pose[1]
        dyaw = math.atan2(math.sin(curr_ekf[2]-self.prev_ekf_pose[2]), math.cos(curr_ekf[2]-self.prev_ekf_pose[2]))
        
        c, s = np.cos(self.prev_ekf_pose[2]), np.sin(self.prev_ekf_pose[2])
        init_t = np.array([[dx*c + dy*s], [-dx*s + dy*c]])
        init_R = np.array([[np.cos(dyaw), -np.sin(dyaw)], [np.sin(dyaw), np.cos(dyaw)]])

        # 3. ICP Matching
        R_delta, t_delta, _, _ = icp_2d(self.prev_points, curr_pts, init_R, init_t)

        # 4. Update Global State
        self.current_global_t += self.current_global_R @ t_delta
        self.current_global_R = self.current_global_R @ R_delta
        self.current_yaw = math.atan2(self.current_global_R[1,0], self.current_global_R[0,0])

        # 5. Map Update (เมื่อมีการเคลื่อนที่)
        dist_moved = np.linalg.norm(self.current_global_t - self.last_map_t)
        angle_moved = abs(self.current_yaw - self.last_map_yaw)

        if dist_moved > 0.05 or angle_moved > 0.03:
            world_pts = (self.current_global_R @ curr_pts.T).T + self.current_global_t.T
            self.update_map(self.current_global_t.flatten(), world_pts)
            self.last_map_t = self.current_global_t.copy()
            self.last_map_yaw = self.current_yaw
            self.publish_map()

        # 6. Publish Path & State
        self.publish_path()
        self.prev_points, self.prev_ekf_pose = curr_pts, curr_ekf

    def get_ekf_pose(self):
        try:
            t = self.tf_buffer.lookup_transform('odom', 'base_link', rclpy.time.Time())
            q = t.transform.rotation
            yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
            return np.array([t.transform.translation.x, t.transform.translation.y, yaw])
        except: return None

    def publish_map(self):
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.info.resolution = self.res
        msg.info.width = self.map_width
        msg.info.height = self.map_height
        msg.info.origin.position.x = self.origin_x
        msg.info.origin.position.y = self.origin_y
        msg.data = self.grid.flatten().tolist()
        self.map_pub.publish(msg)

    def publish_path(self):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(self.current_global_t[0, 0])
        pose.pose.position.y = float(self.current_global_t[1, 0])
        pose.pose.orientation.z = math.sin(self.current_yaw / 2.0)
        pose.pose.orientation.w = math.cos(self.current_yaw / 2.0)
        self.path_msg.poses.append(pose)
        self.path_pub.publish(self.path_msg)

def main():
    rclpy.init()
    node = ICPMapper()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: rclpy.shutdown()

if __name__ == '__main__':
    main()

if __name__ == '__main__':
    main()
