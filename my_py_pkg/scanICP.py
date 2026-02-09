import rclpy
from rclpy.node import Node
import tf2_ros
import numpy as np
from tf_transformations import euler_from_quaternion
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data
from scipy.spatial import cKDTree
import math
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path


# --- ฟังก์ชัน ICP ที่เสถียรขึ้น ---
def icp_2d(X, Y, init_R=None, init_t=None, max_iter=30, tol=1e-5):
    """
    X = prev scan
    Y = curr scan
    return ΔR, Δt  (prev → curr)
    """
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

class ICP(Node):
    def __init__(self):
        super().__init__('icp_node')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.create_subscription(LaserScan, '/scan', self.scan_cb, qos_profile_sensor_data)
        
        self.prev_points = None
        self.prev_ekf_pose = None 
        
        self.path_pub = self.create_publisher(Path, '/icp_path', 10)
        self.path_msg = Path()
        self.path_msg.header.frame_id = 'odom'

        # Global Pose (สะสมจาก ICP)
        self.current_global_R = np.eye(2)
        self.current_global_t = np.zeros((2, 1))
        self.current_yaw = 0.0
        self.current_pos_x = 0.0
        self.current_pos_y = 0.0

    def get_ekf_pose(self):
        try:
            # ใช้เวลาปัจจุบันที่สุด
            t = self.tf_buffer.lookup_transform('odom', 'base_link', rclpy.time.Time())
            x = t.transform.translation.x
            y = t.transform.translation.y
            q = t.transform.rotation
            yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
            return np.array([x, y, yaw])
        except Exception:
            return None

    def scan_cb(self, msg: LaserScan):
    # --- Scan → points ---
        angles = msg.angle_min + np.arange(len(msg.ranges)) * msg.angle_increment
        ranges = np.array(msg.ranges)
        valid = np.isfinite(ranges)
        x = ranges[valid] * np.cos(angles[valid])
        y = ranges[valid] * np.sin(angles[valid])
        curr_pts = np.vstack((x, y)).T

        curr_ekf = self.get_ekf_pose()
        if curr_ekf is None:
            return

        if self.prev_points is None:
            self.prev_points = curr_pts
            self.prev_ekf_pose = curr_ekf
            return

        # --- EKF initial guess (prev → curr) ---
        dx = curr_ekf[0] - self.prev_ekf_pose[0]
        dy = curr_ekf[1] - self.prev_ekf_pose[1]
        dyaw = curr_ekf[2] - self.prev_ekf_pose[2]
        dyaw = math.atan2(math.sin(dyaw), math.cos(dyaw))

        c, s = np.cos(self.prev_ekf_pose[2]), np.sin(self.prev_ekf_pose[2])
        init_t = np.array([[ dx * c + dy * s],
                        [-dx * s + dy * c]])

        init_R = np.array([
            [np.cos(dyaw), -np.sin(dyaw)],
            [np.sin(dyaw),  np.cos(dyaw)]
        ])

        # --- ICP ---
        R_delta, t_delta, _, err = icp_2d(
            self.prev_points, curr_pts,
            init_R, init_t
        )

        # --- Update global pose ---
        self.current_global_t += self.current_global_R @ t_delta
        self.current_global_R = self.current_global_R @ R_delta

        self.current_yaw = math.atan2(
            self.current_global_R[1, 0],
            self.current_global_R[0, 0]
        )

        self.current_pos_x = self.current_global_t[0, 0]
        self.current_pos_y = self.current_global_t[1, 0]

        # --- store ---
        self.prev_points = curr_pts
        self.prev_ekf_pose = curr_ekf

        self.publish_path()

    def publish_path(self):
        pose = PoseStamped()
        pose.header = self.path_msg.header
        pose.header.stamp = self.get_clock().now().to_msg()
        
        pose.pose.position.x = float(self.current_global_t[0, 0])
        pose.pose.position.y = float(self.current_global_t[1, 0])
        
        yaw = math.atan2(self.current_global_R[1, 0], self.current_global_R[0, 0])
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)

        self.path_msg.poses.append(pose)
        print(f'x= {float(self.current_global_t[0, 0])} ,y= {float(self.current_global_t[1, 0])} theta= {self.current_yaw}')
        self.path_pub.publish(self.path_msg)

def main():
    rclpy.init()
    node = ICP()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
