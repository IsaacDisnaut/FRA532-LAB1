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
def icp_2d(X, Y, init_R=None, init_t=None, max_iter=25, tol=1e-5):
    """
    หาการแปลง (R, t) ที่พา X ไปหา Y 
    ในบริบทนี้: X = อดีต (prev), Y = ปัจจุบัน (curr)
    ผลลัพธ์ที่ได้คือ Forward Motion ของหุ่นยนต์
    """
    X_working = X.copy()
    if init_R is not None and init_t is not None:
        X_working = (init_R @ X_working.T).T + init_t.T
    
    tree = cKDTree(Y)
    prev_error = np.inf

    for _ in range(max_iter):
        dists, idx = tree.query(X_working)
        Y_corr = Y[idx]

        x0, y0 = np.mean(X_working, axis=0), np.mean(Y_corr, axis=0)
        H = (X_working - x0).T @ (Y_corr - y0)
        U, _, Vt = np.linalg.svd(H)
        R_iter = Vt.T @ U.T
        if np.linalg.det(R_iter) < 0:
            Vt[1, :] *= -1
            R_iter = Vt.T @ U.T
        
        t_iter = (y0 - R_iter @ x0).reshape(2, 1)
        X_working = (R_iter @ X_working.T).T + t_iter.T

        error = np.mean(dists**2)
        if abs(prev_error - error) < tol:
            break
        prev_error = error

    # คำนวณหา Total Transformation จากจุดเริ่มต้น X ไปยัง X_working สุดท้าย
    mu_X = np.mean(X, axis=0)
    mu_X_w = np.mean(X_working, axis=0)
    H_final = (X - mu_X).T @ (X_working - mu_X_w)
    U, _, Vt = np.linalg.svd(H_final)
    R_final = Vt.T @ U.T
    if np.linalg.det(R_final) < 0:
        Vt[1, :] *= -1
        R_final = Vt.T @ U.T
        
    t_final = (mu_X_w - R_final @ mu_X).reshape(2, 1)

    return R_final, t_final, X_working, prev_error

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
        # 1. เตรียมข้อมูล Scan ปัจจุบัน
        angles = msg.angle_min + np.arange(len(msg.ranges)) * msg.angle_increment
        ranges = np.array(msg.ranges)
        valid = np.isfinite(ranges) & (ranges > msg.range_min) & (ranges < msg.range_max)
        x = ranges[valid] * np.cos(angles[valid])
        y = ranges[valid] * np.sin(angles[valid])
        current_points = np.vstack((x, y)).T

        # ดึงค่า EKF ปัจจุบัน
        curr_ekf = self.get_ekf_pose()
        if curr_ekf is None: return

        # กรณีเริ่มครั้งแรก
        if self.prev_points is None:
            self.prev_points = current_points
            self.prev_ekf_pose = curr_ekf
            return
        diff_yaw = curr_ekf[2] - self.prev_ekf_pose[2]
        # จัดการให้มุมอยู่ในช่วง -pi ถึง pi (สำคัญมาก!)
        diff_yaw = math.atan2(math.sin(diff_yaw), math.cos(diff_yaw))
        
        dx = curr_ekf[0] - self.prev_ekf_pose[0]
        dy = curr_ekf[1] - self.prev_ekf_pose[1]

        # 2. คำนวณความแตกต่างจาก EKF
        diff = curr_ekf - self.prev_ekf_pose
        dx, dy, d_yaw = diff[0], diff[1], diff[2]

        # สร้าง Initial Guess (จากอดีตไปปัจจุบัน)
        init_R = np.array([
            [np.cos(d_yaw), -np.sin(d_yaw)],
            [np.sin(d_yaw),  np.cos(d_yaw)]
        ])

        # ลองเปลี่ยนการคำนวณ local_t: 
        # บางครั้ง EKF ให้ค่าใน Frame Odom เราต้อง Rotate กลับมาที่ Frame หุ่นยนต์ตัวก่อนหน้า
        c, s = np.cos(self.prev_ekf_pose[2]), np.sin(self.prev_ekf_pose[2])
        init_t = np.array([
            [dx * c + dy * s],
            [-dx * s + dy * c]
        ])
        c_prev, s_prev = np.cos(self.prev_ekf_pose[2]), np.sin(self.prev_ekf_pose[2])
        init_tx =  dx * c_prev + dy * s_prev
        init_ty = -dx * s_prev + dy * c_prev
        
        init_R = np.array([
            [np.cos(diff_yaw), -np.sin(diff_yaw)],
            [np.sin(diff_yaw),  np.cos(diff_yaw)]
        ])
        init_t = np.array([[init_tx], [init_ty]])

        # 3. รัน ICP
        R_fwd, t_fwd, _, err = icp_2d(current_points,self.prev_points, init_R, init_t)

        # 4. อัปเดต Global Pose (ลองสลับลำดับการคูณ)
        # การเคลื่อนที่ต้องถูกหมุนตามทิศทางปัจจุบันก่อนแล้วค่อยบวกเข้าตำแหน่ง
        self.current_global_t += self.current_global_R @ t_fwd
        self.current_global_R = self.current_global_R @ R_fwd
        delta_yaw = math.atan2(R_fwd[1, 0], R_fwd[0, 0])

        # 5. เก็บค่าไว้ใช้รอบหน้า
        self.prev_points = current_points
        self.prev_ekf_pose = curr_ekf
        c_glob, s_glob = np.cos(self.current_yaw), np.sin(self.current_yaw)
        self.current_pos_x += float(t_fwd[0, 0] * c_glob - t_fwd[1, 0] * s_glob)
        self.current_pos_y += float(t_fwd[0, 0] * s_glob + t_fwd[1, 0] * c_glob)
        self.current_yaw += delta_yaw
        
    # ทำ Normalize current_yaw อีกรอบ
        self.current_yaw = math.atan2(math.sin(self.current_yaw), math.cos(self.current_yaw))
        # 6. ส่ง Path ไป RViz
        self.publish_path()

    def publish_path(self):
        pose = PoseStamped()
        pose.header = self.path_msg.header
        pose.header.stamp = self.get_clock().now().to_msg()
        
        pose.pose.position.x = float(self.current_global_t[0, 0])/10
        pose.pose.position.y = float(self.current_global_t[1, 0])/10
        
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