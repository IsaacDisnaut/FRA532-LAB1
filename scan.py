#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data
import numpy as np
import math
from geometry_msgs.msg import TransformStamped, PoseStamped
from tf2_ros import TransformBroadcaster
from tf_transformations import euler_from_quaternion, quaternion_from_euler
from nav_msgs.msg import Path





def scan_to_points(scan):
    """ LaserScan -> Nx2 numpy array """
    angles = scan.angle_min + np.arange(len(scan.ranges)) * scan.angle_increment
    ranges = np.array(scan.ranges)

    mask = np.isfinite(ranges)
    ranges = ranges[mask]
    angles = angles[mask]

    x = ranges * np.cos(angles) #convert Polars coor to Cartesian
    y = ranges * np.sin(angles)

    return np.vstack((x, y)).T


def icp_2d(A, B, max_iter=100):
    """
    A: previous points (Nx2)
    B: current points (Nx2)
    return: dx, dy, dtheta
    """

    src = A.copy()
    dst = B.copy()

    T = np.eye(3)

    for _ in range(max_iter):
        # nearest neighbor (brute force)
        indices = []
        for p in src:
            dists = np.linalg.norm(dst - p, axis=1) #
            indices.append(np.argmin(dists))
        matched_dst = dst[indices]

        # centroids
        mu_src = np.mean(src, axis=0)
        mu_dst = np.mean(matched_dst, axis=0)

        src_centered = src - mu_src
        dst_centered = matched_dst - mu_dst

        # SVD
        W = dst_centered.T @ src_centered
        U, _, VT = np.linalg.svd(W)
        R = U @ VT

        if np.linalg.det(R) < 0:
            U[:, -1] *= -1
            R = U @ VT

        t = mu_dst - R @ mu_src

        # update src
        src = (R @ src.T).T + t

        # update transform
        T_step = np.eye(3)
        T_step[:2, :2] = R
        T_step[:2, 2] = t
        T = T_step @ T

    dx = T[0, 2]
    dy = T[1, 2]
    dtheta = math.atan2(T[1, 0], T[0, 0])

    return dx, dy, dtheta


class ICPNode(Node):

    def __init__(self):
        super().__init__('icp_node')

        self.sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile_sensor_data
        )
        self.br = TransformBroadcaster(self)
        self.prev_points = None
        self.x=0
        self.y=0
        self.theta=0
        self.path_pub = self.create_publisher(Path, '/trajectory', 10)
        self.path = Path()
        self.path.header.frame_id = 'odom'
        self.last_time = self.get_clock().now()

    def scan_callback(self, scan):
        curr_points = scan_to_points(scan)

        if self.prev_points is None:
            self.prev_points = curr_points
            self.last_time = self.get_clock().now()
            return

        dx, dy, dtheta = icp_2d(self.prev_points, curr_points)

        self.get_logger().info(
            f"ICP: dx={dx:.3f}, dy={dy:.3f}, dtheta={math.degrees(dtheta):.2f} deg"
        )

        # integrate pose มาจากการคูณRotation,Translation
        self.x += dx * math.cos(self.theta) - dy * math.sin(self.theta)
        self.y += dx * math.sin(self.theta) + dy * math.cos(self.theta)
        self.theta += dtheta

        now = self.get_clock().now()

        # 🔥 publish TF + Path ตรงนี้
        self.publish_tf_and_path(now)

        self.prev_points = curr_points
        self.last_time = now



    def publish_tf_and_path(self, now):
        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'

        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0

        q = quaternion_from_euler(0, 0, self.theta)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.br.sendTransform(t)

        pose = PoseStamped()
        pose.header = t.header
        pose.pose.position.x = self.x
        pose.pose.position.y = self.y
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]

        self.path.poses.append(pose)
        self.path.header.stamp = now.to_msg()
        self.path_pub.publish(self.path)


def main():
    rclpy.init()
    node = ICPNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
