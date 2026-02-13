Part1:EKF Odometry Fusion


Objective

The objective of this part is to implement an Extended Kalman Filter (EKF) to fuse wheel odometry and IMU measurements in order to obtain a filtered and more reliable odometry estimate compared to raw wheel odometry.
Description

In this part, you will compute wheel odometry from /joint_states and fuse it with IMU measurements from /imu using an EKF. The EKF estimates the robot pose by combining a differential-drive motion model with probabilistic sensor updates.

![Uploading image.png…]()
