# Part1:EKF Odometry Fusion

## Objective

The objective of this part is to implement an Extended Kalman Filter (EKF) to fuse wheel odometry and IMU measurements in order to obtain a filtered and more reliable odometry estimate compared to raw wheel odometry.
Description

In this part, you will compute wheel odometry from /joint_states and fuse it with IMU measurements from /imu using an EKF. The EKF estimates the robot pose by combining a differential-drive motion model with probabilistic sensor updates.<br><br>
### Part 1: EKF Odometry Fusion <br>
**Objective** <br>
The objective of this part is to implement an Extended Kalman Filter (EKF) to fuse wheel odometry and IMU measurements in order to obtain a filtered and more reliable odometry estimate compared to raw wheel odometry. <br><br>
**Description** <br>
In this part, you will compute wheel odometry from /joint_states and fuse it with IMU measurements from /imu using an EKF. The EKF estimates the robot pose by combining a differential-drive motion model with probabilistic sensor updates.<br>
### Part 2: ICP Odometry Refinement
**Objective** <br>
The objective of this part is to refine the EKF-based odometry using LiDAR scan matching and evaluate the improvement in accuracy and drift.<br><br>
**Description** <br>
In this part, you will use the EKF odometry from Part 1 as the initial guess for ICP scan matching on consecutive /scan messages. The estimated relative pose from ICP is integrated to produce a LiDAR-based odometry estimate.<br><br>
### Part 3: Full SLAM with slam_toolbox <br>
**Objective**<br>
The objective of this part is to perform full SLAM using slam_toolbox and compare its pose estimation and mapping performance with the ICP-based odometry from Part 2.<br><br>
**Description**<br>
In this part, you will run slam_toolbox using LiDAR data and an odometry source to perform full SLAM with loop closure. The resulting trajectory and map are compared with the ICP-based odometry from Part 2.<br>
## Robot Dimension

<img width="664" height="521" alt="turtlebot3_dimension1" src="https://github.com/user-attachments/assets/2a7e20e4-35c5-40b1-a373-b6a74af9f3b8" /><br>
## Wheel Odometry<br>
### Sequence00 empty hallway<br>
<img width="1668" height="909" alt="RAW00" src="https://github.com/user-attachments/assets/c0f3cef0-20cc-47d0-8994-49bc531f7a72" /> <br>
### Sequence01 Non-Empty Hallway with Sharp Turns <br>
<img width="1121" height="847" alt="Raw01" src="https://github.com/user-attachments/assets/b5041397-a090-407a-8552-ef88b3110e6a" /> <br>
### Sequence 02 – Non-Empty Hallway with Non-Aggressive Motion <br>
<img width="1121" height="847" alt="RAW03" src="https://github.com/user-attachments/assets/a9a86853-0db4-45e2-9363-6d3dfaac9570" /> <br>
## EKF Odometry <br>
### Sequence00 empty hallway<br>
<img width="1668" height="909" alt="EKF00" src="https://github.com/user-attachments/assets/a643ec48-f962-4571-9a3f-4d44f058bcdc" /><br>
## ICP Odometry<br>
## Slam toolbox Odometry<br>


