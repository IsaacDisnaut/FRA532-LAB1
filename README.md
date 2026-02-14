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
<img width="575" height="301" alt="Pasted image" src="https://github.com/user-attachments/assets/010f78ce-dd9e-4e90-8476-c6645b207dbc" /><br>
<img width="242" height="131" alt="Pasted image (2)" src="https://github.com/user-attachments/assets/16fa28bd-029a-436d-aa87-54090e6dc90b" /><br>
cr. [Wheeled Mobile Robot Kinematics](https://control.ros.org/iron/doc/ros2_controllers/doc/mobile_robot_kinematics.html)<br><br>
**x += V<sub>b,x</sub>cos(θ)Δt**<br>
**y += V<sub>b,x</sub>sin(θ)Δt**<br>
### Sequence 00 - empty hallway<br>
<img width="1668" height="909" alt="RAW00" src="https://github.com/user-attachments/assets/c0f3cef0-20cc-47d0-8994-49bc531f7a72" /> <br>
### Sequence 01 - Non-Empty Hallway with Sharp Turns <br>
<img width="1121" height="847" alt="Raw01" src="https://github.com/user-attachments/assets/b5041397-a090-407a-8552-ef88b3110e6a" /> <br>
### Sequence 02 – Non-Empty Hallway with Non-Aggressive Motion <br>
<img width="1121" height="847" alt="RAW03" src="https://github.com/user-attachments/assets/a9a86853-0db4-45e2-9363-6d3dfaac9570" /> <br>
## EKF Odometry <br>
<img width="811" height="445" alt="figure-extended-kalman-filter-algorithm" src="https://github.com/user-attachments/assets/0ce2ce0e-2950-4d91-be41-92a0cd2c110b" /><br>
cr. [Wireless Pi](https://wirelesspi.com/the-extended-kalman-filter-ekf/)<br><br>
<img width="352" height="55" alt="Covariance" src="https://github.com/user-attachments/assets/85f2b643-d0da-47f7-bcbf-5a773678472b" /><br>
<img width="367" height="49" alt="noise" src="https://github.com/user-attachments/assets/679026be-a2f3-48a2-b861-6794fcaf51d0" /><br>
<img width="336" height="141" alt="Jacobian_model" src="https://github.com/user-attachments/assets/a1cd3244-28d3-4527-bb2e-26482865f533" /><br>
<img width="102" height="135" alt="imu_ekf" src="https://github.com/user-attachments/assets/9756af09-f096-4cd0-8e0b-a7954d361ca2" /><br><br>
**IMU model**<br>
<img width="104" height="58" alt="imu_ekf2" src="https://github.com/user-attachments/assets/81ba0fcb-620d-4ff4-bb4c-d0f3ccf46a8d" /><br>
<img width="193" height="49" alt="inu_ekf3" src="https://github.com/user-attachments/assets/de9cc832-611e-47d9-a5ee-3471dbf48994" /><br><br>
**Encoder model**<br>
<img width="113" height="40" alt="encoder_ekf" src="https://github.com/user-attachments/assets/3e543ccb-79af-4458-a81f-b18c0419a414" /><br>
<img width="213" height="47" alt="encoder_ekf2" src="https://github.com/user-attachments/assets/37bc2ffd-8a19-4582-b692-4c689864b7db" /><br><br>
**Prediction model**<br>
<img width="177" height="96" alt="Prediction_step" src="https://github.com/user-attachments/assets/8a01aef5-4913-4b6f-bbcd-7700fd81aa06" /><br>

### Sequence 00 - empty hallway<br>
<img width="1668" height="909" alt="EKF00" src="https://github.com/user-attachments/assets/a643ec48-f962-4571-9a3f-4d44f058bcdc" /><br>
### Sequence 01 – Non-Empty Hallway with Sharp Turns<br>
<img width="1218" height="896" alt="EK" src="https://github.com/user-attachments/assets/e22765bb-f51b-407f-b1a8-11537ffe1e98" /><br>
### Sequence 02 – Non-Empty Hallway with Non-Aggressive Motion<br>
<img width="1218" height="896" alt="EKF02" src="https://github.com/user-attachments/assets/f456a3bb-397a-48df-8bee-37fe1988c97a" /><br><br>
## ICP Odometry<br>

**Initial guess**<br>
<img width="157" height="55" alt="Initial_guess_icp" src="https://github.com/user-attachments/assets/457a7f69-ca15-4c80-8362-8e7c7ff594da" /><br>
X<sup>(0)</sup> = Initial position<br>
R<sub>0</sub> = Rotation matrix from EKF<br>
X = Current set of scan point<br>
t<sub>0</sub>=Translation matrix from EKF<br><br>
**Nearest Neighbor Association**<br>
<img width="257" height="53" alt="image" src="https://github.com/user-attachments/assets/4e484252-0b27-4181-a7c5-5f958a1602f2" /><br>
x<sub>(i)</sub><sup>(k)</sup> = Each index of current point for each iteration<br>
y<sub>j</sub> = Each index of previous point<br><br>
<img width="133" height="121" alt="image" src="https://github.com/user-attachments/assets/7e440a09-1ef2-44a3-b3ce-83785e3940d3" /><br><br>
**Compute centroid**<br>
<img width="287" height="79" alt="image" src="https://github.com/user-attachments/assets/94317786-c74f-415a-8243-fb65da97f5dc" /><br>
**Cross covariance matrix**<br>
<img width="245" height="66" alt="image" src="https://github.com/user-attachments/assets/72c24060-968e-4439-b791-95ac0177460b" /><br>
**SVD**<br>
<img width="112" height="55" alt="image" src="https://github.com/user-attachments/assets/acf3dfcc-6908-446f-b8f4-3344b55a67a0" /><br>
in python we can use U, _, V = np.linalg.svd(H) to define U and V<br>
<img width="112" height="55" alt="image" src="https://github.com/user-attachments/assets/6f33d409-a9ef-4f09-aeb4-c78260e17c4f" /><br>
**Optimal translation**<br>
<img width="108" height="47" alt="image" src="https://github.com/user-attachments/assets/dbc9d43b-034c-41c3-b5e0-e2cc593ad781" /><br>
**Point Update**<br>
<img width="177" height="40" alt="image" src="https://github.com/user-attachments/assets/119d3997-801a-447c-9121-6064685939b7" /><br>
**Accumulate trnsform**<br>
<img width="179" height="67" alt="image" src="https://github.com/user-attachments/assets/2e672ed2-accb-48cc-b15a-29620e0ca103" /><br>
**Error between corresponse point**<br>
<img width="316" height="76" alt="image" src="https://github.com/user-attachments/assets/19392e51-304b-4035-91ee-625b7495380c" /><br>
break the loop when k>=max iteration or error < tol

### Sequence 00 - empty hallway<br>
<img width="2063" height="1060" alt="ICP00" src="https://github.com/user-attachments/assets/c18effbd-a9e9-4584-bbe6-a05647c0b575" /><br>
### Sequence 01 – Non-Empty Hallway with Sharp Turns<br>
<img width="1218" height="896" alt="ICP01" src="https://github.com/user-attachments/assets/82239433-c995-4ccb-88d4-7c585994f48f" /><br>
### Sequence 02 – Non-Empty Hallway with Non-Aggressive Motion<br>
<img width="1218" height="896" alt="ICP03" src="https://github.com/user-attachments/assets/364e2ef5-8e1e-4a2f-8f8d-b0d47a0a3fc3" /><br>
## Slam toolbox Odometry<br>
[SLAM_toolbox source code](https://github.com/phattanaratjeedjeen-sudo/FRA532-LAB-6810/tree/lab1/src/lab1)<br>
### Sequence 00 - empty hallway<br>
### Sequence 01 – Non-Empty Hallway with Sharp Turns<br>
### Sequence 02 – Non-Empty Hallway with Non-Aggressive Motion<br>



