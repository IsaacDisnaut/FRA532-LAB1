import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # 1. ตำแหน่งไฟล์ Config (แก้ 'my_package' เป็นชื่อแพ็กเกจของคุณ)
    slam_config_path = os.path.join(
        get_package_share_directory('my_py_pkg'),
        'config',
        'slam_toolbox_config.yaml'
    )

    # 2. Node: SLAM Toolbox
    # ทำหน้าที่สร้าง Map และคำนวณตำแหน่ง (Odom/Path) จาก Scan
    start_slam_toolbox_node = Node(
        parameters=[
            slam_config_path,
            {'use_sim_time': use_sim_time}
        ],
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen'
    )

    # 3. Node: Robot State Publisher 
    # สำคัญมาก: ต้องมีไฟล์ URDF เพื่อบอกตำแหน่งของ /scan เทียบกับตัวหุ่น
    # (ในตัวอย่างนี้สมมติว่าคุณมีไฟล์ urdf อยู่)
    # start_robot_state_publisher = Node(...) 

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (bag) clock if true'),
        start_slam_toolbox_node,
    ])