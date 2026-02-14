from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    ekf_node = Node(
        package='my_py_pkg',
        executable='test_ekf',   # ชื่อตาม entry_points
        name='ekf_node',
        output='screen',
        parameters=[
            {
                'use_sim_time': False
            }
        ]
    )

    icp_node = Node(
        package='my_py_pkg',
        executable='scan_icp',   # ชื่อตาม entry_points
        name='icp_node',
        output='screen',
        parameters=[
            {
                'use_sim_time': False
            }
        ]
    )

    return LaunchDescription([
        ekf_node,
        icp_node
    ])
