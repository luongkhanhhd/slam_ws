import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_path = get_package_share_directory('my_slam_package')
    
    nav2_bringup = Node(
        package='nav2_bringup',
        executable='bringup_launch.py',
        name='nav2_bringup',
        output='screen',
        parameters=[os.path.join(pkg_path, 'config', 'nav2_params.yaml')]
    )

    return LaunchDescription([
        nav2_bringup
    ])
