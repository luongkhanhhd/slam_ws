import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node, LifecycleNode
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_path = get_package_share_directory('my_slam_package')
    
    rplidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_node',
        name='rplidar_node',
        output='screen',
        parameters=[os.path.join(pkg_path, 'config', 'lidar.yaml')],
    )

    slam_node = LifecycleNode(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        namespace='',
        output='screen',
        parameters=[os.path.join(pkg_path, 'config', 'slam_params.yaml')],
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': open(os.path.join(pkg_path, 'urdf', 'robot.urdf')).read()}]
    )

    odom_node = Node(
        package='my_slam_package',
        executable='odom_publisher.py',
        name='odom_publisher',
        output='screen'
    )

    vel_node = Node(
        package='my_slam_package',
        executable='vel_to_serial.py',
        name='vel_to_serial',
        output='screen'
    )

    configure_slam = ExecuteProcess(
        cmd=['ros2', 'lifecycle', 'set', '/slam_toolbox', 'configure'],
        output='screen'
    )

    activate_slam = ExecuteProcess(
        cmd=['ros2', 'lifecycle', 'set', '/slam_toolbox', 'activate'],
        output='screen'
    )

    return LaunchDescription([
        rplidar_node,
        robot_state_publisher,
        odom_node,
        vel_node,
        slam_node,
        TimerAction(period=2.0, actions=[configure_slam]),
        TimerAction(period=4.0, actions=[activate_slam])
    ])
