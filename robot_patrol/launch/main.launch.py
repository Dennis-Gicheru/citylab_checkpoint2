import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node

def generate_launch_description():

    pkg_share = get_package_share_directory('robot_patrol')
    rviz_config_path = os.path.join(pkg_share, 'rviz', 'patrol_config.rviz')

    # Service server — starts first (topic-free, no remapping)
    direction_service = Node(
        package='robot_patrol',
        executable='direction_service_node',
        output='screen'
    )

    # Patrol node — consumes the service to drive the robot
    patrol = Node(
        package='robot_patrol',
        executable='patrol_with_service_node',
        output='screen'
    )

    # RViz with the saved patrol configuration
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen'
    )

    return LaunchDescription([
        direction_service,
        rviz,
        TimerAction(period=2.0, actions=[patrol])  # patrol starts after server is up
    ])