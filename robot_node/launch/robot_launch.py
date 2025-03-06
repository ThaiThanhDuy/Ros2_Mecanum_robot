from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_node',
            executable='odometry_publisher_node',
            name='odometry_publisher_node',
            output='screen',
            parameters=[],
            remappings=[],
        ),
        Node(
            package='robot_node',
            executable='cml_vel_node',
            name='cml_vel_node',
            output='screen',
            parameters=[],
            remappings=[],
        ),

        # Add other nodes related to your robot here
    ])