from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            output='screen'),
        Node(
            package='ros2_move_turtle',
            executable='move_turtle_exec',
            output='screen'),
    ])
