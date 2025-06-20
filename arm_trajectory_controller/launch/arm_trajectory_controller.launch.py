from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='arm_trajectory_controller',
            executable='arm_trajectory_controller_node',
            name='arm_trajectory_controller_node',
            output='screen',
        ),
    ])
