from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_action_client',
            executable='custom_aciton_client_executable',
            output='screen'),
    ])