from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='vehicle_keyboard_teleop',
            executable='vehicle_keyboard_teleop',
            output='screen',
            emulate_tty=True,
        )
    ])
