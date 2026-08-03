from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    parameters = get_package_share_directory("vehicle_controller") + "/config/vehicle_controller.yaml"
    return LaunchDescription([
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "vehicle_controller",
                "--controller-manager", "/controller_manager",
                "--controller-type", "vehicle_controller/VehicleController",
                "--param-file", parameters,
            ],
            output="screen",
        )
    ])
