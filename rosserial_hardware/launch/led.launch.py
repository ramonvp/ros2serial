from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory("rosserial_hardware"))
    urdf = (share / "example" / "led" / "led.urdf").read_text()
    controllers = str(share / "example" / "led" / "controllers.yaml")

    robot_description = {"robot_description": urdf}

    return LaunchDescription([
        # controller_manager reads robot_description from this node's topic.
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[robot_description],
        ),
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[controllers],
            output="screen",
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["gpio_controller"],
        ),
    ])
