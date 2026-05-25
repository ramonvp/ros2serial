from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="rosserial_server",
            executable="socket_node",
            name="rosserial_server",
            output="screen",
            emulate_tty=True,
            parameters=[]
        )
    ])
