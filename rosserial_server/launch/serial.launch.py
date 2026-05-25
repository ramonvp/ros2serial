from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            name='port',
            default_value='/dev/ttyACM0',
            description='Device port for the serial device'),
        DeclareLaunchArgument(
            name='baud',
            default_value='57600',
            description='Port baudrate'),
        Node(
            package="rosserial_server",
            executable="serial_node",
            name="rosserial_server",
            output="screen",
            emulate_tty=True,
            parameters=[
                {"port": LaunchConfiguration('port')},
                {"baud": LaunchConfiguration('baud')}
            ]
        )
    ])
