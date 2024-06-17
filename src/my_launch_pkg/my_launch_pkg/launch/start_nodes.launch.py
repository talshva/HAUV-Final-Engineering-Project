from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import LogInfo

def generate_launch_description():
    return LaunchDescription([
        LogInfo(msg="Launching ROV nodes..."),
        Node(
            package='micro_ros_agent',
            executable='micro_ros_agent',
            name='micro_ros_agent',
            arguments=['serial', '-b', '115200', '--dev', '/dev/ttyUSB0'],
            output='log'
        ),
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='log'
        ),
        Node(
            package='autopilot_pkg',
            executable='guidance_node',
            name='guidance_node',
            output='log'
        ),
    ])
