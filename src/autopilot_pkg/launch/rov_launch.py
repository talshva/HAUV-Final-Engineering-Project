from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='autopilot_pkg',
            executable='guidance_node',
            name='guidance_node',
            output='screen'
        ),
        ExecuteProcess(
            cmd=['ros2', 'run', 'micro_ros_agent', 'micro_ros_agent', 'serial', '-b', '115200', '--dev', '/dev/ttyUSB0'],
            output='screen'
        )
    ])
