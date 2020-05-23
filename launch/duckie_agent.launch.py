from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='gym_duckietown_ros2_agent',
            node_namespace='rosagent',
            node_executable='rosagent',
            node_name='rosagent'
        ),
        Node(
            package='gym_duckietown_ros2_agent',
            node_namespace='rosagent',
            node_executable='lfcmd',
            node_name='lfcmd'
        )
    ])