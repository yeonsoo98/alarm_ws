from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sound_alarm',
            executable='sound_alarm_node',
            name='sound_alarm_node',
            output='screen'
        ),
        Node(
            package='sound_alarm',
            executable='alarm_publisher',
            name='alarm_publisher',
            output='screen'
        )
    ])
