from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='basics',
            executable='launch_example',
            name='launch_example',
            namespace='wego',
            output='screen',
            parameters=[
                {'topic_name': 'some_topic'}
            ]
        ),
        Node(
            package='basics',
            executable='talker',
            name='talker',
            output='screen'
        )
    ])