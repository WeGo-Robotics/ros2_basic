from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='basics',
            executable='parameter_example',
            name='change_parameter',
            parameters=[
                {'message_content': 'changed contents'}
            ]
        ),
        Node(
            package='basics',
            executable='parameter_example',
            name='remap_topic',
            remappings=[
                ('example_topic', 'another_topic'),
            ]
        ),
        Node(
            package='basics',
            executable='parameter_example',
            namespace='wego',
        ),
    ])