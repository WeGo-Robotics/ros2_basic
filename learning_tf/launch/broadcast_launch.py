from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments = ['--x', '0', 
                         '--y', '3', 
                         '--z', '0', 
                         '--yaw', '0', 
                         '--pitch', '0', 
                         '--roll', '0', 
                         '--frame-id', 'turtle1', 
                         '--child-frame-id', 'carrot']
        ),
        Node(
            package='learning_tf',
            executable='turtle_one_broadcaster',
            name = 'turtle_one_broadcaster'
        ),
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name = 'turtlesim'
        ),
        Node(
            package='learning_tf',
            executable='turtle_two_broadcaster',
            name = 'turtle_two_broadcaster'
        ),
    ])