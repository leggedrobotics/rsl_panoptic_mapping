from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ellipsoid_fit',
            executable='ellipsoid_fit_node',
            name='ellipsoid_fit_node',
            output='screen',
            parameters=[
                {'input_topic': '/new_cloud/dynamic'},
                {'output_topic': '/ellipsoid_axes'}
            ]
        )
    ])