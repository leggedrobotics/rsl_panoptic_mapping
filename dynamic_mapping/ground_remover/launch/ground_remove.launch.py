from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    
    ground_remover_share_dir = get_package_share_directory('ground_remover')
    default_ground_removal_config = os.path.join(ground_remover_share_dir, 'config', 'config.yaml')

    # Declare command-line launch arguments (replacing <arg> tags in ROS 1).
    
    config_filepath_arg = DeclareLaunchArgument(
        'config_filepath',
        default_value= default_ground_removal_config,
        description='Path to ground_remover config file.'
    )
    algorithm_arg = DeclareLaunchArgument(
        'algorithm',
        default_value='0',
        description='Algorithm type.'
    )
    input_topic_arg = DeclareLaunchArgument(
        'input_topic',
        default_value='/ouster_points_self_filtered',
        description='Input point cloud topic.'
    )
    voxel_size_arg = DeclareLaunchArgument(
        'voxel_size',
        default_value='0.15',
        description='Voxel grid leaf size.'
    )
    #patchwork_params_arg = DeclareLaunchArgument(
    #    'patchwork_params_file',
    #    default_value='config/params_ouster128.yaml',
    #    description='Patchwork parameter file.'
    #)

    # ground_remover node (replacing <node pkg="ground_remover" ... />).
    ground_remover_node = Node(
        package='ground_remover',
        executable='ground_remover_node',
        name='ground_remover',
        output='screen',
        # parameters in ROS 2 are passed as a list (of dicts or YAML files).
        parameters=[{
            'config_filepath': LaunchConfiguration('config_filepath'),
            'algorithm':       LaunchConfiguration('algorithm'),
            'input_topic':     LaunchConfiguration('input_topic'),
            'voxel_size':      LaunchConfiguration('voxel_size'),
        }]
    )

    # (Optional) patchwork node that consumes the YAML file
    # in lieu of <rosparam command="load" file="$(find patchwork)/config/params_ouster128.yaml" />
    #patchwork_node = Node(
    #    package='patchwork',
    #    executable='patchwork_node',  # replace with the actual executable name
    #    name='patchwork',
    #    output='screen',
    #    parameters=[LaunchConfiguration('patchwork_params_file')]
    #)

    # Assemble the launch description, similar to listing tags in the XML.
    return LaunchDescription([
        config_filepath_arg,
        algorithm_arg,
        input_topic_arg,
        voxel_size_arg,
        #patchwork_params_arg,
        ground_remover_node,
        #patchwork_node
    ])
