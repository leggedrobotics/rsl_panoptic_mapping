from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    # Get the share directory of each relevant package
    dynamic_mapping_share_dir = get_package_share_directory('dynamic_mapping')
    print("dynamic_mapping_share_dir: ", dynamic_mapping_share_dir)
    ground_remover_share_dir = get_package_share_directory('ground_remover')
    message_matching_share_dir = get_package_share_directory('message_matching')
    #m545_urdf_share_dir = get_package_share_directory('m545_urdf')
    #m545_rviz_share_dir = get_package_share_directory('m545_rviz')
    # Build relative (package-based) paths 
    default_dynamic_mapping_config = os.path.join(dynamic_mapping_share_dir, 'config', 'config.yaml')
    default_ground_removal_config = os.path.join(ground_remover_share_dir, 'config', 'config.yaml')
    default_message_matching_config = os.path.join(message_matching_share_dir, 'config', 'config.yaml')
    default_rviz_config = os.path.join(dynamic_mapping_share_dir, 'rviz', 'rviz.rviz')

    print("default_rviz_config: ", default_rviz_config)
    
    
    
    # Declare launch arguments
    publish_tf_arg = DeclareLaunchArgument(
        'publish_tf', default_value='false', description='Whether to publish tf'
    )
    config_file_arg = DeclareLaunchArgument(
        'config_file', default_value=default_dynamic_mapping_config,
        description='Path to the dynamic mapping config file'
    )
    ground_removal_config_file_arg = DeclareLaunchArgument(
        'ground_removal_config_file', default_value=default_ground_removal_config,
        description='Path to the ground removal config file'
    )
    message_matching_file_arg = DeclareLaunchArgument(
        'message_matching_config_file', default_value=default_message_matching_config,
        description='Path to the message matching config file'
    )
    ground_removal_voxel_size_arg = DeclareLaunchArgument(
        'ground_removal_voxel_size', default_value='0.15', description='Ground removal voxel size'
    )
    launch_rviz_arg = DeclareLaunchArgument(
        'launch_rviz', default_value='true', description='Whether to launch RViz'
    )
    launch_prefix_arg = DeclareLaunchArgument(
        'launch_prefix', 
        default_value='',  # Empty by default
        description='Launch prefix for debugging. To debug with gdb use: "gdb -ex run -ex bt -ex quit --args"'
    )

    # Define group actions for tf publishing
    tf_group = GroupAction(
        condition=IfCondition(LaunchConfiguration('publish_tf')),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        os.getenv('AMENT_PREFIX_PATH').split(os.pathsep)[0], 'm545_urdf', 'launch', 'load.launch.py'
                    )
                )
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        os.getenv('AMENT_PREFIX_PATH').split(os.pathsep)[0], 'm545_rviz', 'launch', 'tf_publisher.launch.py'
                    )
                )
            )
        ]
    )

    # Define nodes
    dynamic_mapping_node = Node(
        package='dynamic_mapping',
        executable='dynamic_mapping_node',
        name='dynamic_mapping',
        output='screen',
        parameters=[
            {'config_file': LaunchConfiguration('config_file')},
            {'ground_removal_config_file': LaunchConfiguration('ground_removal_config_file')},
            {'ground_removal_voxel_size': LaunchConfiguration('ground_removal_voxel_size')},
            {'use_sim_time': True},
        ],
        prefix=LaunchConfiguration('launch_prefix')
    )

    # Define nodes
    message_matching_node = Node(
        package='message_matching',
        executable='message_matching_node',
        name='message_matching',
        output='screen',
        parameters=[LaunchConfiguration('message_matching_config_file')],
    )

    # Define the RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz_dynamic_mapping',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        condition=IfCondition(LaunchConfiguration('launch_rviz'))
    )

    # For convenience, either declare an argument for the RViz config or just use default:
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=default_rviz_config,
        description='Path to the RViz config file.'
    )

    # Assemble the launch description
    return LaunchDescription([
        publish_tf_arg,
        config_file_arg,
        ground_removal_config_file_arg,
        ground_removal_voxel_size_arg,
        launch_rviz_arg,
        launch_prefix_arg,
        message_matching_file_arg,
        rviz_config_arg,
        tf_group,
        dynamic_mapping_node,
        rviz_node,
        message_matching_node
    ])