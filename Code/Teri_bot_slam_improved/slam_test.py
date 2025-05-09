import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    slam_params_file = os.path.join(
        get_package_share_directory('Teri_bot'),
        'config',
        'minimal_slam_params.yaml'
    )
    
    # Basic SLAM configuration
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_params_file,
            {'use_sim_time': use_sim_time}
        ],
    )
    
    # Transform publisher - ensure map frame exists
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_map_odom_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
    )
    
    # Return the LaunchDescription
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use sim time if true'
        ),
        static_tf_node,
        slam_toolbox_node,
    ])