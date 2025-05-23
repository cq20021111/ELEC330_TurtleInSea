import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import ExecuteProcess
from launch.actions import TimerAction  # For delayed node launch


# Function to generate the launch description
def generate_launch_description():

    # Check if we're told to use simulation time; use_sim_time is a launch argument
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Get the path to the Teri_bot package share directory
    package_share_dir = get_package_share_directory('Teri_bot')

    # Specify the relative path to the URDF file
    path_to_urdf = os.path.join(package_share_dir, 'urdf', 'Teri_urdf.sdf')
    
    path_to_world = os.path.join(package_share_dir, 'urdf', 'Atlantis.world')

    # Using the SLAM-specific RViz configuration file
    path_to_rviz = os.path.join(package_share_dir, 'rviz', 'slam.rviz')
    
    gz_world = f"-r -v 4 {path_to_world}"
    
    # Path to the SLAM configuration file
    slam_config_path = os.path.join(package_share_dir, 'config', 'slam_params.yaml')

    # Set the resource path to the workspace source directory
    resource_path = get_package_share_directory('Teri_bot')
    set_gz_sim_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=resource_path + '/../'
    )

    # Create a robot_state_publisher node to publish the robot's state
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': ParameterValue(Command(['xacro ', str(path_to_urdf)]), value_type=str)
        },
        {'use_sim_time': True},
        {'publish_frequency':50.0},
        {'frame_prefix':''}]
    )

    node_joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[
            {'use_sim_time': True},
            {'source_list':['/joint_states']},
            {'rate':50}
        ]
    )

    # Launch Gazebo simulator with an empty world
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ros_gz_sim"),
                "launch",
                "gz_sim.launch.py"
            )
        ),
        launch_arguments={"gz_args": gz_world }.items(),
    )

    # Spawn the robot in Gazebo using the create service equivalent in ros_gz_sim
    spawn_entity = Node(
        package="ros_gz_sim",  # Use the ros_gz_sim package
        executable="create",  # The 'create' executable to spawn models in Gazebo
        arguments=[
            "-name", "robot1",  # The name of the robot in the Gazebo simulation
            "-file", path_to_urdf,  # Path to the URDF file that defines the robot
            "-x", "0", "-y", "0", "-z", "1.05",  # Initial position of the robot in the Gazebo world (x, y, z)
            "-P", "-1.5708",  # Rotate the robot 90 degrees around the Z-axis
            "-Y", "3.1416"
        ],
        output="screen",  # Print the output to the screen
    )

    # Add the ros_gz_bridge node to bridge topics between ROS 2 and Gazebo
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        arguments=[
            # Topic bridges
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/joint2_move@std_msgs/msg/Float64@gz.msgs.Double',
            '/joint3_move@std_msgs/msg/Float64@gz.msgs.Double',
            '/joint4_move@std_msgs/msg/Float64@gz.msgs.Double',
            '/joint5_move@std_msgs/msg/Float64@gz.msgs.Double',
            '/model/robot1/joint/pp2_joint/cmd_thrust@std_msgs/msg/Float64@gz.msgs.Double',
            '/model/robot1/joint/pp3_joint/cmd_thrust@std_msgs/msg/Float64@gz.msgs.Double',
            '/model/robot1/joint/pp4_joint/cmd_thrust@std_msgs/msg/Float64@gz.msgs.Double',
            '/model/robot1/joint/pp5_joint/cmd_thrust@std_msgs/msg/Float64@gz.msgs.Double',
            '/model/robot1/joint/ppt1_joint/cmd_thrust@std_msgs/msg/Float64@gz.msgs.Double',
            '/model/robot1/joint/ppt2_joint/cmd_thrust@std_msgs/msg/Float64@gz.msgs.Double',
            '/cmd_vel@geometry_msgs/msg/Twist[gz.msgs.Twist',
            '/scan/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            '/camera1_image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/red_circle_detected@std_msgs/msg/Bool',
            '/red_circle_info@std_msgs/msg/String',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/map@nav_msgs/msg/OccupancyGrid',
            # '/map_initializer_status@std_msgs/msg/String',
            '/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
            '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
        ],
        output='screen'
    )
   
    # LiDAR converter node - converts 3D point cloud to 2D laser scan data
    lidar_converter = Node(
        package='Teri_bot',
        executable='lidar_converter',
        name='lidar_converter',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'tf_buffer_duration': 2.0}, 
            {'transform_tolerance': 1.0},
        ]  # Publish in at same time as the cache 
    )

    #SLAM toolbox
    slam_toolbox_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
        os.path.join(
        get_package_share_directory('slam_toolbox'),
        'launch',
        'online_async_launch.py'
        )
        ),
        launch_arguments={
        'slam_params_file': slam_config_path,
        'use_sim_time': 'true'
        }.items()
    )

    # SLAM continuous updater 
    slam_continuous_updater = Node(
        package='Teri_bot',
        executable='slam_continuous_updater',
        name='slam_continuous_updater',
        output='screen'
    )

    # Joint controller node
    joint_controller_node = Node(
        package='Teri_bot',
        executable='joint_controller',
        name='joint_controller',
        output='screen'
    )

    #Object Recogniser
    object_recognizer_node = ExecuteProcess(
        cmd=[
        'gnome-terminal', '--', 'bash', '-c',
        'ros2 run Teri_bot object_recognizer; exec bash'
        ],
        output='screen'
        )

    # RViz node - delayed start to ensure TF and map are available
    rviz_node = TimerAction(
        period=3.0,  # Delay 3 seconds
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', path_to_rviz],
                parameters=[
                    {'use_sim_time': use_sim_time},
                    {'tf_buffer_duration': 10.0}
                ],
                output='screen'
            )
        ]
    )
  
    #LiDAR base
    stsatic_lidar_2_base = Node(
        package='tf2_ros',
        executable= 'static_transform_publisher',
        name='static_lidar',
        arguments=['0', '0', '0.29', '0', '-1.5708', '3.14159','base_footprint','lidar_base'],
        parameters=[{'use_sim_time':True}]
    )

    # Return the full launch description with properly ordered components
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time if true'
        ),

        # Set environment variable
        set_gz_sim_resource_path,

        # First launch base infrastructure
        node_robot_state_publisher,
        node_joint_state_publisher,
        
             
        # Then simulation environment
        gz_sim,
        spawn_entity,
        bridge,
        stsatic_lidar_2_base,
        
        # SLAM with proper delay
        lidar_converter,
        slam_toolbox_node,
        slam_continuous_updater,
        
        # Visualization and control
        rviz_node,
        joint_controller_node,
        object_recognizer_node,
          
    ]
)

