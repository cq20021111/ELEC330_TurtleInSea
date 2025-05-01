import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import ExecuteProcess


# Function to generate the launch description
def generate_launch_description():

    # Check if we're told to use simulation time; use_sim_time is a launch argument
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Get the path to the Teri_bot package share directory
    package_share_dir = get_package_share_directory('Teri_bot')

    # Specify the relative path to the URDF file
    path_to_urdf = os.path.join(package_share_dir, 'urdf', 'Teri_urdf.sdf')
    
    path_to_world = os.path.join(package_share_dir, 'urdf', 'Atlantis.world')

    path_to_rviz = os.path.join(package_share_dir, 'rviz', 'default.rviz')
    
    gz_world = f"-r -v 4 {path_to_world}"

    # Set the resource path to the workspace source directory
    # resource_path = os.path.join(os.path.expanduser('~'), 'project_ws', 'src')
    resource_path = get_package_share_directory('Teri_bot')
    set_gz_sim_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=resource_path + '/../'
    )



    # Create a robot_state_publisher node to publish the robot's state, especially joint states and TF information
    node_robot_state_publisher = Node(
        package='robot_state_publisher',  # Use the robot_state_publisher package
        executable='robot_state_publisher',  # The executable name
        name='robot_state_publisher',  # The node name
        output='screen',  # Output will be printed to the screen
        parameters=[{
            'robot_description': ParameterValue(Command(['xacro ', str(path_to_urdf)]), value_type=str)  # Generate the robot description from the URDF file
        }]
    )

    # Launch Gazebo simulator with an empty world
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ros_gz_sim"),  # Get the path to the ros_gz_sim package
                "launch",  # Launch folder
                "gz_sim.launch.py"  # The Gazebo launch file
            )
        ),
        launch_arguments={"gz_args": gz_world }.items(),  # Arguments to pass to Gazebo: load empty.sdf with verbose logging
    )

    # Spawn the robot in Gazebo using the create service equivalent in ros_gz_sim
    spawn_entity = Node(
        package="ros_gz_sim",  # Use the ros_gz_sim package
        executable="create",  # The 'create' executable to spawn models in Gazebo
        arguments=[
            "-name", "robot1",  # The name of the robot in the Gazebo simulation
            "-file", path_to_urdf,  # Path to the URDF file that defines the robot
            "-x", "0", "-y", "0", "-z", "2",  # Initial position of the robot in the Gazebo world (x, y, z)
            "-P", "-1.5708",  # Rotate the robot 90 degrees around the Z-axis
            "-Y", "3.1416"
        ],
        output="screen",  # Print the output to the screen
    )

    # Add the ros_gz_bridge node to bridge joint states and commands between ROS 2 and Gazebo
    bridge = Node(
        package='ros_gz_bridge',  # Use the ros_gz_bridge package
        executable='parameter_bridge',  # Run the parameter bridge executable
        name='ros_gz_bridge',  # Name of the bridge node
        arguments=[
            # Bridge the /joint_states topic from ROS 2 (sensor_msgs/JointState) to Gazebo (gz.msgs.Model)
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
            
            # Bridge velocity commands (if controlling with velocity) between ROS 2 and Gazebo
            '/cmd_vel@geometry_msgs/msg/Twist[gz.msgs.Twist',

            '/lidar/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',

            '/camera1_image@sensor_msgs/msg/Image[gz.msgs.Image',

            # 红色圆形检测结果的话题桥接
            '/red_circle_detected@std_msgs/msg/Bool',
            '/red_circle_info@std_msgs/msg/String',

            # '/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            # '/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',

        ],
        output='screen'  # Print the output to the screen
    )

    # Joint controller in a new terminal window
    joint_controller_node = ExecuteProcess(
        cmd=[
            'gnome-terminal', '--', 'bash', '-c', 
            'ros2 run Teri_bot joint_controller; exec bash'
        ],
        output='screen'
    )

    image_listener_node = ExecuteProcess(
        cmd=[
        'gnome-terminal', '--', 'bash', '-c',
        'ros2 run Teri_bot image_listener; exec bash'
        ],
        output='screen'
        )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', path_to_rviz],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # vision_node = Node(
    #     package='Teri_bot',
    #     executable='color_detector_node',
    #     name='color_detector_node',
    #     output='screen'
    # )

    # Return the full launch description
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',  # Declare the use_sim_time argument
            default_value='false',  # Default to false (no simulation time)
            description='Use simulation time if true'  # Description of the argument
        ),

        # Set the environment variable for GZ_SIM_RESOURCE_PATH
        set_gz_sim_resource_path,

        # Start the joint state publisher, robot state publisher, Gazebo simulation, and spawn the robot
        node_robot_state_publisher,
        gz_sim,
        spawn_entity,
        bridge,  # Add the ros_gz_bridge node
        rviz_node,
        joint_controller_node,  # Add joint_controller node
        # vision_node
        image_listener_node,
        
    ])

