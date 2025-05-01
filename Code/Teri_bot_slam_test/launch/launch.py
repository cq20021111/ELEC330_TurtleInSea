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
from launch.actions import TimerAction  # 导入TimerAction，用于延迟启动节点


# Function to generate the launch description
def generate_launch_description():

    # Check if we're told to use simulation time; use_sim_time is a launch argument
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Get the path to the Teri_bot package share directory
    package_share_dir = get_package_share_directory('Teri_bot')

    # Specify the relative path to the URDF file
    path_to_urdf = os.path.join(package_share_dir, 'urdf', 'Teri_urdf.sdf')
    
    path_to_world = os.path.join(package_share_dir, 'urdf', 'Atlantis.world')

    # 使用SLAM专用的RViz配置文件
    path_to_rviz = os.path.join(package_share_dir, 'rviz', 'slam.rviz')
    
    gz_world = f"-r -v 4 {path_to_world}"
    
    # Path to the SLAM configuration file
    slam_config_path = os.path.join(package_share_dir, 'config', 'slam_params.yaml')

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

            # 添加LaserScan话题和地图话题的桥接
            '/scan@sensor_msgs/msg/LaserScan',
            '/map@nav_msgs/msg/OccupancyGrid',
            '/map_initializer_status@std_msgs/msg/String',

            # '/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            # '/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
        ],
        output='screen'  # Print the output to the screen
    )
    
    # 激光雷达转换节点 - 将3D点云转换为2D激光扫描数据
    lidar_converter_node = Node(
        package='Teri_bot',
        executable='lidar_converter',
        name='lidar_converter',
        output='screen'
    )
    
    # 地图初始化节点 - 帮助SLAM启动，确保它有足够的资源和高优先级
    map_initializer_node = Node(
        package='Teri_bot',
        executable='map_initializer',
        name='map_initializer',
        output='screen',
        # 设置高的进程优先级，确保能够获取足够的CPU资源
        emulate_tty=True  # 启用颜色输出
    )
    
    # SLAM工具箱节点 - 注意在Jazzy中使用正确的包名，延迟10秒启动
    slam_toolbox_node = TimerAction(
        period=10.0,  # 延迟10秒启动，确保地图初始化节点有足够时间发布初始地图
        actions=[
            Node(
                package='slam_toolbox',  # 确认包名是否在Jazzy中相同
                executable='async_slam_toolbox_node',
                name='slam_toolbox',
                output='screen',
                parameters=[
                    slam_config_path,
                    {'use_sim_time': use_sim_time}
                ],
                # 不需要重映射了，因为map_initializer直接发布到/map话题
            )
        ]
    )

    # Joint controller 节点
    joint_controller_node = Node(
        package='Teri_bot',
        executable='joint_controller',
        name='joint_controller',
        output='screen'
    )

    # Image listener 节点
    image_listener_node = Node(
        package='Teri_bot',
        executable='image_listener',
        name='image_listener',
        output='screen'
    )

    # RViz节点 - 稍微延迟启动，确保地图和TF已准备好
    rviz_node = TimerAction(
        period=3.0,  # 延迟3秒启动
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', path_to_rviz],
                parameters=[{'use_sim_time': use_sim_time}],
                output='screen'
            )
        ]
    )

    # 输出调试信息
    debug_info_node = ExecuteProcess(
        cmd=[
            'bash', '-c',
            'sleep 15 && echo "关键调试信息:" && echo "1. 检查是否有/scan话题:" && ros2 topic info /scan && echo "2. 检查是否有/map话题:" && ros2 topic info /map && echo "3. 检查map_initializer状态:" && ros2 topic echo /map_initializer_status -n 1 && echo "4. 检查TF树:" && ros2 run tf2_tools view_frames.py'
        ],
        output='screen'
    )

    # 保存地图节点 - 提示用户如何保存地图
    map_saver_node = ExecuteProcess(
        cmd=[
            'bash', '-c',
            'sleep 2 && echo "提示: 要保存地图，运行: ros2 run nav2_map_server map_saver_cli -f ~/map"'
        ],
        output='screen'
    )

    # Return the full launch description
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',  # Declare the use_sim_time argument
            default_value='true',  # 使用模拟时间
            description='Use simulation time if true'  # Description of the argument
        ),

        # Set the environment variable for GZ_SIM_RESOURCE_PATH
        set_gz_sim_resource_path,

        # 首先启动基础节点
        node_robot_state_publisher,
        gz_sim,
        spawn_entity,
        bridge,  # Add the ros_gz_bridge node
        
        # 然后启动传感器处理节点
        lidar_converter_node,  # 添加激光雷达转换节点
        
        # 注意：先启动地图初始化节点，然后才是SLAM
        map_initializer_node,  # 添加地图初始化节点
        
        # 然后是控制节点
        joint_controller_node,  # 添加joint_controller节点
        image_listener_node,  # 添加image_listener节点
        
        # 延迟启动SLAM和RViz，确保前面的节点已经准备好
        slam_toolbox_node,  # 添加SLAM工具箱节点
        rviz_node,  # 添加RViz节点
        
        # 最后是辅助节点
        debug_info_node,  # 添加调试信息节点
        map_saver_node,  # 添加地图保存提示
    ])

