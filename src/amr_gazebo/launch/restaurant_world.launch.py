import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, AppendEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_amr_description = get_package_share_directory('amr_description')
    pkg_amr_gazebo = get_package_share_directory('amr_gazebo')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Path to xacro and world files
    xacro_file = os.path.join(pkg_amr_description, 'urdf', 'amr.urdf.xacro')
    default_world_path = os.path.join(pkg_amr_gazebo, 'worlds', 'restaurant.world')
    models_path = os.path.join(pkg_amr_gazebo, 'models')

    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    world = LaunchConfiguration('world')
    gui = LaunchConfiguration('gui')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    z_pose = LaunchConfiguration('z_pose')
    yaw_pose = LaunchConfiguration('yaw_pose')

    # Declare arguments
    declare_gui_cmd = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Whether to start Gazebo GUI client (set false for ultra-fast headless)'
    )

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value=default_world_path,
        description='Full path to world model file to load'
    )

    declare_x_pose_cmd = DeclareLaunchArgument(
        'x_pose',
        default_value='2.6',
        description='Initial x position of AMR in world (Kitchen Home Dock)'
    )

    declare_y_pose_cmd = DeclareLaunchArgument(
        'y_pose',
        default_value='3.5',
        description='Initial y position of AMR in world (Kitchen Home Dock)'
    )

    declare_z_pose_cmd = DeclareLaunchArgument(
        'z_pose',
        default_value='0.05',
        description='Initial z position of AMR in world'
    )

    declare_yaw_pose_cmd = DeclareLaunchArgument(
        'yaw_pose',
        default_value='-1.5708',
        description='Initial yaw orientation of AMR in world (-1.5708 rad = facing South towards doorway)'
    )

    # Combine resource paths so Gazebo Sim resolves model://restaurant and model://amr_description
    gz_resource_paths = [
        models_path,
        os.path.dirname(pkg_amr_description),
        pkg_amr_description,
        os.path.dirname(pkg_amr_gazebo),
    ]
    gz_resource_path_str = ':'.join(gz_resource_paths)

    # Directly inject into Python os.environ so ros_gz_sim's launch_gz reads it immediately
    current_gz = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    os.environ['GZ_SIM_RESOURCE_PATH'] = f"{gz_resource_path_str}:{current_gz}" if current_gz else gz_resource_path_str

    current_ign = os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '')
    os.environ['IGN_GAZEBO_RESOURCE_PATH'] = f"{gz_resource_path_str}:{current_ign}" if current_ign else gz_resource_path_str

    # Anti-flicker optimizations for VMware virtualized Qt6 / OGRE2 rendering
    os.environ.setdefault('QSG_RENDER_LOOP', 'basic')
    os.environ.setdefault('QT_QPA_PLATFORM', 'xcb')

    append_gz_resource_path = AppendEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=gz_resource_path_str
    )

    append_ign_resource_path = AppendEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=gz_resource_path_str
    )

    append_qsg_render_loop = AppendEnvironmentVariable(
        name='QSG_RENDER_LOOP',
        value='basic'
    )

    append_qt_platform = AppendEnvironmentVariable(
        name='QT_QPA_PLATFORM',
        value='xcb'
    )

    # Process Xacro to generate robot_description
    robot_description = ParameterValue(
        Command(['xacro ', xacro_file]),
        value_type=str
    )

    # Robot State Publisher Node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time
        }]
    )

    # Launch Gazebo Sim (Harmonic) with restaurant world - GUI mode
    gazebo_sim_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': ['-r ', world]}.items(),
        condition=IfCondition(gui)
    )

    # Launch Gazebo Sim (Harmonic) with restaurant world - Headless (Physics server only)
    gazebo_sim_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': ['-r -s ', world]}.items(),
        condition=UnlessCondition(gui)
    )

    # Spawn AMR entity in Gazebo Sim
    spawn_amr = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_amr',
        output='screen',
        arguments=[
            '-name', 'amr_robot',
            '-topic', 'robot_description',
            '-x', x_pose,
            '-y', y_pose,
            '-z', z_pose,
            '-Y', yaw_pose
        ]
    )

    # ROS-Gz Bridge: Bidirectional topic translation
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        output='screen',
        arguments=[
            # Clock
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            # LiDAR scan
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            # IMU
            '/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU',
            # Drive commands
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            # Odometry
            '/wheel/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            # Joint states (Encoder feedback)
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
        ]
    )

    return LaunchDescription([
        append_gz_resource_path,
        append_ign_resource_path,
        append_qsg_render_loop,
        append_qt_platform,
        declare_gui_cmd,
        declare_use_sim_time,
        declare_world_cmd,
        declare_x_pose_cmd,
        declare_y_pose_cmd,
        declare_z_pose_cmd,
        declare_yaw_pose_cmd,
        robot_state_publisher_node,
        gazebo_sim_gui,
        gazebo_sim_headless,
        spawn_amr,
        bridge
    ])
