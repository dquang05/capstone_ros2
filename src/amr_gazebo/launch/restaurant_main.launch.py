#!/usr/bin/env python3
"""
Launch Gazebo Sim with the Main (Realistic Blueprint) Restaurant World and AMR
Project: Autonomous Mobile Robot (AMR) for Food Delivery

Usage:
    # Full simulation with robot at kitchen dock pad:
    ros2 launch amr_gazebo restaurant_main.launch.py

    # Inspect world only (without robot):
    ros2 launch amr_gazebo restaurant_main.launch.py spawn_robot:=false

    # Custom initial spawn coordinates:
    ros2 launch amr_gazebo restaurant_main.launch.py x_pose:=0.0 y_pose:=0.0 yaw_pose:=0.0
"""

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

    # Path to xacro and main blueprint world files
    xacro_file = os.path.join(pkg_amr_description, 'urdf', 'amr.urdf.xacro')
    default_world_path = os.path.join(pkg_amr_gazebo, 'worlds', 'restaurant_blueprint.world')
    models_path = os.path.join(pkg_amr_gazebo, 'models')

    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    world = LaunchConfiguration('world')
    gui = LaunchConfiguration('gui')
    spawn_robot = LaunchConfiguration('spawn_robot')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    z_pose = LaunchConfiguration('z_pose')
    yaw_pose = LaunchConfiguration('yaw_pose')

    # Declare arguments
    declare_gui_cmd = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Whether to start Gazebo GUI client (set false for headless mode)'
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

    declare_spawn_robot_cmd = DeclareLaunchArgument(
        'spawn_robot',
        default_value='true',
        description='Whether to spawn AMR robot and start bridge (false for pure world inspection)'
    )

    # In restaurant_blueprint.world, Area 08 Main Hall dock is at (8.85, -3.80, 0.008)
    declare_x_pose_cmd = DeclareLaunchArgument(
        'x_pose',
        default_value='8.85',
        description='Initial x position of AMR in world (Area 08 Main Hall)'
    )

    declare_y_pose_cmd = DeclareLaunchArgument(
        'y_pose',
        default_value='-3.80',
        description='Initial y position of AMR in world (Area 08 Main Hall)'
    )

    declare_z_pose_cmd = DeclareLaunchArgument(
        'z_pose',
        default_value='0.05',
        description='Initial z position of AMR in world'
    )

    declare_yaw_pose_cmd = DeclareLaunchArgument(
        'yaw_pose',
        default_value='3.14159',
        description='Initial yaw orientation of AMR in world (Facing West towards dining hall)'
    )

    # Combine resource paths so Gazebo Sim resolves model references
    gz_resource_paths = [
        models_path,
        os.path.dirname(pkg_amr_description),
        pkg_amr_description,
        os.path.dirname(pkg_amr_gazebo),
    ]
    gz_resource_path_str = ':'.join(gz_resource_paths)

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

    # Robot State Publisher Node (active when spawn_robot is true)
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time
        }],
        condition=IfCondition(spawn_robot)
    )

    # Launch Gazebo Sim (Harmonic) with restaurant world - GUI mode
    gazebo_sim_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': ['-r ', world]}.items(),
        condition=IfCondition(gui)
    )

    # Launch Gazebo Sim (Harmonic) with restaurant world - Headless mode
    gazebo_sim_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': ['-r -s ', world]}.items(),
        condition=UnlessCondition(gui)
    )

    # Spawn AMR entity in Gazebo Sim (active when spawn_robot is true)
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
        ],
        condition=IfCondition(spawn_robot)
    )

    # ROS-Gz Bridge: Bidirectional topic translation (active when spawn_robot is true)
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        output='screen',
        arguments=[
            # Simulation Clock
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            # 2D LiDAR scan
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            # 9-DOF IMU
            '/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU',
            # Drive commands
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            # Wheel Odometry
            '/wheel/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            # Joint states (Encoder feedback)
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
        ],
        condition=IfCondition(spawn_robot)
    )

    return LaunchDescription([
        append_gz_resource_path,
        append_ign_resource_path,
        append_qsg_render_loop,
        append_qt_platform,
        declare_gui_cmd,
        declare_use_sim_time,
        declare_world_cmd,
        declare_spawn_robot_cmd,
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
