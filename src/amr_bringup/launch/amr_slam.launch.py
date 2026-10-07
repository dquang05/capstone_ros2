#!/usr/bin/env python3
"""
One-command bringup for Gazebo Simulation + EKF + SLAM Toolbox + RViz2
Project: Autonomous Mobile Robot (AMR) for Food Delivery

Usage:
    # Scan new main (realistic blueprint) restaurant:
    ros2 launch amr_bringup amr_slam.launch.py

    # Scan test (simple rectangular) restaurant:
    ros2 launch amr_bringup amr_slam.launch.py world_type:=test

    # Headless simulation (RViz2 only):
    ros2 launch amr_bringup amr_slam.launch.py gui:=false
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    pkg_amr_gazebo = get_package_share_directory('amr_gazebo')
    pkg_amr_localization = get_package_share_directory('amr_localization')
    pkg_amr_navigation = get_package_share_directory('amr_navigation')
    pkg_amr_bringup = get_package_share_directory('amr_bringup')

    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    gui = LaunchConfiguration('gui')
    rviz = LaunchConfiguration('rviz')
    world_type = LaunchConfiguration('world_type')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_gui = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Whether to start Gazebo GUI client'
    )

    declare_rviz = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Whether to start RViz2'
    )

    declare_world_type = DeclareLaunchArgument(
        'world_type',
        default_value='main',
        description='Restaurant world to simulate: "main" (realistic blueprint) or "test" (rectangular)'
    )

    # 1a. Gazebo Harmonic Simulation with Main Restaurant (Default)
    gazebo_main_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_amr_gazebo, 'launch', 'restaurant_main.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'gui': gui,
        }.items(),
        condition=IfCondition(PythonExpression(["'", world_type, "' == 'main'"]))
    )

    # 1b. Gazebo Harmonic Simulation with Test Restaurant
    gazebo_test_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_amr_gazebo, 'launch', 'restaurant_test.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'gui': gui,
        }.items(),
        condition=IfCondition(PythonExpression(["'", world_type, "' == 'test'"]))
    )

    # 2. Extended Kalman Filter (EKF) Node
    ekf_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_amr_localization, 'launch', 'ekf.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items()
    )

    # 3. SLAM Toolbox Node (Asynchronous Mapping)
    slam_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_amr_navigation, 'launch', 'slam.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items()
    )

    # 4. RViz2 for SLAM Visualization
    rviz_config_file = os.path.join(pkg_amr_bringup, 'rviz', 'slam.rviz')
    rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(rviz)
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_gui,
        declare_rviz,
        declare_world_type,
        gazebo_main_cmd,
        gazebo_test_cmd,
        ekf_cmd,
        slam_cmd,
        rviz_cmd
    ])
