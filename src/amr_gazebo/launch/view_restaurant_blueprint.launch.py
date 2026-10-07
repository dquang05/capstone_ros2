#!/usr/bin/env python3
"""
Alias launch file: forwards to restaurant_main.launch.py with spawn_robot:=false.
Kept for backward compatibility.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    pkg_amr_gazebo = get_package_share_directory('amr_gazebo')
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_amr_gazebo, 'launch', 'restaurant_main.launch.py')
            ),
            launch_arguments={'spawn_robot': 'false'}.items()
        )
    ])
