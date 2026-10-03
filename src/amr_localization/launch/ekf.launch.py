import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_amr_localization = get_package_share_directory('amr_localization')

    use_sim_time = LaunchConfiguration('use_sim_time')
    ekf_config_file = LaunchConfiguration('ekf_config_file')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_ekf_config_file = DeclareLaunchArgument(
        'ekf_config_file',
        default_value=os.path.join(pkg_amr_localization, 'config', 'ekf.yaml'),
        description='Full path to the EKF yaml config file'
    )

    # Robot Localization EKF node
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            ekf_config_file,
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('odometry/filtered', '/odometry/filtered')
        ]
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_ekf_config_file,
        ekf_node
    ])
