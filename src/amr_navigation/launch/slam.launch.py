import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_amr_navigation = get_package_share_directory('amr_navigation')
    pkg_slam_toolbox = get_package_share_directory('slam_toolbox')

    use_sim_time = LaunchConfiguration('use_sim_time')
    slam_params_file = LaunchConfiguration('slam_params_file')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_slam_params_file = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(pkg_amr_navigation, 'config', 'slam_toolbox.yaml'),
        description='Full path to the SLAM Toolbox yaml config file'
    )

    # Include slam_toolbox online_async_launch with lifecycle transition handling
    slam_toolbox_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_slam_toolbox, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam_params_file': slam_params_file,
            'autostart': 'true',
            'use_lifecycle_manager': 'false'
        }.items()
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_slam_params_file,
        slam_toolbox_cmd
    ])
