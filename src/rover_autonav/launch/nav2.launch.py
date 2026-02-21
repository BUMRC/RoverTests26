import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    pkg_dir = get_package_share_directory('rover_autonav')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    nav2_params = os.path.join(pkg_dir, 'config', 'nav2_params.yaml')

    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'params_file': nav2_params,
            'use_sim_time': 'false',
            'autostart': 'true',
        }.items(),
    )

    return LaunchDescription([
        nav2_bringup,
    ])
