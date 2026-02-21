import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    pkg_dir = get_package_share_directory('rover_autonav')
    launch_dir = os.path.join(pkg_dir, 'launch')

    # Start ZED camera + URDF + depth-to-laserscan
    zed_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'zed.launch.py')
        ),
    )

    # Start Nav2 after a short delay to let ZED TFs stabilize
    nav2_launch = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(launch_dir, 'nav2.launch.py')
                ),
            ),
        ],
    )

    # Start Foxglove bridge
    foxglove_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'foxglove.launch.py')
        ),
    )

    return LaunchDescription([
        zed_launch,
        nav2_launch,
        foxglove_launch,
    ])
