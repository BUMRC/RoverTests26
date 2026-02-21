import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_dir = get_package_share_directory('rover_autonav')
    launch_dir = os.path.join(pkg_dir, 'launch')

    enable_motors_arg = DeclareLaunchArgument(
        'enable_motors',
        default_value='true',
        description='Set to false to disable motor driver (for testing without hardware)',
    )

    # Start ZED camera + URDF + depth-to-laserscan
    zed_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'zed.launch.py')
        ),
    )

    # Start motor driver (conditional)
    motors_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'motors.launch.py')
        ),
        condition=IfCondition(LaunchConfiguration('enable_motors')),
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
        enable_motors_arg,
        zed_launch,
        motors_launch,
        nav2_launch,
        foxglove_launch,
    ])
