import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    pkg_dir = get_package_share_directory('rover_autonav')

    # Process URDF xacro
    xacro_file = os.path.join(pkg_dir, 'description', 'rover.urdf.xacro')
    robot_description = xacro.process_file(xacro_file).toxml()

    # Robot state publisher (publishes rover body + camera mount TFs)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen',
    )

    # Joint state publisher (for fixed joints visualization)
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        output='screen',
    )

    # ZED camera node
    zed_wrapper_dir = get_package_share_directory('zed_wrapper')
    zed_config = os.path.join(pkg_dir, 'config', 'zed_config.yaml')

    zed_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(zed_wrapper_dir, 'launch', 'zed_camera.launch.py')
        ),
        launch_arguments={
            'camera_model': 'zed2i',
            'camera_name': 'zed',
            'publish_tf': 'true',
            'publish_map_tf': 'true',
            'publish_urdf': 'false',
            'ros_params_override_path': zed_config,
        }.items(),
    )

    return LaunchDescription([
        robot_state_publisher,
        joint_state_publisher,
        zed_camera,
    ])
