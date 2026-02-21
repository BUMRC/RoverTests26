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
            'publish_urdf': 'false',
            'ros_params_override_path': zed_config,
        }.items(),
    )

    # Depth image to laser scan
    depthimage_to_laserscan = Node(
        package='depthimage_to_laserscan',
        executable='depthimage_to_laserscan_node',
        name='depthimage_to_laserscan',
        remappings=[
            ('depth', '/zed/zed_node/depth/depth_registered'),
            ('depth_camera_info', '/zed/zed_node/depth/camera_info'),
        ],
        parameters=[{
            'scan_height': 10,
            'scan_time': 0.033,
            'range_min': 0.3,
            'range_max': 20.0,
            'output_frame': 'base_link',
        }],
        output='screen',
    )

    return LaunchDescription([
        robot_state_publisher,
        joint_state_publisher,
        zed_camera,
        depthimage_to_laserscan,
    ])
