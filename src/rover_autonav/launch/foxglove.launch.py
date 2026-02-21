from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    topic_whitelist = [
        '/zed/zed_node/rgb/image_rect_color/compressed',
        '/zed/zed_node/depth/depth_registered',
        '/zed/zed_node/depth/camera_info',
        '/zed/zed_node/odom',
        '/zed/zed_node/path_odom',
        '/zed/zed_node/pose',
        '/scan',
        '/local_costmap/costmap',
        '/local_costmap/published_footprint',
        '/global_costmap/costmap',
        '/global_costmap/published_footprint',
        '/plan',
        '/odom',
        '/tf',
        '/tf_static',
        '/goal_pose',
        '/robot_description',
        '/initialpose',
    ]

    foxglove_bridge = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        parameters=[{
            'port': 8765,
            'address': '0.0.0.0',
            'num_threads': 2,
            'topic_whitelist': topic_whitelist,
            'send_buffer_limit': 50000000,
            'max_qos_depth': 5,
        }],
        output='screen',
    )

    return LaunchDescription([
        foxglove_bridge,
    ])
