from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    motor_driver = Node(
        package='rover_autonav',
        executable='motor_driver.py',
        name='motor_driver',
        output='screen',
    )

    return LaunchDescription([
        motor_driver,
    ])
