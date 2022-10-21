import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('wit-imu-driver'),
        'config',
        'wt901c.yaml'
        )
    return LaunchDescription([
        Node(
            package = 'wit-imu-driver',
            executable = 'wit-imu-driver_node',
            output = 'screen',
            parameters = [config]
        )
    ])