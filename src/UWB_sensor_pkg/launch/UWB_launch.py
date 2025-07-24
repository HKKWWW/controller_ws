from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os

params_file = os.path.join(
    get_package_share_directory('UWB_sensor_pkg'),  # 改成你实际的包名
    'config',
    'UWB_params.yaml'
)

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='UWB_sensor_pkg',
            executable='UWB_node',
            name='UWB_node',
            output='screen',
            parameters=[params_file]
        )
    ])
