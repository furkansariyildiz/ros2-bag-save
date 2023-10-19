import os
from launch import LaunchDescription
from launch_ros.actions import SetParameter
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros2_bag_save',
            executable='ros2_bag_save.py',
            name='ros2_bag_recorder_node',
            output='screen',
            parameters=[os.path.join(
                get_package_share_directory('ros2_bag_save'),
                'config', 'config.yaml')]
        )
    ])