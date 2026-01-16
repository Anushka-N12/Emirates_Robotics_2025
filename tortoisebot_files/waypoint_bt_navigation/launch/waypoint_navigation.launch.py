from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='waypoint_bt_navigation',
            executable='behavior_tree_main',
            name='bt_behavior_tree'
        )
    ])
