"""启动 slam_toolbox 建图；底盘节点必须先提供 odom 到 base_link 的 TF。"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_params = Path(get_package_share_directory('balance_bot_bringup')) / 'config' / 'slam_toolbox.yaml'
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=str(default_params)),
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[LaunchConfiguration('params_file')],
        ),
    ])
