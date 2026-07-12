"""整机 bringup 的第一阶段：仅启动已验证的 X5 直连雷达。"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('enable_lidar', default_value='true'),
        DeclareLaunchArgument('lidar_port', default_value='/dev/ttyS1'),
        DeclareLaunchArgument('lidar_baud_rate', default_value='115200'),
        DeclareLaunchArgument('lidar_frame_id', default_value='laser_link'),
        DeclareLaunchArgument('lidar_scan_topic', default_value='/scan'),
        Node(
            package='ld03_lidar',
            executable='ld03_lidar_node',
            name='ld03_lidar_node',
            output='screen',
            condition=IfCondition(LaunchConfiguration('enable_lidar')),
            parameters=[{
                'port': LaunchConfiguration('lidar_port'),
                'baud_rate': LaunchConfiguration('lidar_baud_rate'),
                'frame_id': LaunchConfiguration('lidar_frame_id'),
                'scan_topic': LaunchConfiguration('lidar_scan_topic'),
            }],
        ),
    ])
