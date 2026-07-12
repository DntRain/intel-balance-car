"""启动 AMCL 与 Nav2；必须指定一个由 slam_toolbox 保存的地图 YAML。"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    package_share = Path(get_package_share_directory('balance_bot_bringup'))
    nav2_launch = Path(get_package_share_directory('nav2_bringup')) / 'launch' / 'bringup_launch.py'
    return LaunchDescription([
        DeclareLaunchArgument('map', description='由 slam_toolbox 保存的地图 YAML 文件'),
        DeclareLaunchArgument('params_file', default_value=str(package_share / 'config' / 'nav2_params.yaml')),
        DeclareLaunchArgument('autostart', default_value='true'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(nav2_launch)),
            launch_arguments={
                'map': LaunchConfiguration('map'),
                'params_file': LaunchConfiguration('params_file'),
                'autostart': LaunchConfiguration('autostart'),
                'use_sim_time': 'false',
            }.items(),
        ),
    ])
