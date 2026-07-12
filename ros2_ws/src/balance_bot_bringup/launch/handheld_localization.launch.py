"""手持 AMCL 定位：RF2O 提供里程计，Nav2 localization 加载已有地图。"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(get_package_share_directory('balance_bot_bringup'))
    sensors = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(package_share / 'launch' / 'sensors.launch.py')),
    )
    laser_odometry = Node(
        package='rf2o_laser_odometry',
        executable='rf2o_laser_odometry_node',
        name='rf2o_laser_odometry',
        output='screen',
        parameters=[{
            'laser_scan_topic': '/scan',
            'odom_topic': '/odom_rf2o',
            'publish_tf': True,
            'base_frame_id': 'base_link',
            'odom_frame_id': 'odom',
            'init_pose_from_topic': '',
            'freq': 20.0,
        }],
    )
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(Path(get_package_share_directory('nav2_bringup')) / 'launch' / 'localization_launch.py')),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'params_file': LaunchConfiguration('params_file'),
            'autostart': 'true',
            'use_sim_time': 'false',
        }.items(),
    )
    return LaunchDescription([
        DeclareLaunchArgument('map', description='要加载的 ROS 地图 YAML 绝对路径'),
        DeclareLaunchArgument(
            'params_file',
            default_value=str(package_share / 'config' / 'nav2_params.yaml'),
        ),
        sensors,
        laser_odometry,
        localization,
    ])
