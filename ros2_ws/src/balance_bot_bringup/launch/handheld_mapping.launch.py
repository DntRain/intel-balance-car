"""手持 X5+LD03 建图：激光里程计提供 TF，再由 slam_toolbox 建图。"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
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
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(package_share / 'launch' / 'slam.launch.py')),
    )
    return LaunchDescription([sensors, laser_odometry, slam])
