"""一次启动机器人静态 TF、LD03 雷达和 IMX219 相机网页预览。"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def include(package_name, launch_name):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(Path(get_package_share_directory(package_name)) / 'launch' / launch_name)),
    )


def generate_launch_description():
    package_share = Path(get_package_share_directory('balance_bot_bringup'))
    ld08_pwm = ExecuteProcess(
        cmd=['python3', str(package_share / 'scripts' / 'ld08_pwm.py')],
        output='screen',
    )
    return LaunchDescription([
        ld08_pwm,
        include('balance_bot_description', 'description.launch.py'),
        include('ld03_lidar', 'ld03_lidar.launch.py'),
        include('balance_bot_bringup', 'camera.launch.py'),
    ])
