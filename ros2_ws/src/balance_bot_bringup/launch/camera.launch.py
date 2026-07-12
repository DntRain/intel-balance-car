"""启动 IMX219 MIPI 相机、JPEG 编码和网页预览，并统一图像坐标系。"""
from pathlib import Path

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config_path = Path(get_package_prefix('mipi_cam')) / 'lib' / 'mipi_cam' / 'config'

    camera = Node(
        package='mipi_cam',
        executable='mipi_cam',
        name='mipi_cam',
        output='screen',
        parameters=[{
            'config_path': LaunchConfiguration('config_path'),
            'camera_calibration_file_path': LaunchConfiguration('calibration_file'),
            'out_format': 'nv12',
            'framerate': LaunchConfiguration('framerate'),
            'image_width': LaunchConfiguration('image_width'),
            'image_height': LaunchConfiguration('image_height'),
            'io_method': 'ros',
            'channel': LaunchConfiguration('channel'),
            'video_device': 'default',
            'frame_ts_type': 'sensor',
            'rotation': LaunchConfiguration('rotation'),
            'cal_rotation': LaunchConfiguration('rotation'),
            'gdc_bin_file': 'default',
            'link_type': 0,
            'link_port': 0,
            'cal_alpha': 0.0,
        }, {'frame_id': 'camera_link'}],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
    )

    jpeg_codec = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(Path(get_package_share_directory('hobot_codec')) / 'launch' / 'hobot_codec_encode.launch.py')),
        launch_arguments={
            'codec_in_mode': 'ros',
            'codec_out_mode': 'ros',
            'codec_jpg_quality': '85.0',
            'codec_sub_topic': '/image_raw',
            'codec_pub_topic': '/image_jpeg',
        }.items(),
    )
    websocket = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(Path(get_package_share_directory('websocket')) / 'launch' / 'websocket.launch.py')),
        launch_arguments={
            'websocket_image_topic': '/image_jpeg',
            'websocket_only_show_image': 'True',
        }.items(),
    )
    shared_memory = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(Path(get_package_share_directory('hobot_shm')) / 'launch' / 'hobot_shm.launch.py')),
    )

    return LaunchDescription([
        # mipi_cam 内部直接拼接配置文件名，目录末尾的 / 不能省略。
        DeclareLaunchArgument('config_path', default_value=str(config_path) + '/'),
        DeclareLaunchArgument('calibration_file', default_value='default'),
        DeclareLaunchArgument('image_width', default_value='960'),
        DeclareLaunchArgument('image_height', default_value='544'),
        DeclareLaunchArgument('framerate', default_value='30.0'),
        DeclareLaunchArgument('channel', default_value='0'),
        DeclareLaunchArgument('rotation', default_value='0.0'),
        DeclareLaunchArgument('log_level', default_value='warn'),
        shared_memory,
        camera,
        jpeg_codec,
        websocket,
    ])
