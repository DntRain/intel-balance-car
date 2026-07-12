from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pose_names = [
        'laser_x', 'laser_y', 'laser_z', 'laser_roll', 'laser_pitch', 'laser_yaw',
        'camera_x', 'camera_y', 'camera_z', 'camera_roll', 'camera_pitch', 'camera_yaw',
    ]
    xacro_path = PathJoinSubstitution([
        FindPackageShare('balance_bot_description'),
        'urdf',
        'balance_bot.urdf.xacro',
    ])
    robot_description = Command([
        'xacro ', xacro_path,
        *sum(([f' {name}:=', LaunchConfiguration(name)] for name in pose_names), []),
    ])

    return LaunchDescription([
        DeclareLaunchArgument('laser_x', default_value='0'),
        DeclareLaunchArgument('laser_y', default_value='0'),
        DeclareLaunchArgument('laser_z', default_value='0.20'),
        DeclareLaunchArgument('laser_roll', default_value='0'),
        DeclareLaunchArgument('laser_pitch', default_value='0'),
        DeclareLaunchArgument('laser_yaw', default_value='0'),
        DeclareLaunchArgument('camera_x', default_value='0'),
        DeclareLaunchArgument('camera_y', default_value='0'),
        DeclareLaunchArgument('camera_z', default_value='0.20'),
        DeclareLaunchArgument('camera_roll', default_value='0'),
        DeclareLaunchArgument('camera_pitch', default_value='0'),
        DeclareLaunchArgument('camera_yaw', default_value='0'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': ParameterValue(robot_description, value_type=str),
            }],
            output='screen',
        ),
    ])
