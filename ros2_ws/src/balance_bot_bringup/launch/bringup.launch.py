"""整机 bringup 骨架 —— 阶段 2 实现。

计划启动项：
  1. chassis_driver（balance_chassis_driver）
  2. robot_state_publisher（balance_bot_description 的 URDF）
  3. 雷达驱动（按实际型号选包，见 docs/references.md 待办）
  4. robot_localization EKF（config/ekf.yaml）
建图模式另起 slam.launch.py（slam_toolbox）；导航模式 nav.launch.py（Nav2 + 已存地图）。
"""
from launch import LaunchDescription


def generate_launch_description():
    return LaunchDescription([
        # TODO(阶段2): 按上述计划补齐节点
    ])
