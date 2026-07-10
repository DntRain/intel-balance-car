"""底盘串口驱动节点骨架 —— 阶段 2 实现。

协议定义见 docs/serial-protocol.md（唯一权威，常量改动需同步）。
订阅 /cmd_vel → 组下行帧写串口；解析上行遥测帧 → 发布 /odom_raw、/imu/data_raw。
"""
import rclpy
from rclpy.node import Node

# ---- 里程计常量（与 docs/serial-protocol.md 保持一致）----
WHEEL_DIAMETER_MM = 67.0     # ⚠️ 待实测
WHEEL_TRACK_MM = 170.0       # ⚠️ 待实测
COUNTS_PER_WHEEL_REV = 1320  # 11 线 × 4 倍频 × 减速比 30

FRAME_HEADER = b'\xaa\x55'
TYPE_TELEMETRY = 0x01   # 上行 28 字节载荷
TYPE_CMD_VEL = 0x10     # 下行 4 字节载荷


class ChassisDriver(Node):
    def __init__(self):
        super().__init__('chassis_driver')
        self.declare_parameter('port', '/dev/ttyS1')  # 按 RDK X5 实际 UART 调整
        self.declare_parameter('baud', 115200)
        # TODO(阶段2): 打开串口；订阅 /cmd_vel；创建 /odom_raw、/imu/data_raw 发布者
        # TODO(阶段2): 读线程逐字节找帧头 → 校验 → 解析 → 差速运动学积分出 odom
        # TODO(阶段2): 是否发布 odom→base_link TF 由参数控制（EKF 启用时应关闭）


def main():
    rclpy.init()
    node = ChassisDriver()
    rclpy.spin(node)
    rclpy.shutdown()
