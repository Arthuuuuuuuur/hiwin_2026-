#!/usr/bin/env python3
"""
fake_sensor.py

發布假的感測器訊號到 /disc_sensor
用於測試判斷邏輯，不需要實際感測器

模擬狀態：
  NORMAL      → 左=40, 右=50   (std ≈ 5)
  TILT        → 左=40, 右=65   (std ≈ 12.5)
  OFFSET_LEFT → 左=30, 右=80   (std ≈ 25)
  OFFSET_RIGHT→ 左=80, 右=30   (std ≈ 25)
  OVERLAP     → 左=15, 右=17   (兩個都 < 20)
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import time


# ============================================================
# 假訊號設定
# ============================================================

# 修改這裡來切換模擬狀態
SIMULATE_STATE = "TILT"   # 改成你想測試的狀態

FAKE_VALUES = {
    "NORMAL"       : [40.0, 50.0],   # std ≈ 5.0   → 正常
    "TILT"         : [40.0, 65.0],   # std ≈ 12.5  → 傾斜
    "OFFSET_LEFT"  : [30.0, 80.0],   # std ≈ 25.0  → 偏左
    "OFFSET_RIGHT" : [80.0, 30.0],   # std ≈ 25.0  → 偏右
    "OVERLAP"      : [15.0, 17.0],   # 兩個都 < 20 → 重疊
}


# ============================================================
# 假感測器節點
# ============================================================
class FakeSensorNode(Node):

    def __init__(self):
        super().__init__('fake_sensor_node')

        self.publisher = self.create_publisher(
            Float32MultiArray,
            '/disc_sensor',
            10
        )

        self.timer = self.create_timer(0.05, self._publish)  # 20Hz

        left, right = FAKE_VALUES[SIMULATE_STATE]
        self.get_logger().info(f'假感測器啟動')
        self.get_logger().info(f'模擬狀態：{SIMULATE_STATE}')
        self.get_logger().info(f'左={left}mm  右={right}mm')

        import numpy as np
        std = float(np.std([left, right]))
        self.get_logger().info(f'標準差：{std:.2f}')


    def _publish(self):
        left, right = FAKE_VALUES[SIMULATE_STATE]
        msg = Float32MultiArray()
        msg.data = [left, right]
        self.publisher.publish(msg)


# ============================================================
# 主函數
# ============================================================
def main(args=None):
    rclpy.init(args=args)
    node = FakeSensorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()