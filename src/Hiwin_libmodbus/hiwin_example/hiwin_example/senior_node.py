#!/usr/bin/env python3
"""
sensor_node.py

讀取 Arduino Serial 數據（左右兩顆 VL53L1X）
發布到 ROS2 Topic: /disc_sensor
訊息格式: Float32MultiArray [左距離, 右距離]
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import serial
import threading


# ============================================================
# 參數設定
# ============================================================
SERIAL_PORT = '/dev/ttyACM0'   # Arduino 的 Serial Port
BAUD_RATE   = 115200            # 鮑率（跟 Arduino 一致）
PUBLISH_HZ  = 20                # 發布頻率（Hz）


# ============================================================
# SensorNode
# ============================================================
class SensorNode(Node):

    def __init__(self):
        super().__init__('sensor_node')

        # ── 建立 Publisher ──
        self.publisher = self.create_publisher(
            Float32MultiArray,
            '/disc_sensor',
            10
        )

        # ── 初始化 Serial ──
        try:
            self.serial = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1.0)
            self.get_logger().info(f'Serial 連接成功：{SERIAL_PORT}')
        except serial.SerialException as e:
            self.get_logger().error(f'Serial 連接失敗：{e}')
            raise

        # ── 最新感測器數值 ──
        self.left_dist  = 0.0
        self.right_dist = 0.0

        # ── 啟動 Serial 讀取執行緒 ──
        self.serial_thread = threading.Thread(
            target=self._serial_read_loop,
            daemon=True
        )
        self.serial_thread.start()

        # ── 建立定時發布 Timer ──
        self.timer = self.create_timer(
            1.0 / PUBLISH_HZ,
            self._publish_sensor
        )

        self.get_logger().info('Sensor Node 啟動完成')


    # ============================================================
    # Serial 讀取執行緒
    # ============================================================
    def _serial_read_loop(self):
        """持續讀取 Arduino Serial，解析 CSV 格式"""

        # 清空緩衝區
        self.serial.reset_input_buffer()

        while True:
            try:
                line = self.serial.readline().decode('utf-8').strip()

                if not line or ',' not in line:
                    continue

                parts = line.split(',')

                if len(parts) != 2:
                    continue

                left  = float(parts[0])
                right = float(parts[1])

                # 過濾異常值（例如 -1 或 9999）
                if left  <= 0 or left  >= 9000:
                    continue
                if right <= 0 or right >= 9000:
                    continue

                self.left_dist  = left
                self.right_dist = right

            except (ValueError, UnicodeDecodeError):
                # 解析失敗，跳過這筆
                continue
            except Exception as e:
                self.get_logger().error(f'Serial 讀取錯誤：{e}')
                break


    # ============================================================
    # 定時發布 Topic
    # ============================================================
    def _publish_sensor(self):
        """定時發布感測器數值到 /disc_sensor"""

        msg = Float32MultiArray()
        msg.data = [self.left_dist, self.right_dist]
        self.publisher.publish(msg)


    # ============================================================
    # 關閉時清理
    # ============================================================
    def destroy_node(self):
        if self.serial.is_open:
            self.serial.close()
            self.get_logger().info('Serial 已關閉')
        super().destroy_node()


# ============================================================
# 主函數
# ============================================================
def main(args=None):
    rclpy.init(args=args)
    node = None

    try:
        node = SensorNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'錯誤：{e}')
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()