#!/usr/bin/env python3
import time
import rclpy
from enum import Enum
from threading import Thread
from rclpy.node import Node
from rclpy.task import Future
from geometry_msgs.msg import Twist
from hiwin_interfaces.srv import RobotCommand  # 換回舊版的單一服務

# ============================================================
# 速度與加速度設定
# ============================================================
DEFAULT_VELOCITY = 30
DEFAULT_ACCELERATION = 30
LINE_VELOCITY = 100
LINE_ACCELERATION = 100

# ============================================================
# 吸盤針腳定義
# ============================================================
VACUUM_PIN = 1   # 請依實際針腳號修改

# ============================================================
# 位置定義
# 格式：[X, Y, Z, Rx, Ry, Rz]
# ============================================================

# HOME 位置
HOME_POSE = [210.737, 122.031, -276.256, 1.133, 0.00, -89.007]

# ----------- 下層櫃子位置 -----------
LOWER_FRONT_POSE = [436.396, 602.953, -5.844, -0.032, -1.129, 2.615]
LOWER_INSIDE_POSE = [779.672, 602.949, -5.772, -0.032, -1.131, 2.615]
LOWER_DOWN_POSE = [779.672, 602.949, 36.667, -0.032, -1.131, 2.615]

# ----------- 上層櫃子位置 -----------
UPPER_FRONT_POSE = [436.341, 602.949, -198.703, -0.032, -1.128, 2.614]
UPPER_INSIDE_POSE = [784.174, 602.949, -198.703, -0.032, -1.128, 2.614]
UPPER_DOWN_POSE = [784.173, 602.949, -168.404, -0.032, -1.129, 2.614]


# ============================================================
# 狀態列舉
# ============================================================
class States(Enum):
    INIT = 0
    HOME_MOVE = 1
    
    # 下層櫃子取物階段
    LOWER_FRONT_MOVE = 2      # 移動到下層櫃子前方
    LOWER_INSIDE_MOVE = 3     # 水平進入下層櫃子
    LOWER_DOWN_MOVE = 4       # 下降到吸取點
    PICK_OBJECT = 5           # 啟動吸盤
    LOWER_UP_MOVE = 6         # 上升回原點位
    LOWER_OUT_MOVE = 7        # 水平退出下層櫃子
    
    # 上層櫃子放置階段
    UPPER_FRONT_MOVE = 8      # 移動到上層櫃子前方
    UPPER_INSIDE_MOVE = 9     # 水平進入上層櫃子
    UPPER_DOWN_MOVE = 10      # 下降到放置點
    PLACE_OBJECT = 11         # 關閉吸盤
    UPPER_UP_MOVE = 12        # 上升回原點位
    UPPER_OUT_MOVE = 13       # 水平退出上層櫃子
    
    # 結束階段
    FINAL_HOME_MOVE = 14      # 回到 HOME 位置
    CLOSE_ROBOT = 15          # 關閉機械臂
    FINISH = 16               # 完成


# ============================================================
# 機械臂控制策略類
# ============================================================
class CabinetPickPlaceStrategy(Node):

    def __init__(self):
        super().__init__('cabinet_pick_place_strategy')
        # 換回原本單一的 hiwinmodbus_service
        self.hiwin_client = self.create_client(RobotCommand, 'hiwinmodbus_service')

    # ============================================================
    # 狀態機核心邏輯
    # ============================================================
    def _state_machine(self, state: States) -> States:
        
        # ----------- 初始化 -----------
        if state == States.INIT:
            self.get_logger().info('INIT')
            nest_state = States.HOME_MOVE


        # ----------- 移動到 HOME -----------
        elif state == States.HOME_MOVE:
            self.get_logger().info('HOME_MOVE')
            req = self.generate_robot_request(
                cmd_mode=RobotCommand.Request.PTP,
                cmd_type=RobotCommand.Request.POSE_CMD,
                pose=self.list_to_twist(HOME_POSE),
                holding=True
            )
            res = self.call_hiwin(req)
            self.get_logger().info('動作已完成，準備進入下一階段') # 加這一行
            nest_state = States.LOWER_FRONT_MOVE


        # ============================================================
        # 下層櫃子取物階段
        # ============================================================
        
        # ----------- 移動到下層櫃子前方（PTP）-----------
        elif state == States.LOWER_FRONT_MOVE:
            self.get_logger().info('LOWER_FRONT_MOVE: 移動到下層櫃子前方')
            req = self.generate_robot_request(
                cmd_mode=RobotCommand.Request.PTP,
                cmd_type=RobotCommand.Request.POSE_CMD,
                pose=self.list_to_twist(LOWER_FRONT_POSE),
                holding=True
            )
            res = self.call_hiwin(req)
            self.get_logger().info('動作已完成，準備進入下一階段') # 加這一
            nest_state = States.LOWER_INSIDE_MOVE


        # ----------- 水平進入下層櫃子（LINE）-----------
        elif state == States.LOWER_INSIDE_MOVE:
            self.get_logger().info('LOWER_INSIDE_MOVE: 水平進入下層櫃子')
            req = self.generate_robot_request(
                cmd_mode=RobotCommand.Request.LINE,
                cmd_type=RobotCommand.Request.POSE_CMD,
                pose=self.list_to_twist(LOWER_INSIDE_POSE),
                holding=True,
                velocity=LINE_VELOCITY,
                acceleration=LINE_ACCELERATION
            )
            res = self.call_hiwin(req)
            self.get_logger().info('動作已完成，準備進入下一階段') # 加這一行
            nest_state = States.LOWER_DOWN_MOVE


        # ----------- 下降到吸取點（LINE）-----------
        elif state == States.LOWER_DOWN_MOVE:
            self.get_logger().info('LOWER_DOWN_MOVE: 下降到吸取點')
            req = self.generate_robot_request(
                cmd_mode=RobotCommand.Request.LINE,
                cmd_type=RobotCommand.Request.POSE_CMD,
                pose=self.list_to_twist(LOWER_DOWN_POSE),
                holding=True,
                velocity=LINE_VELOCITY,
                acceleration=LINE_ACCELERATION
            )
            res = self.call_hiwin(req)
            self.get_logger().info('動作已完成，準備進入下一階段') # 加這一行
            nest_state = States.PICK_OBJECT


        # ----------- 啟動吸盤吸取物體 -----------
        elif state == States.PICK_OBJECT:
            self.get_logger().info('PICK_OBJECT: 啟動吸盤')
            self.call_io(VACUUM_PIN, True)
            print("吸盤已啟動，物體已吸取 (等待2秒)")
            time.sleep(2)  # 等待吸取穩固
            nest_state = States.LOWER_UP_MOVE


        # ----------- 上升回原點位（LINE）-----------
        elif state == States.LOWER_UP_MOVE:
            self.get_logger().info('LOWER_UP_MOVE: 上升回原點位')
            req = self.generate_robot_request(
                cmd_mode=RobotCommand.Request.LINE,
                cmd_type=RobotCommand.Request.POSE_CMD,
                pose=self.list_to_twist(LOWER_INSIDE_POSE),
                holding=True,
                velocity=LINE_VELOCITY,
                acceleration=LINE_ACCELERATION
            )
            res = self.call_hiwin(req)
            self.get_logger().info('動作已完成，準備進入下一階段') # 加這一行
            nest_state = States.LOWER_OUT_MOVE


        # ----------- 水平退出下層櫃子（LINE）-----------
        elif state == States.LOWER_OUT_MOVE:
            self.get_logger().info('LOWER_OUT_MOVE: 水平退出下層櫃子')
            req = self.generate_robot_request(
                cmd_mode=RobotCommand.Request.LINE,
                cmd_type=RobotCommand.Request.POSE_CMD,
                pose=self.list_to_twist(LOWER_FRONT_POSE),
                holding=True,
                velocity=LINE_VELOCITY,
                acceleration=LINE_ACCELERATION
            )
            res = self.call_hiwin(req)
            self.get_logger().info('動作已完成，準備進入下一階段') # 加這一行
            nest_state = States.LOWER_FRONT_MOVE
            # 退出後等待 3 秒
            self.get_logger().info('退出櫃子，等待 3 秒...')
            time.sleep(3)
            nest_state = States.UPPER_FRONT_MOVE


        # ============================================================
        # 上層櫃子放置階段
        # ============================================================

        # ----------- 移動到上層櫃子前方（PTP）-----------
        elif state == States.UPPER_FRONT_MOVE:
            self.get_logger().info('UPPER_FRONT_MOVE: 移動到上層櫃子前方')
            req = self.generate_robot_request(
                cmd_mode=RobotCommand.Request.PTP,
                cmd_type=RobotCommand.Request.POSE_CMD,
                pose=self.list_to_twist(UPPER_FRONT_POSE),
                holding=True
            )
            res = self.call_hiwin(req)
            self.get_logger().info('動作已完成，準備進入下一階段') # 加這一行
            nest_state = States.UPPER_INSIDE_MOVE


        # ----------- 水平進入上層櫃子（LINE）-----------
        elif state == States.UPPER_INSIDE_MOVE:
            self.get_logger().info('UPPER_INSIDE_MOVE: 水平進入上層櫃子')
            req = self.generate_robot_request(
                cmd_mode=RobotCommand.Request.LINE,
                cmd_type=RobotCommand.Request.POSE_CMD,
                pose=self.list_to_twist(UPPER_INSIDE_POSE),
                holding=True,
                velocity=LINE_VELOCITY,
                acceleration=LINE_ACCELERATION
            )
            res = self.call_hiwin(req)
            self.get_logger().info('動作已完成，準備進入下一階段') # 加這一行
            nest_state = States.UPPER_DOWN_MOVE


        # ----------- 下降到放置點（LINE）-----------
        elif state == States.UPPER_DOWN_MOVE:
            self.get_logger().info('UPPER_DOWN_MOVE: 下降到放置點')
            req = self.generate_robot_request(
                cmd_mode=RobotCommand.Request.LINE,
                cmd_type=RobotCommand.Request.POSE_CMD,
                pose=self.list_to_twist(UPPER_DOWN_POSE),
                holding=True,
                velocity=LINE_VELOCITY,
                acceleration=LINE_ACCELERATION
            )
            res = self.call_hiwin(req)
            self.get_logger().info('動作已完成，準備進入下一階段') # 加這一行
            nest_state = States.PLACE_OBJECT


        # ----------- 關閉吸盤放置物體 -----------
        elif state == States.PLACE_OBJECT:
            self.get_logger().info('PLACE_OBJECT: 關閉吸盤')
            self.call_io(VACUUM_PIN, False)
            print("吸盤已關閉，物體已放置 (等待2秒)")
            time.sleep(2)  # 等待物體確實放下
            nest_state = States.UPPER_UP_MOVE


        # ----------- 上升回原點位（LINE）-----------
        elif state == States.UPPER_UP_MOVE:
            self.get_logger().info('UPPER_UP_MOVE: 上升回原點位')
            req = self.generate_robot_request(
                cmd_mode=RobotCommand.Request.LINE,
                cmd_type=RobotCommand.Request.POSE_CMD,
                pose=self.list_to_twist(UPPER_INSIDE_POSE),
                holding=True,
                velocity=LINE_VELOCITY,
                acceleration=LINE_ACCELERATION
            )
            res = self.call_hiwin(req)
            self.get_logger().info('動作已完成，準備進入下一階段') # 加這一行
            nest_state = States.UPPER_OUT_MOVE


        # ----------- 水平退出上層櫃子（LINE）-----------
        elif state == States.UPPER_OUT_MOVE:
            self.get_logger().info('UPPER_OUT_MOVE: 水平退出上層櫃子')
            req = self.generate_robot_request(
                cmd_mode=RobotCommand.Request.LINE,
                cmd_type=RobotCommand.Request.POSE_CMD,
                pose=self.list_to_twist(UPPER_FRONT_POSE),
                holding=True,
                velocity=LINE_VELOCITY,
                acceleration=LINE_ACCELERATION
            )
            res = self.call_hiwin(req)
            self.get_logger().info('動作已完成，準備進入下一階段') # 加這一行
            nest_state = States.FINAL_HOME_MOVE


        # ============================================================
        # 結束階段
        # ============================================================

        # ----------- 回到 HOME 位置（PTP）-----------
        elif state == States.FINAL_HOME_MOVE:
            self.get_logger().info('FINAL_HOME_MOVE: 回到 HOME 位置')
            req = self.generate_robot_request(
                cmd_mode=RobotCommand.Request.PTP,
                cmd_type=RobotCommand.Request.POSE_CMD,
                pose=self.list_to_twist(HOME_POSE),
                holding=True
            )
            res = self.call_hiwin(req)
            self.get_logger().info('動作已完成，準備進入下一階段') # 加這一行
            nest_state = States.CLOSE_ROBOT


        # ----------- 關閉機械臂 -----------
        elif state == States.CLOSE_ROBOT:
            self.get_logger().info('CLOSE_ROBOT: 關閉機械臂')
            req = self.generate_robot_request(cmd_mode=RobotCommand.Request.CLOSE)
            res = self.call_hiwin(req)
            self.get_logger().info('動作已完成，準備進入下一階段') # 加這一行
            nest_state = States.FINISH


        # ----------- 未知狀態 -----------
        else:
            print("Error: Invalid state")
            nest_state = None
            self.get_logger().error('Input state not supported!')

        return nest_state


    # ============================================================
    # IO 控制（使用舊版作法）
    # ============================================================
    def call_io(self, pin, on_off):
        request = RobotCommand.Request()
        request.cmd_mode = RobotCommand.Request.DIGITAL_OUTPUT
        request.cmd_type = RobotCommand.Request.POSE_CMD
        request.digital_output_pin = pin
        
        if on_off:
            request.digital_output_cmd = RobotCommand.Request.DIGITAL_ON
        else:
            request.digital_output_cmd = RobotCommand.Request.DIGITAL_OFF
        
        request.holding = False
        
        # 補齊其他欄位
        request.velocity = DEFAULT_VELOCITY
        request.acceleration = DEFAULT_ACCELERATION
        request.tool = 1
        request.base = 11
        request.pose = Twist()
        request.joints = [float('inf')] * 6
        request.circ_s = []
        request.circ_end = []
        request.jog_joint = 6
        request.jog_dir = 0
        request.move_dir = "z"
        request.move_dis = 0.01

        while not self.hiwin_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('service not available, waiting...')
        
        future = self.hiwin_client.call_async(request)
        time.sleep(0.5)
        return None

    # ============================================================
    # 主迴圈
    # ============================================================
    def _main_loop(self):
        state = States.INIT
        while state != States.FINISH:
            state = self._state_machine(state)
            if state == None:
                break
        self.destroy_node()


    # ============================================================
    # 等待 Future 完成
    # ============================================================
    def _wait_for_future_done(self, future: Future, timeout=-1):
        time_start = time.time()
        while not future.done():
            time.sleep(0.01)
            if timeout > 0 and time.time() - time_start > timeout:
                self.get_logger().error('Wait for service timeout!')
                return False
        return True

    # ============================================================
    # 將 list 轉成 Twist 格式 (舊版輔助函式)
    # ============================================================
    def list_to_twist(self, pose_list):
        pose_ = Twist()
        [pose_.linear.x, pose_.linear.y, pose_.linear.z] = pose_list[0:3]
        [pose_.angular.x, pose_.angular.y, pose_.angular.z] = pose_list[3:6]
        return pose_

    # ============================================================
    # 生成 RobotCommand 請求 (舊版作法)
    # ============================================================
    def generate_robot_request(
            self,
            holding=True,
            cmd_mode=RobotCommand.Request.PTP,
            cmd_type=RobotCommand.Request.POSE_CMD,
            velocity=DEFAULT_VELOCITY,
            acceleration=DEFAULT_ACCELERATION,
            tool=1,
            base=11,
            digital_input_pin=0,
            digital_output_pin=0,
            digital_output_cmd=RobotCommand.Request.DIGITAL_OFF,
            pose=Twist(),
            joints=[float('inf')]*6,
            circ_s=[],
            circ_end=[],
            jog_joint=6,
            jog_dir=0,
            move_dir="z",
            move_dis=0.01
            ):
        request = RobotCommand.Request()
        request.digital_input_pin = digital_input_pin
        request.digital_output_pin = digital_output_pin
        request.digital_output_cmd = digital_output_cmd
        request.acceleration = acceleration
        request.jog_joint = jog_joint
        request.velocity = velocity
        request.tool = tool
        request.base = base
        request.cmd_mode = cmd_mode
        request.cmd_type = cmd_type
        request.circ_end = circ_end
        request.jog_dir = jog_dir
        request.holding = holding
        request.joints = joints
        request.circ_s = circ_s
        request.pose = pose
        request.move_dir = move_dir
        request.move_dis = move_dis
        return request

    # ============================================================
    # 發送請求給機械臂
    # ============================================================
    def call_hiwin(self, req):
        while not self.hiwin_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('service not available, waiting again...')
        future = self.hiwin_client.call_async(req)
        if self._wait_for_future_done(future):
            res = future.result()
        else:
            res = None
        return res

    # ============================================================
    # 啟動主迴圈執行緒
    # ============================================================
    def start_main_loop_thread(self):
        self.main_loop_thread = Thread(target=self._main_loop)
        self.main_loop_thread.daemon = True
        self.main_loop_thread.start()


# ============================================================
# 主函數
# ============================================================
def main(args=None):
    rclpy.init(args=args)
    strategy = CabinetPickPlaceStrategy()
    strategy.start_main_loop_thread()
    rclpy.spin(strategy)
    rclpy.shutdown()


if __name__ == "__main__":
    main()