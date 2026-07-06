#!/usr/bin/env python3
import time
import rclpy
from enum import Enum
from threading import Thread
from rclpy.node import Node
from rclpy.task import Future
from geometry_msgs.msg import Twist
from hiwin_interfaces.srv import RobotCommand

# ============================================================
# 速度與加速度設定
# ============================================================
DEFAULT_VELOCITY = 10
DEFAULT_ACCELERATION = 10

# ============================================================
# 吸盤針腳
# ============================================================
VACUUM_PIN = 1

# ============================================================
# 點位定義（佔位符，請填入實際座標）
# 格式：[X, Y, Z, Rx, Ry, Rz]
# ============================================================
HOME_POSE = [210.737, 122.031, -276.256, 1.133, 0.00, -89.007]
PICK_POSE = [397.961, 44.794, 158.653, 1.133, 0.001, -89.003]   # 吸取點
PLACE_POSE = [58.226, 44.794, 144.983, 1.134, 0.001, -89.008]  # 放置點


# ============================================================
# 狀態列舉 (新增 POST_PICK_HOME_MOVE 與 WAIT_AT_HOME)
# ============================================================
class States(Enum):
    INIT = 0
    HOME_MOVE = 1               # 移動到 HOME
    PICK_MOVE = 2               # 移動到吸取點
    PICK_IO_ON = 3              # 開啟吸盤
    POST_PICK_HOME_MOVE = 4     # [新增] 吸完回到 HOME
    WAIT_AT_HOME = 5            # [新增] 維持 3 秒
    PLACE_MOVE = 6              # 移動到放置點
    PLACE_IO_OFF = 7            # 關閉吸盤
    FINAL_HOME_MOVE = 8         # 回到 HOME
    CLOSE_ROBOT = 9             # 關閉機械臂
    FINISH = 10                 # 完成


# ============================================================
# 簡單測試節點
# ============================================================
class SimpleTestStrategy(Node):

    def __init__(self):
        super().__init__('simple_test_strategy')
        self.hiwin_client = self.create_client(RobotCommand, 'hiwinmodbus_service')


    # ============================================================
    # 狀態機
    # ============================================================
    def _state_machine(self, state: States) -> States:

        # ----------- 初始化 -----------
        if state == States.INIT:
            self.get_logger().info('INIT')
            nest_state = States.HOME_MOVE


        # ----------- 移動到 HOME -----------
        elif state == States.HOME_MOVE:
            self.get_logger().info('HOME_MOVE: 移動到 HOME 位置')
            req = self.generate_robot_request(
                cmd_mode=RobotCommand.Request.PTP,
                cmd_type=RobotCommand.Request.POSE_CMD,
                pose=self.list_to_twist(HOME_POSE),
                holding=True
            )
            res = self.call_hiwin(req)
            nest_state = States.PICK_MOVE


        # ----------- 移動到吸取點 -----------
        elif state == States.PICK_MOVE:
            self.get_logger().info('PICK_MOVE: 移動到吸取點')
            req = self.generate_robot_request(
                cmd_mode=RobotCommand.Request.PTP,
                cmd_type=RobotCommand.Request.POSE_CMD,
                pose=self.list_to_twist(PICK_POSE),
                holding=True
            )
            res = self.call_hiwin(req)
            nest_state = States.PICK_IO_ON


        # ----------- 開啟吸盤 -----------
        elif state == States.PICK_IO_ON:
            self.get_logger().info('PICK_IO_ON: 開啟吸盤')
            self.call_io(VACUUM_PIN, True)
            print("吸盤已開啟")
            # 修改：吸完後進入「回到 HOME」的狀態
            nest_state = States.POST_PICK_HOME_MOVE


        # ----------- [新增] 吸完回到 HOME -----------
        elif state == States.POST_PICK_HOME_MOVE:
            self.get_logger().info('POST_PICK_HOME_MOVE: 吸完回到 HOME 位置')
            req = self.generate_robot_request(
                cmd_mode=RobotCommand.Request.PTP,
                cmd_type=RobotCommand.Request.POSE_CMD,
                pose=self.list_to_twist(HOME_POSE),
                holding=True
            )
            res = self.call_hiwin(req)
            # 抵達 HOME 後進入等待狀態
            nest_state = States.WAIT_AT_HOME


        # ----------- [新增] 等待 3 秒 -----------
        elif state == States.WAIT_AT_HOME:
            self.get_logger().info('WAIT_AT_HOME: 停留等待 3 秒')
            time.sleep(3.0)  # 暫停 3 秒
            # 等待完畢後，前往放置點
            nest_state = States.PLACE_MOVE


        # ----------- 移動到放置點 -----------
        elif state == States.PLACE_MOVE:
            self.get_logger().info('PLACE_MOVE: 移動到放置點')
            req = self.generate_robot_request(
                cmd_mode=RobotCommand.Request.PTP,
                cmd_type=RobotCommand.Request.POSE_CMD,
                pose=self.list_to_twist(PLACE_POSE),
                holding=True
            )
            res = self.call_hiwin(req)
            nest_state = States.PLACE_IO_OFF


        # ----------- 關閉吸盤 -----------
        elif state == States.PLACE_IO_OFF:
            self.get_logger().info('PLACE_IO_OFF: 關閉吸盤')
            self.call_io(VACUUM_PIN, False)
            print("吸盤已關閉")
            nest_state = States.FINAL_HOME_MOVE


        # ----------- 回到 HOME -----------
        elif state == States.FINAL_HOME_MOVE:
            self.get_logger().info('FINAL_HOME_MOVE: 回到 HOME 位置')
            req = self.generate_robot_request(
                cmd_mode=RobotCommand.Request.PTP,
                cmd_type=RobotCommand.Request.POSE_CMD,
                pose=self.list_to_twist(HOME_POSE),
                holding=True
            )
            res = self.call_hiwin(req)
            nest_state = States.CLOSE_ROBOT


        # ----------- 關閉機械臂 -----------
        elif state == States.CLOSE_ROBOT:
            self.get_logger().info('CLOSE_ROBOT: 關閉機械臂')
            req = self.generate_robot_request(cmd_mode=RobotCommand.Request.CLOSE)
            res = self.call_hiwin(req)
            nest_state = States.FINISH


        # ----------- 未知狀態 -----------
        else:
            print("Error: Invalid state")
            nest_state = None
            self.get_logger().error('Input state not supported!')

        return nest_state


    # ============================================================
    # IO 控制（不等待回應，因為控制器不會回應 IO 指令）
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
        
        # 異步發送，不等待回應
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
    # 將 list 轉成 Twist 格式
    # ============================================================
    def list_to_twist(self, pose_list):
        pose_ = Twist()
        [pose_.linear.x, pose_.linear.y, pose_.linear.z] = pose_list[0:3]
        [pose_.angular.x, pose_.angular.y, pose_.angular.z] = pose_list[3:6]
        return pose_


    # ============================================================
    # 生成 RobotCommand 請求
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
    strategy = SimpleTestStrategy()
    strategy.start_main_loop_thread()
    rclpy.spin(strategy)
    rclpy.shutdown()


if __name__ == "__main__":
    main()