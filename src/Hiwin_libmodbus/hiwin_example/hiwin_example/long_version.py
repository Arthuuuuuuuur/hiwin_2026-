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
DEFAULT_VELOCITY = 60
DEFAULT_ACCELERATION = 60
LINE_VELOCITY = 100
LINE_ACCELERATION = 100

# ============================================================
# 吸盤針腳定義
# ============================================================
VACUUM_PIN = 1

# ============================================================
# 位置定義（佔位符，請填入實際座標）
# 格式：[X, Y, Z, Rx, Ry, Rz]
# ============================================================
HOME_POSE        = [-0.03, 367.996, 293.741, -180.000, 0.005, 90.002]  # HOME 位置
LOWER_FRONT_POSE = [-326.624, 467.526, 22.552, -179.992, 0.000, 177.403]  # 下層櫃子前方
LOWER_ABOVE_POSE = [-326.624, 467.526, 22.552, -179.992, 0.000, 177.403]  # 物體正上方
LOWER_DOWN_POSE  = [-326.624, 467.526, 7.275, -179.992, 0.000, 177.403]  # 吸取點
UPPER_FRONT_POSE = [-326.624, 467.525, 244.784, 179.998, 0.007, 69.822]  # 上層櫃子前方
UPPER_ABOVE_POSE = [-326.624, 467.525, 244.784, -179.998, 0.000, 178.165]  # 放置區正上方
UPPER_DOWN_POSE  = [-326.624, 467.525, 217.784, -179.998, 0.000, 178.165]  # 放置點

# ============================================================
# A6 旋轉角度定義（佔位符，請填入實際角度）
# 格式：[A1, A2, A3, A4, A5, A6]
# ============================================================
JOINTS_READY       = [34.938, -42.683, 11.480, 0.005, -58.802, 55.114]  # 準備吸取位置（第2、9、15步共用）
JOINTS_ABOVE_PICK  = [34.938, -42.683, 11.480, 0.005, -58.802, -52.468]  # 物體正上方（第4步）
JOINTS_ABOVE_PLACE = [34.938, -37.156, 40.612, 0.004, -93.462, -53.227]  # 放置區上方（第10步）


# ============================================================
# 狀態列舉
# ============================================================
class States(Enum):
    INIT = 0
    HOME_MOVE = 1               # 移動到 HOME

    # 下層取物階段
    ROTATE_READY      = 2       # 末端旋轉到準備吸取位置（HOME 後先旋轉）
    LOWER_FRONT_MOVE  = 3       # 移動到下層櫃子前方
    ROTATE_ABOVE_PICK = 4       # 末端旋轉到物體正上方
    LOWER_DOWN_MOVE   = 5       # 下降到吸取點
    PICK_OBJECT       = 6       # 啟動吸盤
    LOWER_UP_MOVE     = 7       # 上升回物體正上方
    ROTATE_READY_BACK = 8       # 末端旋轉回準備吸取位置

    # 上層放置階段
    UPPER_FRONT_MOVE   = 9      # 移動到上層櫃子前方
    ROTATE_ABOVE_PLACE = 10     # 末端旋轉到放置區上方
    UPPER_DOWN_MOVE    = 11     # 下降到放置點
    PLACE_OBJECT       = 12     # 關閉吸盤（放置）
    UPPER_UP_MOVE      = 13     # 上升回放置區上方
    ROTATE_READY_FINAL = 14     # 末端旋轉回準備吸取位置

    # 結束階段
    FINAL_HOME_MOVE = 15        # 回到 HOME
    CLOSE_ROBOT     = 16        # 關閉機械臂
    FINISH          = 17        # 完成


# ============================================================
# 機械臂控制策略類
# ============================================================
class CabinetPickPlaceStrategy(Node):

    def __init__(self):
        super().__init__('cabinet_pick_place_strategy')
        self.hiwin_client = self.create_client(RobotCommand, 'hiwinmodbus_service')


    # ============================================================
    # 狀態機核心邏輯
    # ============================================================
    def _state_machine(self, state: States) -> States:

        # ----------- 初始化 -----------
        if state == States.INIT:
            self.get_logger().info('INIT')
            nest_state = States.HOME_MOVE


        # ----------- 1. 移動到 HOME -----------
        elif state == States.HOME_MOVE:
            self.get_logger().info('HOME_MOVE: 移動到 HOME 位置')
            req = self.generate_robot_request(
                cmd_mode=RobotCommand.Request.PTP,
                cmd_type=RobotCommand.Request.POSE_CMD,
                pose=self.list_to_twist(HOME_POSE),
                holding=True
            )
            res = self.call_hiwin(req)
            self.get_logger().info('完成')
            nest_state = States.ROTATE_READY  # ← HOME 後先旋轉


        # ============================================================
        # 下層取物階段
        # ============================================================

        # ----------- 2. 末端旋轉到準備吸取位置 -----------
        elif state == States.ROTATE_READY:
            self.get_logger().info('ROTATE_READY: 末端旋轉到準備吸取位置')
            req = self.generate_robot_request(
                cmd_mode=RobotCommand.Request.PTP,
                cmd_type=RobotCommand.Request.JOINTS_CMD,
                joints=JOINTS_READY,
                holding=True
            )
            res = self.call_hiwin(req)
            self.get_logger().info('完成')
            nest_state = States.LOWER_FRONT_MOVE  # ← 旋轉完再移動


        # ----------- 3. 移動到下層櫃子前方（PTP）-----------
        elif state == States.LOWER_FRONT_MOVE:
            self.get_logger().info('LOWER_FRONT_MOVE: 移動到下層櫃子前方')
            req = self.generate_robot_request(
                cmd_mode=RobotCommand.Request.PTP,
                cmd_type=RobotCommand.Request.POSE_CMD,
                pose=self.list_to_twist(LOWER_FRONT_POSE),
                holding=True
            )
            res = self.call_hiwin(req)
            self.get_logger().info('完成')
            nest_state = States.ROTATE_ABOVE_PICK


        # ----------- 4. 末端旋轉到物體正上方 -----------
        elif state == States.ROTATE_ABOVE_PICK:
            self.get_logger().info('ROTATE_ABOVE_PICK: 末端旋轉到物體正上方')
            req = self.generate_robot_request(
                cmd_mode=RobotCommand.Request.PTP,
                cmd_type=RobotCommand.Request.JOINTS_CMD,
                joints=JOINTS_ABOVE_PICK,
                holding=True
            )
            res = self.call_hiwin(req)
            self.get_logger().info('完成')
            nest_state = States.LOWER_DOWN_MOVE


        # ----------- 5. 下降到吸取點（LINE）-----------
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
            self.get_logger().info('完成')
            nest_state = States.PICK_OBJECT

        # ----------- 6. 啟動吸盤 -----------
        elif state == States.PICK_OBJECT:
            self.get_logger().info('PICK_OBJECT: 啟動吸盤')
            self.call_io(VACUUM_PIN, True)
            print("吸盤已啟動，物體已吸取 (等待2秒)")
            time.sleep(0.5)
            nest_state = States.LOWER_UP_MOVE


        # ----------- 7. 上升回物體正上方（LINE）-----------
        elif state == States.LOWER_UP_MOVE:
            self.get_logger().info('LOWER_UP_MOVE: 上升回物體正上方')
            req = self.generate_robot_request(
                cmd_mode=RobotCommand.Request.LINE,
                cmd_type=RobotCommand.Request.POSE_CMD,
                pose=self.list_to_twist(LOWER_ABOVE_POSE),
                holding=True,
                velocity=LINE_VELOCITY,
                acceleration=LINE_ACCELERATION
            )
            res = self.call_hiwin(req)
            self.get_logger().info('完成')
            nest_state = States.ROTATE_READY_BACK


        # ----------- 8. 末端旋轉回準備吸取位置 -----------
        elif state == States.ROTATE_READY_BACK:
            self.get_logger().info('ROTATE_READY_BACK: 末端旋轉回準備吸取位置')
            req = self.generate_robot_request(
                cmd_mode=RobotCommand.Request.PTP,
                cmd_type=RobotCommand.Request.JOINTS_CMD,
                joints=JOINTS_READY,  # ← 跟第 2 步一樣
                holding=True
            )
            res = self.call_hiwin(req)
            self.get_logger().info('完成')
            time.sleep(3)
            nest_state = States.UPPER_FRONT_MOVE


        # ============================================================
        # 上層放置階段
        # ============================================================

        # ----------- 9. 移動到上層櫃子前方（PTP）-----------
        elif state == States.UPPER_FRONT_MOVE:
            self.get_logger().info('UPPER_FRONT_MOVE: 移動到上層櫃子前方')
            req = self.generate_robot_request(
                cmd_mode=RobotCommand.Request.PTP,
                cmd_type=RobotCommand.Request.POSE_CMD,
                pose=self.list_to_twist(UPPER_FRONT_POSE),
                holding=True
            )
            res = self.call_hiwin(req)
            self.get_logger().info('完成')
            nest_state = States.ROTATE_ABOVE_PLACE


        # ----------- 10. 末端旋轉到放置區上方 -----------
        elif state == States.ROTATE_ABOVE_PLACE:
            self.get_logger().info('ROTATE_ABOVE_PLACE: 末端旋轉到放置區上方')
            req = self.generate_robot_request(
                cmd_mode=RobotCommand.Request.PTP,
                cmd_type=RobotCommand.Request.JOINTS_CMD,
                joints=JOINTS_ABOVE_PLACE,
                holding=True
            )
            res = self.call_hiwin(req)
            self.get_logger().info('完成')
            nest_state = States.UPPER_DOWN_MOVE


        # ----------- 11. 下降到放置點（LINE）-----------
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
            self.get_logger().info('完成')
            nest_state = States.PLACE_OBJECT


        # ----------- 12. 關閉吸盤（放置）-----------
        elif state == States.PLACE_OBJECT:
            self.get_logger().info('PLACE_OBJECT: 關閉吸盤')
            self.call_io(VACUUM_PIN, False)
            print("吸盤已關閉，物體已放置 (等待2秒)")
            time.sleep(0.5)
            nest_state = States.UPPER_UP_MOVE


        # ----------- 13. 上升回放置區上方（LINE）-----------
        elif state == States.UPPER_UP_MOVE:
            self.get_logger().info('UPPER_UP_MOVE: 上升回放置區上方')
            req = self.generate_robot_request(
                cmd_mode=RobotCommand.Request.LINE,
                cmd_type=RobotCommand.Request.POSE_CMD,
                pose=self.list_to_twist(UPPER_ABOVE_POSE),
                holding=True,
                velocity=LINE_VELOCITY,
                acceleration=LINE_ACCELERATION
            )
            res = self.call_hiwin(req)
            self.get_logger().info('完成')
            nest_state = States.FINAL_HOME_MOVE


        # # ----------- 14. 末端旋轉回準備吸取位置 -----------
        # elif state == States.ROTATE_READY_FINAL:
        #     self.get_logger().info('ROTATE_READY_FINAL: 末端旋轉回準備吸取位置')
        #     req = self.generate_robot_request(
        #         cmd_mode=RobotCommand.Request.PTP,
        #         cmd_type=RobotCommand.Request.JOINTS_CMD,
        #         joints=JOINTS_READY,  # ← 跟第 2、8 步一樣
        #         holding=True
        #     )
        #     res = self.call_hiwin(req)
        #     self.get_logger().info('完成')
        #     nest_state = States.FINAL_HOME_MOVE


        # ============================================================
        # 結束階段
        # ============================================================

        # ----------- 15. 回到 HOME（PTP）-----------
        elif state == States.FINAL_HOME_MOVE:
            self.get_logger().info('FINAL_HOME_MOVE: 回到 HOME 位置')
            req = self.generate_robot_request(
                cmd_mode=RobotCommand.Request.PTP,
                cmd_type=RobotCommand.Request.POSE_CMD,
                pose=self.list_to_twist(HOME_POSE),
                holding=True
            )
            res = self.call_hiwin(req)
            self.get_logger().info('完成')
            nest_state = States.CLOSE_ROBOT


        # ----------- 16. 關閉機械臂 -----------
        elif state == States.CLOSE_ROBOT:
            self.get_logger().info('CLOSE_ROBOT: 關閉機械臂')
            req = self.generate_robot_request(
                cmd_mode=RobotCommand.Request.CLOSE
            )
            res = self.call_hiwin(req)
            nest_state = States.FINISH


        # ----------- 未知狀態 -----------
        else:
            print("Error: Invalid state")
            nest_state = None
            self.get_logger().error('Input state not supported!')

        return nest_state


    # ============================================================
    # IO 控制（不等待回應）
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
        request.velocity = DEFAULT_VELOCITY
        request.acceleration = DEFAULT_ACCELERATION
        request.tool = 7
        request.base = 6
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
            tool=7,
            base=6,
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