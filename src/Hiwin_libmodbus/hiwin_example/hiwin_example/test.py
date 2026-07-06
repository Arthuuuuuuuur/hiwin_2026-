#!/usr/bin/env python3
import time
import numpy as np
import rclpy
from enum import Enum
from threading import Thread
from rclpy.node import Node
from rclpy.task import Future
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist
from hiwin_interfaces.srv import RobotCommand

# ============================================================
# 速度設定
# ============================================================
DEFAULT_VELOCITY     = 10
DEFAULT_ACCELERATION = 10
LINE_VELOCITY        = 10
LINE_ACCELERATION    = 10

# ============================================================
# 吸盤針腳
# ============================================================
VACUUM_PIN = 1

# ============================================================
# 感測器判斷閾值
# ============================================================
STD_TILT_MIN    = 13.0   # 標準差 > 這個 → 傾斜
STD_TILT_MAX    = 25.0   # 標準差 > 這個 → 偏移
OVERLAP_THRESHOLD = 20.0  # 左右都小於這個 → 重疊

# 偏移調整距離（mm）
OFFSET_DISTANCE = 30.0

# # 傾斜 A4 補正角度（度）
# TILT_A4_OFFSET = 7.858   # 根據實際測量調整

# ============================================================
# 點位定義（佔位符，請填入實際座標）
# 格式：[X, Y, Z, Rx, Ry, Rz]
# ============================================================
HOME_POSE         = [0.00, 368.00, 293.50, -180.00, 0.00, 90.00]  # HOME
DETECT_POSE       = [-104.008, 367.993, -61.442, 180.00, 0.00, 90.00]  # 偵測位置（吸取點上方 3cm）
PICK_DOWN_POSE    = [-104.008, 367.993, -83.766, 180.00, 0.00, 90.00]   # 吸取點
PLACE_ABOVE_POSE  = [219.694, 367.993, -83.766, 180.00, 0.00, 90.00]  # 放置點上方
PLACE_DOWN_POSE   = [219.694, 367.993, -111.617, 180.00, 0.00, 90.00]  # 放置點

# ============================================================
# 關節角度定義（傾斜用）
# 格式：[A1, A2, A3, A4, A5, A6]
# ============================================================
JOINTS_NORMAL = [15.782, -27.164, -34.181, 0.00, -28.656, 15.782]  # 正常姿態角度
JOINTS_TILT   = [15.782, -27.164, -34.181, 7.858, -28.656, 15.782]  # 傾斜補正角度（A1 + A4 同時調整）

# 傾斜下降距離（mm）
TILT_DOWN_DISTANCE = 13.0  # 請依實際測量調整


# ============================================================
# 狀態列舉
# ============================================================
class States(Enum):
    INIT          = 0
    HOME_MOVE     = 1
    DETECT_MOVE   = 2
    READ_SENSOR   = 3
    JUDGE_STATE   = 4
    ADJUST_POSE   = 5
    DOWN_MOVE     = 6
    PICK_OBJECT   = 7
    UP_MOVE       = 8
    RESTORE_ANGLE = 9
    PLACE_MOVE    = 10
    PLACE_DOWN    = 11
    PLACE_OBJECT  = 12
    PLACE_UP      = 13
    FINAL_HOME    = 14
    SKIP          = 15
    FINISH        = 16


# ============================================================
# 策略節點
# ============================================================
class StrategyNode(Node):

    def __init__(self):
        super().__init__('strategy_node')

        self.hiwin_client = self.create_client(
            RobotCommand, 'hiwinmodbus_service'
        )

        self.sensor_subscription = self.create_subscription(
            Float32MultiArray,
            '/disc_sensor',
            self._sensor_callback,
            10
        )

        self.left_dist  = 0.0
        self.right_dist = 0.0
        self.disc_state = "UNKNOWN"

        # 動態調整後的點位
        self.adjusted_detect_pose = list(DETECT_POSE)
        self.adjusted_pick_pose   = list(PICK_DOWN_POSE)

        self.main_loop_thread = Thread(target=self._main_loop)
        self.main_loop_thread.daemon = True
        self.main_loop_thread.start()

        self.get_logger().info('Strategy Node 啟動完成')


    # ============================================================
    # 感測器 Callback
    # ============================================================
    def _sensor_callback(self, msg):
        self.left_dist  = msg.data[0]
        self.right_dist = msg.data[1]


    # ============================================================
    # 讀取感測器 5 次取平均
    # ============================================================
    def _read_sensor_avg(self, n=5):
        readings_left  = []
        readings_right = []

        for _ in range(n):
            readings_left.append(self.left_dist)
            readings_right.append(self.right_dist)
            time.sleep(0.05)

        avg_left  = float(np.mean(readings_left))
        avg_right = float(np.mean(readings_right))

        self.get_logger().info(
            f'感測器平均值：左={avg_left:.2f}mm 右={avg_right:.2f}mm'
        )
        return avg_left, avg_right


    # ============================================================
    # 判斷唱片狀態
    # ============================================================
    def _detect_disc_state(self, left, right):

        # 第一步：重疊（固定閾值）
        if left < OVERLAP_THRESHOLD and right < OVERLAP_THRESHOLD:
            self.get_logger().warn('判斷結果：重疊')
            return "OVERLAP"

        # 第二步：偏移/傾斜（標準差）
        std = float(np.std([left, right]))
        self.get_logger().info(f'標準差：{std:.2f}')

        if std > STD_TILT_MAX:
            if right > left:
                self.get_logger().info('判斷結果：偏左')
                return "OFFSET_LEFT"
            else:
                self.get_logger().info('判斷結果：偏右')
                return "OFFSET_RIGHT"

        if std > STD_TILT_MIN:
            self.get_logger().info('判斷結果：傾斜')
            return "TILT"

        self.get_logger().info('判斷結果：正常')
        return "NORMAL"


    # ============================================================
    # 主迴圈
    # ============================================================
    def _main_loop(self):
        state = States.INIT
        while state != States.FINISH:
            state = self._state_machine(state)
            if state is None:
                break
        self.destroy_node()


    # ============================================================
    # 狀態機
    # ============================================================
    def _state_machine(self, state: States) -> States:

        # ----------- INIT -----------
        if state == States.INIT:
            self.get_logger().info('INIT')
            return States.HOME_MOVE


        # ----------- 1. 回到 HOME -----------
        elif state == States.HOME_MOVE:
            self.get_logger().info('HOME_MOVE: 回到 HOME')
            req = self.generate_robot_request(
                cmd_mode=RobotCommand.Request.PTP,
                cmd_type=RobotCommand.Request.POSE_CMD,
                pose=self.list_to_twist(HOME_POSE),
                holding=True
            )
            self.call_hiwin(req)
            return States.DETECT_MOVE


        # ----------- 2. 移動到偵測位置 -----------
        elif state == States.DETECT_MOVE:
            self.get_logger().info('DETECT_MOVE: 移動到偵測位置')
            req = self.generate_robot_request(
                cmd_mode=RobotCommand.Request.PTP,
                cmd_type=RobotCommand.Request.POSE_CMD,
                pose=self.list_to_twist(DETECT_POSE),
                holding=True
            )
            self.call_hiwin(req)
            time.sleep(2.5)# 等手臂穩定
            return States.READ_SENSOR


        # ----------- 3. 讀取感測器 -----------
        elif state == States.READ_SENSOR:
            self.get_logger().info('READ_SENSOR: 讀取感測器')
            time.sleep(0.5)  # 等待感測器穩定
            self.avg_left, self.avg_right = self._read_sensor_avg(n=5)
            return States.JUDGE_STATE


        # ----------- 4. 判斷唱片狀態 -----------
        elif state == States.JUDGE_STATE:
            self.get_logger().info('JUDGE_STATE: 判斷唱片狀態')
            self.disc_state = self._detect_disc_state(
                self.avg_left, self.avg_right
            )
            if self.disc_state == "OVERLAP":
                return States.SKIP
            return States.ADJUST_POSE


        # ----------- 5. 調整姿態 -----------
        elif state == States.ADJUST_POSE:
            self.get_logger().info(f'ADJUST_POSE: 調整姿態（{self.disc_state}）')

            # 預設：不調整
            self.adjusted_detect_pose = list(DETECT_POSE)
            self.adjusted_pick_pose   = list(PICK_DOWN_POSE)

            if self.disc_state == "NORMAL":
                # 不需要調整
                self.get_logger().info('正常，不需要調整')

            elif self.disc_state == "OFFSET_LEFT":
                # POSE_CMD：X + 30mm
                self.get_logger().info(f'偏左，X + {OFFSET_DISTANCE}mm')
                self.adjusted_detect_pose[0] -= OFFSET_DISTANCE
                self.adjusted_pick_pose[0]   -= OFFSET_DISTANCE
                req = self.generate_robot_request(
                    cmd_mode=RobotCommand.Request.PTP,
                    cmd_type=RobotCommand.Request.POSE_CMD,
                    pose=self.list_to_twist(self.adjusted_detect_pose),
                    holding=True
                )
                self.call_hiwin(req)

            elif self.disc_state == "OFFSET_RIGHT":
                # POSE_CMD：X - 30mm
                self.get_logger().info(f'偏右，X - {OFFSET_DISTANCE}mm')
                self.adjusted_detect_pose[0] += OFFSET_DISTANCE
                self.adjusted_pick_pose[0]   += OFFSET_DISTANCE
                req = self.generate_robot_request(
                    cmd_mode=RobotCommand.Request.PTP,
                    cmd_type=RobotCommand.Request.POSE_CMD,
                    pose=self.list_to_twist(self.adjusted_detect_pose),
                    holding=True
                )
                self.call_hiwin(req)

            elif self.disc_state == "TILT":
                # JOINTS_CMD：調整 A1 + A4
                self.get_logger().info('傾斜，調整 A1 + A4 角度')
                req = self.generate_robot_request(
                    cmd_mode=RobotCommand.Request.PTP,
                    cmd_type=RobotCommand.Request.JOINTS_CMD,
                    joints=JOINTS_TILT,
                    holding=True
                )
                self.call_hiwin(req)

                # 讀取調整後的當前 XYZ
                res = self.call_hiwin(
                    self.generate_robot_request(
                        cmd_mode=RobotCommand.Request.CHECK_POSE
                    )
                )
                if res is not None:
                    current_xyz = list(res.current_position)
                    self.get_logger().info(
                        f'調整後 XYZ：{current_xyz}'
                    )
                    # 計算下降點（Z 往下 TILT_DOWN_DISTANCE）
                    self.tilt_pick_pose = list(current_xyz)
                    self.tilt_pick_pose[2] -= TILT_DOWN_DISTANCE
                    # 記錄上升點（回到 A4 調整後的位置）
                    self.tilt_above_pose = list(current_xyz)
                else:
                    self.get_logger().error('讀取 XYZ 失敗！')
                    return States.SKIP

            return States.DOWN_MOVE


        # ----------- 6. 下降到吸取點（LINE）-----------
        elif state == States.DOWN_MOVE:
            self.get_logger().info('DOWN_MOVE: 下降到吸取點')

            # 傾斜用讀取後計算的點位，其他用一般點位
            if self.disc_state == "TILT":
                pick_pose = self.tilt_pick_pose
            else:
                pick_pose = self.adjusted_pick_pose

            req = self.generate_robot_request(
                cmd_mode=RobotCommand.Request.LINE,
                cmd_type=RobotCommand.Request.POSE_CMD,
                pose=self.list_to_twist(pick_pose),
                holding=True,
                velocity=LINE_VELOCITY,
                acceleration=LINE_ACCELERATION
            )
            self.call_hiwin(req)
            return States.PICK_OBJECT


        # ----------- 7. 啟動吸盤 -----------
        elif state == States.PICK_OBJECT:
            self.get_logger().info('PICK_OBJECT: 啟動吸盤')
            self.call_io(VACUUM_PIN, True)
            time.sleep(0.5)
            return States.UP_MOVE


        # ----------- 8. 上升回偵測位置（LINE）-----------
        elif state == States.UP_MOVE:
            self.get_logger().info('UP_MOVE: 上升回偵測位置')

            # 傾斜：先上升到 A4 調整後的 XYZ，再回偵測位置
            if self.disc_state == "TILT":
                # 第一步：上升到傾斜調整後的 XYZ
                req = self.generate_robot_request(
                    cmd_mode=RobotCommand.Request.LINE,
                    cmd_type=RobotCommand.Request.POSE_CMD,
                    pose=self.list_to_twist(self.tilt_above_pose),
                    holding=True,
                    velocity=LINE_VELOCITY,
                    acceleration=LINE_ACCELERATION
                )
                self.call_hiwin(req)
                # 第二步：回到偵測位置 XYZ（PTP）
                req = self.generate_robot_request(
                    cmd_mode=RobotCommand.Request.PTP,
                    cmd_type=RobotCommand.Request.POSE_CMD,
                    pose=self.list_to_twist(DETECT_POSE),
                    holding=True
                )
                self.call_hiwin(req)
                return States.RESTORE_ANGLE
            else:
                # 一般情況：直接上升回偵測位置
                req = self.generate_robot_request(
                    cmd_mode=RobotCommand.Request.LINE,
                    cmd_type=RobotCommand.Request.POSE_CMD,
                    pose=self.list_to_twist(self.adjusted_detect_pose),
                    holding=True,
                    velocity=LINE_VELOCITY,
                    acceleration=LINE_ACCELERATION
                )
                self.call_hiwin(req)
                return States.PLACE_MOVE


        # ----------- 9. 恢復正常角度（傾斜專用）-----------
        elif state == States.RESTORE_ANGLE:
            self.get_logger().info('RESTORE_ANGLE: 恢復正常角度')
            req = self.generate_robot_request(
                cmd_mode=RobotCommand.Request.PTP,
                cmd_type=RobotCommand.Request.JOINTS_CMD,
                joints=JOINTS_NORMAL,
                holding=True
            )
            self.call_hiwin(req)
            return States.PLACE_MOVE


        # ----------- 10. 移動到放置點上方（PTP）-----------
        elif state == States.PLACE_MOVE:
            self.get_logger().info('PLACE_MOVE: 移動到放置點上方')
            req = self.generate_robot_request(
                cmd_mode=RobotCommand.Request.PTP,
                cmd_type=RobotCommand.Request.POSE_CMD,
                pose=self.list_to_twist(PLACE_ABOVE_POSE),
                holding=True
            )
            self.call_hiwin(req)
            return States.PLACE_DOWN


        # ----------- 11. 下降到放置點（LINE）-----------
        elif state == States.PLACE_DOWN:
            self.get_logger().info('PLACE_DOWN: 下降到放置點')
            req = self.generate_robot_request(
                cmd_mode=RobotCommand.Request.LINE,
                cmd_type=RobotCommand.Request.POSE_CMD,
                pose=self.list_to_twist(PLACE_DOWN_POSE),
                holding=True,
                velocity=LINE_VELOCITY,
                acceleration=LINE_ACCELERATION
            )
            self.call_hiwin(req)
            return States.PLACE_OBJECT


        # ----------- 12. 關閉吸盤 -----------
        elif state == States.PLACE_OBJECT:
            self.get_logger().info('PLACE_OBJECT: 關閉吸盤')
            self.call_io(VACUUM_PIN, False)
            time.sleep(0.5)
            return States.PLACE_UP


        # ----------- 13. 上升（LINE）-----------
        elif state == States.PLACE_UP:
            self.get_logger().info('PLACE_UP: 上升')
            req = self.generate_robot_request(
                cmd_mode=RobotCommand.Request.LINE,
                cmd_type=RobotCommand.Request.POSE_CMD,
                pose=self.list_to_twist(PLACE_ABOVE_POSE),
                holding=True,
                velocity=LINE_VELOCITY,
                acceleration=LINE_ACCELERATION
            )
            self.call_hiwin(req)
            return States.FINAL_HOME


        # ----------- 重疊跳過 -----------
        elif state == States.SKIP:
            self.get_logger().warn('SKIP: 重疊唱片，跳過')
            return States.FINAL_HOME


        # ----------- 14. 回到 HOME -----------
        elif state == States.FINAL_HOME:
            self.get_logger().info('FINAL_HOME: 回到 HOME')
            req = self.generate_robot_request(
                cmd_mode=RobotCommand.Request.PTP,
                cmd_type=RobotCommand.Request.POSE_CMD,
                pose=self.list_to_twist(HOME_POSE),
                holding=True
            )
            self.call_hiwin(req)
            return States.FINISH


        else:
            self.get_logger().error('未知狀態！')
            return None


    # ============================================================
    # IO 控制（吸盤）
    # ============================================================
    def call_io(self, pin, on_off):
        request = RobotCommand.Request()
        request.cmd_mode = RobotCommand.Request.DIGITAL_OUTPUT
        request.cmd_type = RobotCommand.Request.POSE_CMD
        request.digital_output_pin = pin
        request.digital_output_cmd = (
            RobotCommand.Request.DIGITAL_ON if on_off
            else RobotCommand.Request.DIGITAL_OFF
        )
        request.holding = False
        request.velocity = DEFAULT_VELOCITY
        request.acceleration = DEFAULT_ACCELERATION
        request.tool = 0
        request.base = 0
        request.pose = Twist()
        request.joints = [float('inf')] * 6
        request.circ_s = []
        request.circ_end = []
        request.jog_joint = 6
        request.jog_dir = 0
        request.move_dir = "z"
        request.move_dis = 0.01
        while not self.hiwin_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('等待服務...')
        future = self.hiwin_client.call_async(request)
        time.sleep(0.5)
        return None


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
    # list 轉 Twist
    # ============================================================
    def list_to_twist(self, pose_list):
        pose_ = Twist()
        [pose_.linear.x,  pose_.linear.y,  pose_.linear.z ] = [float(v) for v in pose_list[0:3]]
        [pose_.angular.x, pose_.angular.y, pose_.angular.z] = [float(v) for v in pose_list[3:6]]
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
            tool=0,
            base=0,
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
            self.get_logger().info('service not available, waiting...')
        future = self.hiwin_client.call_async(req)
        if self._wait_for_future_done(future):
            return future.result()
        return None


# ============================================================
# 主函數
# ============================================================
def main(args=None):
    rclpy.init(args=args)
    node = None

    try:
        node = StrategyNode()
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