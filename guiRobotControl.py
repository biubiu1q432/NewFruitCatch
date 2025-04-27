from nicegui import ui
import time
from math import pi
import serial
# from utils.Arm_Control import *
from utils.Car import *

# from rclpy.node import Node
# from std_msgs.msg import String
# from sensor_msgs.msg import Image
# from utils.Socket_Serve import *

class Arm:
    def __init__(self, local="left"):
        # self.arm_ser: serial.Serial
        self.arm_pwm = {
            "0": 1500,
            "1": 1500,
            "2": 1500,
            "3": 1500,
            "4": 1500,
            "5": 1500,
        }
        self.l0 = 75
        self.l1 = 260
        self.l2 = 200
        self.l3 = 150
        self.X = 0
        self.Y = 0
        self.Z = 0
        self.angle = 0
        self.is_move = None
        self.count = 0
        self.is_close = False
        self.num = 0

        self.local = local
        if self.local == "left":
            self.start_aim_fruit_action = (0, 220, 370, 14)
            self.start_aim_vegetable_action = (0, 320, 220, -82)
            self.pre_aim_action = (0, 230, 380, -30)
            self.start_aim_apple_action = (170, 10, 430, 18)
        else:
            self.start_aim_fruit_action = (0, 210, 320, -5)
            self.start_aim_vegetable_action = (0, 310, 200, -95)
            self.pre_aim_action = (0, 160, 300, -62)
            self.start_aim_apple_action = (190, 20, 340, -7)

    def init(self, Option) -> bool:
        try:
            self.arm_ser = serial.Serial(
                Option["port"], Option["baud_rate"]
            )
            if not self.arm_ser.is_open:
                self.arm_ser.open()  # 打开串口
            print("成功打开机械臂串")
            self.arm_ser.flushInput()
            return True
        except:
            print("机械臂串口打开失败")
            return False

    def get_ordinary(self, pwm1, pwm2, pwm3, pwm4):

        # l0 = 40
        # l1 = 105
        # l2 = 75
        # l3 = 200
        # 计算每个舵机角度
        if self.local == "left":
            theta1 = -(pwm1 - 1500) * 270 / 2000
            theta2 = (pwm2 - 1500) * 270 / 2000
        else:
            theta1 = (pwm1 - 1500) * 270 / 2000
            theta2 = -(pwm2 - 1500) * 270 / 2000
        theta3 = (pwm3 - 1500) * 270 / 2000
        theta4 = (pwm4 - 1500) * 270 / 2000
        # print(f"Theta1: {theta1}, Theta2: {theta2}, Theta3: {theta3}, Theta4: {theta4}")

        # if theta1 < -45 or theta1 > 135:
        #     return None
        theta4 = -theta4
        theta2 += 90
        angle = theta4 + theta2 - theta3
        theta3 = 180 - theta3

        theta1 = math.radians(theta1)
        theta2 = math.radians(theta2)
        theta3 = math.radians(theta3)
        theta4 = math.radians(theta4)
        # angle = math.radians(angle)
        aaa = math.cos(theta3)
        yz2 = -aaa * 2 * self.l1 * self.l2 + self.l1 * self.l1 + self.l2 * self.l2    # yz2为y2+z2
        bbb = (yz2 + self.l1 * self.l1 - self.l2 * self.l2) / (2 * self.l1 * math.sqrt(yz2))
        zf_ccc = theta2 - math.acos(bbb)
        # print(f"zf_ccc: {zf_ccc}")
        y = math.cos(zf_ccc) * math.sqrt(yz2)
        z = math.sqrt(yz2 - y * y)
        # print(z, l0)
        # print(l3 * math.sin(angle * pi / 180))

        y = y + self.l3 * math.cos(angle * pi / 180)    # 此时y为原始坐标x2+y2
        x = y * math.sin(theta1)
        y = y * math.cos(theta1)
        # if x > 0:
        if zf_ccc < 0:
            z = -z
        z = z + self.l0 + self.l3 * math.sin(angle * pi / 180)

        X = int(x)
        Y = int(y)
        Z = int(z)
        angle = int(angle)
        return [X, Y, Z, angle]

    def check_is_aim(self,x ,y, z, angle):
        for i in range(2):
            X, Y, Z, angle = self.get_ordinate()
            time.sleep(0.1)
        if abs(X - x) < 10 and abs(Y - y) < 10:
            return True
        return False

    def pre_aim(self, times=1000):
        """
        确定是抓水果还是蔬菜的动作
        """
        self.send_ordinate(*self.pre_aim_action, times=times)
        status = self.check_is_aim(*self.pre_aim_action)
        if status:
            return True
        else:
            return False

    def start_aim(self, mode="fruit"):
        """
        抓水果的前置动作
        """
        if mode == "fruit":
            start_aim_action = self.start_aim_fruit_action
        elif mode == "vegetable":
            start_aim_action = self.start_aim_vegetable_action
        else:
            start_aim_action = self.start_aim_apple_action

        if mode == "fruit" or "vegetable":
            if self.local == "right":
                self.send_pwm(5, 1400, times=800)
                time.sleep(0.2)

        else:
            if self.local == "left":
                self.send_pwm(5, 1700, times=500)
                time.sleep(0.2)

        self.send_ordinate(*start_aim_action, times=500)
        time.sleep(0.6)
        status = self.check_is_aim(*start_aim_action)
        if status:
            return True
        else:
            return False

    def receive(self):
        try:
            if self.arm_ser.in_waiting > 0:
                message = self.arm_ser.readline().decode('utf-8')
                message = message.strip()

                if message[0] == "@":
                    data = message.split('|')
                    index = data[0][1]
                    self.arm_pwm[index] = int(data[1])
                elif message[0] == "|":
                    if message[1] == "1":
                        self.is_move = True
                        # print("机械臂能到达坐标点")
                    elif message[1] == "0":
                        self.is_move = False

        except Exception as e:
            self.num += 1
            print(e)
            print("机械臂串口接收失败")

    def Receiver(self):
        while True:
            self.receive()
            if self.is_close or self.num > 10:
                break

    def rotate(self, move_angle, times=1000, mode="pwm", is_print=False):  # 底部舵机想要旋转的角度
        """
        :param angle:
        :param times:
        :param mode:
        :return:
        """
        X, Y, Z, angle = self.get_ordinate()
        if mode == "pwm":
            self.get_cur_pwm()
            time.sleep(0.1)

            pwm0 = self.arm_pwm["0"]
            if is_print:
                print(f"before_pwm: {pwm0}")
            pwm0 = pwm0 + int(move_angle * 2000 / 270)
            if is_print:
                print(f"after_pwm: {pwm0}")
            # print('pwm0', pwm0)
            if pwm0 < 500:
                pwm0 = 500
            if pwm0 > 2500:
                pwm0 = 2500
            self.send_pwm(0, int(pwm0), times=times)
        else:
            xy_angle = math.atan2(Y, X)
            arm_long = math.sqrt(X ** 2 + Y ** 2)
            need_rotate_angle = move_angle * math.pi / 180
            now_angle = xy_angle + need_rotate_angle
            add_x = arm_long * math.cos(now_angle)
            add_y = arm_long * math.sin(now_angle)
            self.send_ordinate(add_x, add_y, Z, angle, times=300, mode=0)

    def catch_apple_reset(self):
        pass

    def reset(self):
        self.arm_ser.write(("$DJR!\r\n").encode('utf-8'))

    def reset_except_claw(self, times=1500):
        if times < 1000:
            command = "{" + "#000P1500T0{times}!#001P1500T0{times}!#002P1500T0{times}!#003P1500T0{times}!".format(
                times=times) + "}"
        else:
            command = "{" + "#000P1500T{times}!#001P1500T{times}!#002P1500T{times}!#003P1500T{times}!".format(
                times=times) + "}"
        self.arm_ser.write(command.encode('utf-8'))

    def reset_except_lower(self, times=1500):
        if times < 1000:
            command = "{" + "#001P1500T0{times}!#002P1500T0{times}!#003P1500T0{times}!#005P1500T{times}!".format(
                times=times) + "}"
        else:
            command = "{" + "#001P1500T{times}!#002P1500T{times}!#003P1500T{times}!#005P1500T{times}!".format(
                times=times) + "}"
        self.arm_ser.write(command.encode('utf-8'))

    def reset_except_lower_and_claw(self, times=1500):
        if times < 1000:
            command = "{" + "#001P1500T0{times}!#002P1500T0{times}!#003P1500T0{times}!".format(
                times=times) + "}"
        else:
            command = "{" + "#001P1500T{times}!#002P1500T{times}!#003P1500T{times}!".format(
                times=times) + "}"
        self.arm_ser.write(command.encode('utf-8'))

    def put_down(self, times=400):
        time.sleep(0.2)
        if self.local == "left":
            self.send_pwm(0, 2165, times=700)
        else:
            self.send_pwm(0, 825, times=700)
        time.sleep(0.6)
        # self.reset()
        # time.sleep(0.3)
        if self.local == "left":
            self.send_ordinate(-190, -10, 360, -5, times=800, is_close_control=False)
            time.sleep(0.6)
        else:
            self.send_ordinate(-190, -10, 330, -5, times=800, is_close_control=False)
            time.sleep(0.6)

        self.send_pwm(5, 1600, times=times)
        time.sleep(0.5)
        self.reset_except_lower(times=times)
        time.sleep(0.5)
        self.reset()
        time.sleep(0.2)
        if self.local == "left":
            self.send_pwm(1, 1400, times=times)
        else:
            self.send_pwm(1, 1600, times=times)

        time.sleep(0.3)

        self.reset()
        time.sleep(0.4)
        self.send_pwm(0, 1500, times=times)
        return True

    def up(self, up_dis, times=500, need_forward=False, forward_num=10):
        x, y, z, angle = self.get_ordinate()
        if need_forward:
            y += forward_num
        status = self.send_ordinate(x, y, int(z + up_dis), angle, mode=0, times=times)
        return status

    def send_pwm(self, index: int, pwm: int, times=1000):
        self.arm_pwm[str(index)] = pwm
        if times < 1000:
            message = f"#00{index}P{pwm}T0{times}!\r\n"
        else:
            message = f"#00{index}P{pwm}T{times}!\r\n"
        self.arm_ser.write(message.encode('utf-8'))

    def send_ordinate_arr(self, x, y, z, angle, mode=0, times=1500):

        self.arm_ser.write(
            (f"$KMS:{int(x)},{int(y)},{int(z)},{times},{int(angle)},{mode}!\r\n").encode(encoding="UTF-8"))
        print(f"机械臂往 x：{int(x)}， y：{int(y)}， z：{int(z)}， angle：{int(angle)} 移动")

    def send_ordinate(self, x, y, z, angle, mode=0, times=1500, pos_ward=False, is_close_control=True):
        if abs(y) < 80 and abs(x) < 130:
            return False
        if x < -300:
            print("x过于靠右，请调整")
            return False
        self.is_move = None
        self.get_ordinate()
        time.sleep(0.3)
        self.send_ordinate_arr(x, y, z, angle, mode=mode, times=times)
        self.count = 0
        start_time = time.time()
        if is_close_control:
            while True:
                time.sleep(0.02)
                cur_time = time.time()
                if cur_time - start_time > 4:
                    print("未收到机械臂返回的结果")
                    return False

                if self.is_move:
                    time.sleep(1)
                    print("机械臂成功到达")
                    self.is_move = None
                    return True

                elif self.is_move is False:
                    self.count += 1
                    if mode == 0:
                        if self.count < 5:
                            if pos_ward:
                                self.send_ordinate_arr(x, y, z, angle + self.count * 5, mode=mode, times=times)
                            else:
                                self.send_ordinate_arr(x, y, z, angle - self.count * 5, mode=mode, times=times)
                        else:
                            self.is_move = None
                            return False
                    elif mode == 1:
                        if self.count < 5:
                            if pos_ward:
                                self.send_ordinate_arr(x + self.count * 5, y, z, angle, mode=mode, times=times)
                            else:
                                self.send_ordinate_arr(x - self.count * 5, y, z, angle, mode=mode, times=times)
                        else:
                            self.is_move = None
                            return False
        else:

            time.sleep(times * 0.001)
            self.is_move = None
            return True

    def get_pwm(self):
        return self.arm_pwm

    def get_cur_pwm(self):
        self.arm_ser.write(("@|\r\n").encode(encoding="UTF-8"))

    def get_ordinate(self):
        self.get_cur_pwm()
        time.sleep(0.3)
        pwm0 = self.arm_pwm['0']
        pwm1 = self.arm_pwm['1']
        pwm2 = self.arm_pwm['2']
        pwm3 = self.arm_pwm['3']
        result = self.get_ordinary(pwm0, pwm1, pwm2, pwm3)
        print(f"当前机械臂位置为 x：{result[0]}， y：{result[1]}， z：{result[2]}， angle：{result[3]}")
        return result

    def add_pwm(self, index: int, add_pwm: int, times: int=1000):
        self.get_cur_pwm()
        time.sleep(0.2)
        pwm = self.arm_pwm[str(index)]
        self.send_pwm(index, pwm + add_pwm, times=times)

    def get_data(self):
        return self.X, self.Y, self.Z, self.angle

    def run_thread(self):
        arm_receive_thread = threading.Thread(target=self.Receiver, daemon=True)
        # arm_update_thread = threading.Timer(1, self.update)
        arm_receive_thread.start()
        # arm_update_thread.start()

    def close(self):
        self.arm_ser.close()
        self.is_close = True
        print("机械臂串口关闭")



Option = {
        "left": {
            "video": {"video_ser": "/dev/video_left",
                      "node_name": "Image_Receiver_left",
                      "img_topic": "/dev/video_left",
                      },
            "local": "left",
            "arm_ser": {
                "port": "/dev/arm_left",
                "baud_rate": 115200
            }
        },

        "right": {
            "video": {"video_ser": "/dev/video_right",
                      "node_name": "Image_Receiver_right",
                      "img_topic": "/dev/video_right",
                      },
            "local": "right",
            "arm_ser": {
                "port": "/dev/arm_right",
                "baud_rate": 115200
            }
        },
        "Car_Serial": {
            "port": "/dev/car",
            "baud_rate": 115200
        }
    }


def class_init(Option):
    # img_receiver = Image_Receiver(name=Option["video"]["node_name"], topic=Option["video"]["img_topic"])
    # executor = rclpy.executors.SingleThreadedExecutor()
    # camera = Camera(Option["video"]["video_ser"], img_receiver=img_receiver, executor=executor, system_type="Linux")
    arm = Arm(local=Option["local"])
    # arm.init(Option["arm_ser"])
    # arm.run_thread()
    # catcher = Catcher(arm, camera)
    return arm

def serial_open(arm, Option):
    status = arm.init(Option["arm_ser"])
    if status:
        arm.run_thread()
        ui.notify("串口打开成功")
    else:
        ui.notify("串口打开失败")
    return arm

def serial_close(arm:Arm):
    arm.close()
    ui.notify("串口关闭成功")

# rclpy.init()
arm_left = class_init(Option["left"])
arm_right = class_init(Option["right"])
car = Car_Bridge()
# car.init(Option["Car_Serial"])
# car.run_thread()
def stop_move(args):
    text = "停止运动"
    car.stop()

def send_pwm(e, name, arm):
    input_value = e.args
    try:
        pwm = int(input_value)
        if pwm < 1500 or pwm > 2500:
            ui.notify('请输入1500-2500的数字')
        else:
            ui.notify(f'向{name}发送pwm值{pwm}')
    except:
        ui.notify('请输入数字')


def create_pwm_input(name, arm:Arm):
    with ui.row().style('align-items: center; gap: 10px;'):
        ui.label(name)
        ui.input('pwm值').on('change', lambda e: send_pwm(e, name, arm))


with ui.column().style('position: absolute; top: 50%; left: 60%; transform: translateY(-50%);'):
    ui.label("左机械臂")
    with ui.row().style('align-items: center; gap: 10px;'):
        ui.button('打开').on('click', lambda: serial_open(arm_left, Option["left"]))
        ui.button('关闭').on('click', lambda: serial_close(arm_left))
    create_pwm_input('舵机0', arm_left)
    create_pwm_input('舵机1', arm_left)
    create_pwm_input('舵机2', arm_left)
    create_pwm_input('舵机3', arm_left)
    create_pwm_input('舵机4', arm_left)
    create_pwm_input('舵机5', arm_left)

with ui.column().style('position: absolute; top: 50%; left: 75%; transform: translateY(-50%);'):
    ui.label("右机械臂")
    with ui.row().style('align-items: center; gap: 10px;'):
        ui.button('打开').on('click', lambda: serial_open(arm_right, Option["right"]))
        ui.button('关闭').on('click', lambda: serial_close(arm_right))
    create_pwm_input('舵机0', arm_right)
    create_pwm_input('舵机1', arm_right)
    create_pwm_input('舵机2', arm_right)
    create_pwm_input('舵机3', arm_right)
    create_pwm_input('舵机4', arm_right)
    create_pwm_input('舵机5', arm_right)

# 定义按钮的回调函数
def move(direction):
    ui.notify(f'移动: {direction}')


def go_distance(method):
    text = ""
    if method == "forward":
        # print("前进")
        text = "前进"
        ui.notify(text)
        car.go_distance_control(speed=0.2)
    elif method == "backward":
        # print("后退")
        text = "后退"
        ui.notify(text)
        car.go_distance_control(speed=-0.3)
    # ui.notify(text)


def stop_move(args):
    text = "停止运动"
    car.stop()


def yaw(method):
    text = ""
    if method == "left":
        # print("左转")
        text = "左转"
        ui.notify(text)
        car.yaw_adjustment_control(speed=1)
    elif method == "right":
        # print("右转")
        text = "右转"
        ui.notify(text)
        car.yaw_adjustment_control(speed=-1)

    # ui.notify(text)
remaps = {
    'W': {"action": go_distance, "args": 'forward'},
    'S': {"action": go_distance, "args": 'backward'},
    'A': {"action": yaw, "args": 'left'},
    'D': {"action": yaw, "args": 'right'},
    'stop move': {"action": stop_move, "args": None, "position": (0, 640), "size": (140, 50)},   # FIXME: 改一下位置
}

# 长按按钮
def add_button_longpress(name, style=None):

    press_timer = None
    button = ui.button(name)
    if style is not None:
        button.style(style)
    else:
        button.style('width: 80px; height: 80px;')
    # 兼容鼠标长按识别
    button.on('mousedown', lambda: start_timer(remaps[name]['action'], remaps[name]['args']))
    button.on('mouseup', lambda: cancel_press(press_timer))
    button.on('mouseleave', lambda: cancel_press(press_timer))
    # 兼容手机触屏识别
    button.on('touchstart', lambda: start_timer(remaps[name]['action'], remaps[name]['args']))
    button.on('touchend', lambda: cancel_press(press_timer))
    button.on('touchcancel', lambda: cancel_press(press_timer))

    # 启动长按定时器的函数
    def start_timer(repeat_action, args):
        nonlocal press_timer
        # 每0.5秒重复调用 `repeat_action`
        press_timer = ui.timer(0.2, lambda: repeat_action(args), active=True)  # 用 lambda 延迟调用

    # 定义一个函数，取消定时器
    def cancel_press(timer):
        if timer:
            stop_move()
            timer.deactivate()

def create_move_button():
    with ui.grid(columns=3).style('position: absolute; top: 50%; left: 40%; transform: translate(-50%, -50%);'):
        ui.label('')  # 空白占位
        add_button_longpress('W')  # 上方 W
        ui.label('')  # 空白占位
        add_button_longpress('A')  # 左方 A
        add_button_longpress('s')
        add_button_longpress('D')
        ui.label('')  # 空白占位
        add_button_longpress('stop move')



create_move_button()



ui.run()


# def yaw(method):
#     text = ""
#     if method == "left":
#         #print("左转")
#         text = "左转"
#         ui.notify(text)
#         #robot.car_bridge.yaw_adjustment(-1)
#     elif method == "right":
#         #print("右转")
#         text = "右转"
#         ui.notify(text)
#         #robot.car_bridge.yaw_adjustment(1)
#     #ui.notify(text)
#
# def lift(method):
#     text = ""
#     if method == "up":
#         arm_states[2] = 1
#         #print("上升")
#         text = "上升"
#         ui.notify(text)
#         #robot.arm_bridge.arm_cmd(arm_states)
#         time.sleep(1.8)
#     elif method == "down":
#         if arm_states[1][0] == 1 or arm_states[1][1] == 1:
#             arm_states[1][0], arm_states[1][1] = 0, 0
#             text = "先 保证两轴外折, 再 "
#             #robot.arm_bridge.arm_cmd(arm_states)
#             time.sleep(1)
#         arm_states[2] = 0
#         text += "下降"
#         ui.notify(text)
#         #robot.arm_bridge.arm_cmd(arm_states)
#         time.sleep(1.8)
#     #ui.notify(text)
#
#
# remaps = {
#     'up': {"action": go_distance, "args": 'forward', "position": (120, 100), "size": (80, 50)},
#     'down': {"action": go_distance, "args": 'backward', "position": (180, 100), "size": (80, 50)},
#     'left': {"action": yaw, "args": 'left', "position": (180, 10), "size": (80, 50)},
#     'right': {"action": yaw, "args": 'right', "position": (180, 190), "size": (80, 50)},
#     'lift up': {"action": lift, "args": "up", "position": (230, 500), "size": (140, 50)},
#     'lift down': {"action": lift, "args": "down", "position": (230, 640), "size": (140, 50)},
#     'left elbow': {"action": lift_elbow, "args": "left",  "position": (155, 500), "size": (140, 50)},
#     'right elbow': {"action": lift_elbow, "args": "right", "position": (155, 640), "size": (140, 50)},
#     'left craw': {"action": craw, "args": "left", "position": (80, 500), "size": (140, 50)},
#     'right craw': {"action": craw, "args": "right", "position": (80, 640), "size": (140, 50)},
# }
#
#
#
# def add_button(name):
#     f = f"""
#         position: absolute;
#         top: {remaps[name]["position"][0]}px;
#         left: {remaps[name]["position"][1]}px;
#         width: {remaps[name]["size"][0]}px;
#         height: {remaps[name]["size"][1]}px;
#         """
#     ui.button(name).style(f).on('click', lambda: remaps[name]['action'](remaps[name]['args']))
#
#
#
# add_button('up')
# add_button('down')
# add_button('left')
# add_button('right')
#
# add_button('lift up')
# add_button('lift down')
# add_button('left elbow')
# add_button('right elbow')
# add_button('left craw')
# add_button('right craw')
#
# ui.run()