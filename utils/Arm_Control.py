import serial
import time
import math
import threading
from math import pi
import cv2
from datetime import datetime
from utils.Camera import *
from utils.Run import *


class Arm:
    def __init__(self, local="left"):
        # self.arm_ser: serial.Serial
        self.arm_pwm = {
            "0": 1500,
            "1": 1370,
            "2": 1350,
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
            self.start_aim_fruit_action = (0, 180, 430, 14)
            self.start_aim_vegetable_action = (0, 320, 220, -82)
            # self.pre_aim_action1 = (0, 205, 380, -36)
            self.pre_aim_action1 = (0, 205, 400, -45)
            # self.pre_aim_action2 = (0, 240, 400, 2)
            self.pre_aim_action2 = (0, 220, 430, 5)
            self.start_aim_apple_action = (170, 10, 422, 18)
        else:
            self.start_aim_fruit_action = (0, 180, 305, -8)
            self.start_aim_vegetable_action = (0, 315, 185, -95)
            self.pre_aim_action1 = (0, 100, 340, -66)
            self.pre_aim_action2 = (0, 180, 350, -15)
            self.start_aim_apple_action = (190, 20, 330, -7)

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

    def check_is_aim(self, x, y, z, angle):
        for i in range(2):
            X, Y, Z, angle = self.get_ordinate()
            time.sleep(0.1)
        if abs(X - x) < 10 and abs(Y - y) < 10 and abs(Z - z) < 15:
            return True
        return False

    def pre_aim(self, times=1000, action=0):
        """
        确定是抓水果还是蔬菜的动作
        """
        if action == 0:
            pre_aim_action = self.pre_aim_action1
        else:
            pre_aim_action = self.pre_aim_action2
        self.send_ordinate(*pre_aim_action, times=times)
        status = self.check_is_aim(*pre_aim_action)
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
        elif mode is None:
            start_aim_action = None
        else:
            start_aim_action = self.start_aim_apple_action

        if mode == "fruit" or mode == "vegetable":
            if self.local == "right":

                self.send_pwm(5, 1000, times=800)
                time.sleep(0.8)
                if mode == "fruit":
                    self.send_pwm(1, 1320)
                    time.sleep(0.6)
            else:
                self.send_pwm(5, 1200, times=800)
                if mode == "fruit":
                    self.send_pwm(1, 1680)
                    time.sleep(0.6)


        else:
            if self.local == "left":
                self.send_pwm(5, 1700, times=500)
                time.sleep(0.2)
            else:
                self.send_pwm(5, 1650, times=500)
        if start_aim_action is None:
            self.reset()
            return True
        else:
            self.send_ordinate(*start_aim_action, times=1700)
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
        for i in range(2):
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
        time.sleep(0.5)
        if self.local == "right":
            self.send_pwm(3, 1650, times=400)
        return True

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
            self.send_ordinate(-190, -10, 340, -5, times=800, is_close_control=False)
            time.sleep(0.6)

        self.send_pwm(5, 1600, times=times)
        time.sleep(0.8)
        if self.local == "left":
            self.reset_except_lower(times=800)
            time.sleep(1.6)
        else:
            self.reset_except_lower(times=times)
            time.sleep(1)
        self.reset()
        time.sleep(0.2)
        # if self.local == "left":
        #     self.send_pwm(1, 1400, times=times)
        # else:
        #     self.send_pwm(1, 1600, times=times)

        # time.sleep(0.3)

        # self.reset()
        time.sleep(0.4)
        # self.send_pwm(0, 1500, times=times)
        return True

    def up(self, up_dis, times=500, need_forward=False, x_forward_num=0, y_forward_num=0):
        for _ in range(2):
            x, y, z, angle = self.get_ordinate()
        if need_forward:
            y += y_forward_num
            x += x_forward_num
        if up_dis > 0:
            status = self.send_ordinate(x, y, int(z + up_dis), angle, mode=0, times=times, pos_ward=True)
        else:
            status = self.send_ordinate(x, y, int(z + up_dis), angle, mode=0, times=times)
        return status

    def send_pwm(self, index: int, pwm: int, times=1000):
        self.arm_pwm[str(index)] = pwm
        if times < 1000:
            if pwm < 1000:
                message = f"#00{index}P0{pwm}T0{times}!\r\n"
            else:
                message = f"#00{index}P{pwm}T0{times}!\r\n"
        else:
            if pwm < 1000:
                message = f"#00{index}P0{pwm}T{times}!\r\n"
            else:
                message = f"#00{index}P{pwm}T{times}!\r\n"
        self.arm_ser.write(message.encode('utf-8'))

    def send_ordinate_arr(self, x, y, z, angle, mode=0, times=1500):

        self.arm_ser.write(
            (f"$KMS:{int(x)},{int(y)},{int(z)},{times},{int(angle)},{mode}!\r\n").encode(encoding="UTF-8"))
        print(f"机械臂往 x：{int(x)}， y：{int(y)}， z：{int(z)}， angle：{int(angle)} 移动")

    def send_ordinate(self, x, y, z, angle, mode=0, times=1500, pos_ward=False, is_close_control=True):
        """
        :param x: 
        :param y: 
        :param z: 
        :param angle: 
        :param mode: 模式0为发的坐标到不了而迭代角度，模式1
        :param times: 
        :param pos_ward: 迭代坐标是增加还是减少
        :param is_close_control: 是否等待机械臂返回值
        :return: 
        """
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
        time.sleep(0.1)

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

class Catcher:
    def __init__(self, arm: Arm, camera: Camera):
        # frame = camera.get_frame()
        # self.w = frame.shape[1]
        # self.h = frame.shape[0]
        self.w = 640
        self.h = 480
        self.Z_height = 120  # 机械臂云台离地高度
        self.l0 = 75    # 机械臂第一段臂长
        self.f = 1.8
        self.count = 0

        self.px = 0.0014
        self.Step = 0
        self.num = 0
        self.cam_arm_dis = 65  # 相机与机械臂末端的距离
        self.arm = arm
        self.camera = camera

    def detect_x_dis(self, is_show=False):

        results = self.camera.detect(conf=0.77, limit_area=2000, is_show=is_show)
        x_move = None
        result = results[0]
        if result["color"] is not None:
            if "red" in result["color"] or "green" in result["color"]:
                x_move = result["x_move"]
            return x_move
        else:
            return None

    def aim_vegetable(self, is_show=False):
        mode = "vegetable"
        for i in range(3):
            self.camera.get_frame()
            results = self.camera.detect(conf=0.75, limit_area=4000, mode=mode, is_show=is_show)
            result = results[0]
            print(result)
            if result["color"] is not None:

                if "red" in result["color"]:
                    x_distance = result["x_distance"]
                    y_distance = result["y_distance"]
                    x_move = result["x_move"]
                    y_move = result["y_move"]
                    deep = result["deep"]
                    X, Y, Z, angle = self.arm.get_ordinate()

                    if abs(x_distance) > 50 + i * 5 or abs(
                            y_distance) > 50 + i * 5:  # 中心点的阈值，离物体越近，阈值愈大

                        if self.arm.local == "left":
                            move_X = int(X + x_move)
                        else:
                            move_X = int(X - x_move)

                        if abs(y_move) > 300:
                            y_move = y_move / 4
                        elif abs(y_move) > 200:
                            y_move = y_move / 3
                        elif abs(y_move) > 100:
                            y_move = y_move / 2
                        move_Y = int(Y - y_move)
                        print(f"move_X: {move_X}, move_Y: {move_Y}")

                        Z = Z - 10
                        if move_Y > 400:
                            if i == 0:
                                if move_Y > 440:
                                    Z = Z - 30
                                    angle += 25
                                    move_Y -= 25
                                else:
                                    if self.arm.local == "right":
                                        angle += 10

                        print(f"修改后的move_X: {move_X}， move_Y: {move_Y}， Z: {Z}， angle: {angle}")

                        if is_show:
                            input("按任意键继续")
                        for _ in range(3):
                            status = self.arm.send_ordinate(move_X, move_Y, Z-10, angle, times=200, pos_ward=True)
                            # time.sleep(0.5)
                            if status:
                                time.sleep(1.1)
                                break
                            else:
                                move_X -= move_X / abs(move_X) * 20
                                move_Y -= move_Y / abs(move_Y) * 20
                    else:
                        print("成功对齐")
                        return True, result

                elif "green" in result["color"]:
                    return False, None

                elif result["color"] is False:
                    print("摄像头出现问题")

                else:
                    return False, None

        if result["color"] is not None:
            if "red" in result["color"]:
                return True, result
        else:
            print("对齐次数过多或者摄像头出问题了")
            return None, None

    def aim_fruit(self, is_show=False):
        mode = "fruit"
        for i in range(3):      # 多次尝试
            # if is_detect:
            self.camera.get_frame()     # 避免摄像头延时
            results = self.camera.detect(conf=0.63, limit_area=6000, mode=mode, is_show=is_show)
            print(results)
            result = results[0]     # 取面积最大的一个
            if result["color"] is not None:
                if "red" in result["color"]:
                    x_distance = result["x_distance"]
                    y_distance = result["y_distance"]
                    x_move = result["x_move"]
                    y_move = result["y_move"]
                    deep = result["deep"]
                    w = result["w"]
                    h = result["h"]
                    area = result["area"]
                    X, Y, Z, angle = self.arm.get_ordinate()

                    if w > 350 and h > 230:
                        print("成功对齐")
                        return True, result

                    if abs(x_distance) > 65 + i * 5 or abs(
                            y_distance) > 65 + i * 5:  # 中心点的阈值，离物体越近，阈值愈大

                        xy_angle = math.atan2(Y, X)
                        arm_long = math.sqrt(X ** 2 + Y ** 2)
                        need_rotate_angle = math.atan2(x_move, arm_long + deep)
                        need_rotate_angle = need_rotate_angle * 180 / math.pi
                        need_rotate_angle = -round(need_rotate_angle, 2)
                        now_angle = xy_angle + need_rotate_angle

                        print(f"need_rotate_angle: {need_rotate_angle}")
                        # print(f"add_x: {add_x}, add_y: {add_y}")

                        if is_show:
                            time.sleep(0.5)
                            input("请稍后")

                        if self.arm.local == "right":
                            if abs(need_rotate_angle) > 3 and i <= 2:

                                self.arm.rotate(need_rotate_angle, times=200)
                                time.sleep(0.4)
                        else:
                            if abs(need_rotate_angle) > 2 and i <= 2:
                                self.arm.rotate(need_rotate_angle, times=300)
                                time.sleep(0.5)
                            else:
                                bias = 20
                                now_angle = abs(now_angle)
                                add_x = bias * math.cos(now_angle * math.pi / 180) * (-1 if x_distance > 0 else 1)
                                add_y = bias * math.sin(now_angle * math.pi / 180)
                                move_z = int(Z - y_move / 4)
                                if i == 0:
                                    angle -= 6
                                    add_y += 40

                                self.arm.send_ordinate(X - add_x, Y + add_y, move_z, angle, times=300, mode=0)

                        if abs(y_move) > 20:
                            if abs(y_move) > 100:
                                y_move = y_move / 2
                            if self.arm.local == "left":
                                if area > 25000:
                                    y_forward_num = -20
                                else:
                                    y_forward_num = 30
                                status = self.arm.up(-y_move, times=500, need_forward=True, y_forward_num=y_forward_num)
                            else:
                                if area > 25000:
                                    y_forward_num = -20
                                else:
                                    y_forward_num = 20
                                status = self.arm.up(-y_move, times=500, y_forward_num=y_forward_num)
                            if not status:
                                self.arm.up(-y_move, times=500, need_forward=True, y_forward_num=40)

                        time.sleep(0.4)

                    else:
                        print("水果成功对齐")
                        return True, result

                elif result["color"] is False:
                    print("摄像头出现问题，请检查摄像头")

                else:
                    return False, None

            # else:   # 代表无成熟或者没有水果的情况
            #     return False, None
        if result["color"] is not None:
            if "red" in result["color"]:
                return True, result
        else:
            print("对齐次数过多或者摄像头出问题了")
            return None, None

    def catch_vegetable(self, num=3):
        self.camera.get_frame()  # 避免摄像头延时
        results = self.camera.detect(conf=0.8, limit_area=5000, mode="vegetable")
        result = results[0]
        print(f"catch_result:  {result}")
        if result["color"] is not None:
            if "red" in result["color"]:
                x_distance = result["x_distance"]
                y_distance = result["y_distance"]
                x_move = result["x_move"]
                y_move = result["y_move"]
                deep = result["deep"]
                X, Y, Z, angle = self.arm.get_ordinate()
                height = Z + self.Z_height
                if x_move > 0:
                    flag_x = -1
                else:
                    flag_x = 1

                if Y != 0:
                    XY_angle = math.atan2(X, Y)
                else:
                    XY_angle = flag_x * math.pi / 2

                if self.arm.local == "right":
                    if abs(angle+5) < 88:
                        print("距离稍远的情况")
                        bias = height / math.tan(abs(angle)*math.pi/180) + 30
                        catch_Y = int(Y) + bias * math.cos(XY_angle)
                        catch_X = int(X) + (bias * math.sin(XY_angle) + 30)
                        catch_angle = angle + 20
                    else:
                        if y_move > 0:
                            catch_Y = int(Y) + 20
                        else:
                            catch_Y = int(Y) - y_move * 1.3 + 25

                        catch_X = int(X) - x_move * 1.5
                        catch_angle = angle

                    if catch_Y > 480:
                        catch_angle = catch_angle + 15
                    if abs(catch_angle) > 75:
                        catch_Z = -55
                    else:
                        catch_Z = -65

                else:
                    if abs(angle-16) < 82:
                        print("距离稍远的情况")
                        bias = height / math.tan(abs(angle-17)*math.pi/180)     # + 40
                        catch_Y = int(Y) + bias * math.cos(XY_angle)
                        catch_X = int(X) + bias * math.sin(XY_angle) * flag_x

                    else:
                        catch_Y = int(Y) - y_move + 50
                        catch_X = int(X) + x_move
                    catch_angle = angle + 10

                    catch_Z = -36

                print(f"修改后catch_X:{catch_X}, catch_Y:{catch_Y}, catch_Z:{catch_Z}, catch_angle:{catch_angle}")
                for i in range(3):    # 多次发送坐标
                    status = self.arm.send_ordinate(catch_X, catch_Y, catch_Z, catch_angle, times=800, mode=0, pos_ward=True)
                    if status:
                        time.sleep(0.55)
                        if self.arm.local == "right":
                            self.arm.send_pwm(5, 2100, times=700)
                        else:
                            self.arm.send_pwm(5, 1920, times=700)
                        time.sleep(0.6)
                        # self.arm.add_pwm(3, -200, times=500)
                        # time.sleep(0.2)
                        if self.arm.local == "right":
                            self.arm.send_ordinate(catch_X, catch_Y-60, catch_Z + 90, catch_angle + 20, times=600)
                        time.sleep(0.6)
                        # self.arm.send_pwm(5, 2000, times=500)
                        # time.sleep(0.2)
                        self.arm.reset_except_claw()
                        time.sleep(0.2)
                        return True
                    else:
                        arm_long = self.arm.l1 + self.arm.l2 + self.arm.l3
                        goal_long = math.sqrt(catch_X ** 2 + catch_Y ** 2 + (catch_Z-self.arm.l0) ** 2)
                        if goal_long > arm_long:
                            arm_long -= 30
                            # catch_Z -= 20
                            catch_Y = arm_long * math.cos(XY_angle)
                            catch_X = arm_long * math.sin(XY_angle)

                return False

            elif result["color"] is False:
                return None

            else:
                return False

        else:
            return False

    def catch_fruit(self, num=3):
        self.camera.get_frame()  # 避免摄像头延时
        results = self.camera.detect(conf=0.7, limit_area=5000, mode="fruit")
        result = results[0]
        print(f"catch_result:  {result}")
        if result["color"] is None:
            results = self.camera.color_detect(mode="fruit")
            result = results[0]
        if result["color"] is not None:
            if "red" in result["color"]:
                x_distance = result["x_distance"]
                y_distance = result["y_distance"]
                x_move = result["x_move"]
                y_move = result["y_move"]
                deep = result["deep"]
                area = result["area"]

                X, Y, Z, angle = self.arm.get_ordinate()
                # print(f'当前机械臂位置 X:{X}, Y:{Y}, Z:{Z}, angle:{angle}')
                height = Z + self.Z_height

                if X < 0:
                    flag_x = -1
                else:
                    flag_x = 1
                if Y != 0:
                    XY_angle = math.atan2(X, Y)
                else:
                    XY_angle = flag_x * math.pi / 2

                catch_Y = int(deep * math.cos(XY_angle) + Y)
                catch_X = int(deep * math.sin(XY_angle) + X)
                catch_Z = int(Z - y_move)
                catch_angle = math.atan2(Z - self.l0, math.sqrt(catch_X ** 2 + catch_Y ** 2))
                catch_angle = catch_angle * 180 / pi - 18

                print(f"catch_X: {catch_X}, catch_Y: {catch_Y}, catch_Z: {catch_Z}, catch_angle: {catch_angle}")
                # if self.arm.local == "right":
                if self.arm.local == "right":
                    if deep < 185:
                        catch_Z -= 20
                    if catch_Y < 400:
                        if abs(x_move) < 3 or abs(y_move) <= 3:
                            catch_Y += 25
                        else:
                            catch_Y += 10
                        bias = 78
                    else:
                        bias = 95
                else:
                    catch_Z -= 10
                    bias = 80  # 50
                catch_Y -= bias * math.cos(XY_angle)
                catch_X -= bias * math.sin(XY_angle) * flag_x
                catch_Z += 8

                if area > 150000:
                    if self.arm.local == "right":
                        catch_Y = Y + 25
                        catch_angle = catch_angle - 30
                        catch_Z -= 20
                    else:
                        catch_Y -= 25
                        catch_angle = catch_angle - 20

                print(f"修改后catch_X:{catch_X}, catch_Y:{catch_Y}, catch_Z:{catch_Z}, catch_angle:{catch_angle}")
                # if catch_Y < 290:
                #     catch_Y = 290

                for i in range(num):    # 多次发送坐标
                    status = self.arm.send_ordinate(catch_X, catch_Y, catch_Z, catch_angle, times=1000, mode=0, pos_ward=True)
                    if status:
                        time.sleep(0.1)
                        if self.arm.local == "right":
                            self.arm.send_pwm(5, 2000, times=1000)
                        else:
                            self.arm.send_pwm(5, 1950, times=1000)
                        time.sleep(0.7)
                        # 抬起大臂，使水果脱离钩子
                        if self.arm.local == "right":
                            self.arm.add_pwm(1, -100, times=600)
                        else:
                            self.arm.add_pwm(1, 100, times=600)
                        time.sleep(1)
                        # if self.arm.local == "left":
                        #     time.sleep(0.3)
                        self.arm.reset_except_claw()
                        return True
                    else:
                        arm_long = self.arm.l1 + self.arm.l2 + self.arm.l3
                        goal_long = math.sqrt(catch_X ** 2 + catch_Y ** 2 + (catch_Z-self.arm.l0) ** 2)
                        if goal_long > arm_long:
                            print(11111111111111111111)
                            if catch_Y > 580:
                                arm_long -= 20
                            else:
                                arm_long -= 65
                            catch_Z -= 20
                            catch_Y = arm_long * math.cos(XY_angle)
                            catch_X = arm_long * math.sin(XY_angle)
                            # catch_angle = math.atan2(Z - self.l0, math.sqrt(catch_X ** 2 + catch_Y ** 2))
                        else:
                            bias = 25
                            catch_Y -= bias * math.cos(XY_angle)
                            catch_X -= bias * math.sin(XY_angle) * flag_x

                return False

            elif result["color"] is False:
                print("摄像头出问题了")
                return None

            else:
                return False

        else:
            return False

    def detect_apple(self, is_show=False, choose="left"):
        frame, results = self.camera.model_detect(conf=0.8, is_show=False, limit_area=11000)
        print(results)

        if results[0]["color"] != "None":
            results.sort(key=lambda x: x["area"], reverse=True)
        if len(results) >= 2:
            results = results[:2]
            result1 = results[0]
            result2 = results[1]
            if result1["color"] == "red_apple" and result2["color"] == "red_apple":
                if result1["area"] - result2["area"] <= 3000 and abs(result1["deep"] - result2["deep"]) <= 40: # 两个大小相似的红苹果
                    if result1["cx"] < result2["cx"]:
                        if choose == "left":
                            goal = result1
                        else:
                            goal = result2
                    else:
                        if choose == "left":
                            goal = result2
                        else:
                            goal = result1

                else:   # 估计是远近不相同的红苹果
                    goal = result1

            elif result1["color"] == "red_apple":
                goal = result1
            elif result2["color"] == "red_apple":
                goal = result2
            else:
                goal = {"color": "None"}
        else:
            if results[0]["color"] == "red_apple":
                goal = results[0]
            else:
                goal = {"color": "None"}

        if goal["color"] != "None":

            cx = goal["cx"]
            cy = goal["cy"]
            w = goal["w"]
            h = goal["h"]
            x1 = cx - 0.5 * w
            y1 = cy - 0.5 * h
            cv2.putText(frame,
                            'goal_apple', (int(x1), int(y1 - 2)),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.60, (255, 0, 255),
                            thickness=1)
            # 获取当前时间，并格式化为文件名
            current_time = time.strftime("%m-%d %H:%M:%S", time.localtime())
            cv2.imwrite(f"/home/dianfei/catch_robot_v2/picture/{self.arm.local}_detect_apple_{current_time}.jpg", frame)
            if is_show:
                cv2.imshow("goal_apple_image", frame)
                cv2.waitKey(0)

        return goal

    def aim_apple(self, is_show=False):
        for i in range(2):      # 多次尝试
            # if is_detect:
            result = self.detect_apple(is_show=is_show, choose=self.arm.local)
            if "red" in result["color"]:
                x_distance = result["x_distance"]
                y_distance = result["y_distance"]
                x_move = result["x_move"]
                y_move = result["y_move"]
                deep = result["deep"]
                w = result["w"]
                h = result["h"]
                X, Y, Z, angle = self.arm.get_ordinate()

                if w > 350 and h > 230:
                    print("成功对齐")
                    return True, result

                if abs(x_distance) > 65 + i * 5 or abs(
                        y_distance) > 65 + i * 5:  # 中心点的阈值，离物体越近，阈值愈大
                    xy_angle = math.atan2(Y, X)
                    arm_long = math.sqrt(X ** 2 + Y ** 2)
                    need_rotate_angle = math.atan2(x_move, arm_long + deep)
                    need_rotate_angle = need_rotate_angle * 180 / math.pi
                    need_rotate_angle = -round(need_rotate_angle, 2)
                    now_angle = xy_angle + need_rotate_angle

                    print(f"need_rotate_angle: {need_rotate_angle}")
                    # print(f"add_x: {add_x}, add_y: {add_y}")

                    if is_show:
                        time.sleep(0.5)
                        input("请稍后")
                    if y_move > 40:
                        angle -= 5
                    time.sleep(1)
                    if self.arm.local == "right":
                        if abs(need_rotate_angle) > 3 and i <= 2:

                            self.arm.rotate(need_rotate_angle, times=200)
                            time.sleep(0.4)
                    else:
                        if abs(need_rotate_angle) >= 4 and i <= 1:
                            if i == 1:
                                need_rotate_angle = need_rotate_angle * 0.9
                            self.arm.rotate(need_rotate_angle, times=200, mode="pwm")
                            time.sleep(0.4)
                        else:
                            bias = 30
                            now_angle = abs(now_angle)
                            add_x = bias * math.cos(now_angle * math.pi / 180) * (-1 if x_distance > 0 else 1)
                            add_y = bias * math.sin(now_angle * math.pi / 180)
                            move_z = int(Z - y_move / 4)

                            self.arm.send_ordinate(X + add_x, Y + add_y, move_z, angle, times=300, mode=0)

                    if abs(y_move) > 20:
                        if abs(y_move) > 90:
                            y_move = y_move / 2
                        if i == 0:  # 机械臂前探距离
                            forward_num = 40
                        else:
                            forward_num = 20

                        if self.arm.local == "left":
                            forward_num += 40
                            status = self.arm.up(-y_move, times=400, need_forward=True, x_forward_num=forward_num)
                        else:
                            status = self.arm.up(-y_move, times=400, need_forward=True, x_forward_num=forward_num)

                        if not status:
                            self.arm.up(-y_move, times=500, need_forward=True, x_forward_num=60)

                    time.sleep(0.4)

                else:
                    print("水果成功对齐")
                    return True, result

            elif result["color"] is False:
                print("摄像头出现问题，请检查摄像头")

            else:   # 代表无成熟的情况
                if i == 0:
                    if self.arm.local == "left":
                        self.arm.add_pwm(0, 200, times=300)
                    else:
                        self.arm.add_pwm(0, -150, times=300)
                    time.sleep(1)
                else:
                    return False, None

        if "red" in result["color"]:    # 对齐次数过多直接抓
            return True, result
        else:
            print(f"{self.arm.local}: 摄像头出问题了")
            return None, None

    def catch_apple(self, num=3, is_show=False):

        result = self.detect_apple(is_show=is_show, choose=self.arm.local)
        print(f"{self.arm.local} catch_result:  {result}")
        if "red" in result["color"]:
            x_distance = result["x_distance"]
            y_distance = result["y_distance"]
            x_move = result["x_move"]
            y_move = result["y_move"]
            deep = result["deep"]
            X, Y, Z, angle = self.arm.get_ordinate()
            # print(f'当前机械臂位置 X:{X}, Y:{Y}, Z:{Z}, angle:{angle}')
            height = Z + self.Z_height

            if X < 0:
                flag_x = -1
            else:
                flag_x = 1
            if Y != 0:
                XY_angle = math.atan2(X, Y)
            else:
                XY_angle = flag_x * math.pi / 2

            catch_Y = int(deep * math.cos(XY_angle) + Y)
            catch_X = int(deep * math.sin(XY_angle) + X)
            if abs(x_move) > 35:
                catch_Y -= x_move

            catch_Z = int(Z - y_move)
            catch_angle = math.atan2(Z - self.l0, math.sqrt(catch_X ** 2 + catch_Y ** 2))
            catch_angle = catch_angle * 180 / pi - 15
            print(f"catch_X: {catch_X}, catch_Y: {catch_Y}, catch_Z: {catch_Z}, catch_angle: {catch_angle}")
            # if self.arm.local == "right":

            if self.arm.local == "right":
                catch_angle = -5
                if catch_X < 400:
                    if abs(x_move) < 3 or abs(y_move) <= 3:
                        catch_X += 25
                    else:
                        catch_X += 10
                    bias = 25
                else:
                    bias = 35
            else:
                if y_move > 30:
                    catch_angle = -7
                elif y_move < -30:
                    catch_angle = 7
                catch_Z -= 10
                bias = 40
            catch_Y -= bias * math.cos(XY_angle)
            catch_X -= bias * math.sin(XY_angle) * flag_x

            print(f"修改后catch_X:{catch_X}, catch_Y:{catch_Y}, catch_Z:{catch_Z}, catch_angle:{catch_angle}")
            if catch_X < 300:
                catch_X = 300

            for i in range(num):    # 多次发送坐标
                status = self.arm.send_ordinate(catch_X, catch_Y, catch_Z, catch_angle, times=600, mode=0, pos_ward=True)
                time.sleep(0.4)
                if status:
                    time.sleep(0.5)
                    if self.arm.local == "right":
                        self.arm.send_pwm(5, 2050, times=700)
                    else:
                        self.arm.send_pwm(5, 1900, times=700)

                    time.sleep(0.8)
                    # 大臂下拉
                    if self.arm.local == "right":
                        self.arm.add_pwm(1, -100, times=1000)
                        time.sleep(1)
                        self.arm.add_pwm(1, 200, times=400)
                    else:
                        self.arm.add_pwm(1, 100, times=1000)
                        time.sleep(1)
                        self.arm.add_pwm(1, -200, times=400)
                    # time.sleep(0.6)
                    # 小臂回收
                    self.arm.add_pwm(2, 250, times=300)
                    time.sleep(1)
                    # if self.arm.local == "left":
                    #     time.sleep(0.3)
                    self.arm.reset_except_lower_and_claw(times=700)
                    time.sleep(0.8)
                    return True
                else:
                    arm_long = self.arm.l1 + self.arm.l2 + self.arm.l3
                    goal_long = math.sqrt(catch_X ** 2 + catch_Y ** 2 + (catch_Z-self.arm.l0) ** 2)
                    if goal_long > arm_long:
                        print(11111111111111111111)
                        if catch_X > 580:
                            arm_long -= 40
                        else:
                            arm_long -= 65
                        catch_Z -= 40
                        catch_Y = arm_long * math.cos(XY_angle)
                        catch_X = arm_long * math.sin(XY_angle)
                        # catch_angle = math.atan2(Z - self.l0, math.sqrt(catch_X ** 2 + catch_Y ** 2))
                    else:
                        bias = 25
                        catch_Y -= bias * math.cos(XY_angle)
                        catch_X -= bias * math.sin(XY_angle) * flag_x

            return False

        elif result["color"] is False:
            print("摄像头出问题了")
            return None

        else:
            return False

