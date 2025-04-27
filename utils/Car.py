import serial
import time
import math
import numpy as np
from utils.Run import *

class Car_Bridge:
    def __init__(self, initial_direction="up"):
        self.Yaw = 0
        self.is_close = False
        self.count = 0
        self.is_move = None
        self.create_map(rows=400, cols=450, local=(360, 50))
        self.initial_direction = initial_direction
        self.car_ser = None
        self.Option = None
        self.map = None

    def init(self, Option) -> bool:
        self.Option = Option

        try:
            self.car_ser = serial.Serial(
                Option["port"], Option["baud_rate"]
            )
            if self.car_ser.is_open == False:
                self.car_ser.open()  # 打开串口
                self.is_close = False
            print("成功打开底盘串口")
            self.car_ser.flushInput()  # type: ignore
            return True
        except Exception as e:
            print("底盘串口打开失败:", e)
            return False
    """
    创建地图，地图上只有一个点，表示小车的位置
    没卵用的，与其相关的函数不用在意
    """
    def create_map(self, rows, cols, local: tuple):
        self.rows = rows
        self.cols = cols
        self.map = np.zeros((rows + 1, cols + 1))
        self.map[local] = 1

    def change_position(self, position):
        self.map = np.where(self.map == 1, 0, 0)
        self.map[position] = 1

    def get_position(self):
        position = np.where(self.map == 1)
        position = (position[0][0], position[1][0])
        return position

    def move_(self, x, y):      # 改变车在地图上的坐标
        x = x * 100
        y = y * 100
        cur_Yaw = self.get_cur_Yaw()
        position = self.get_position()
        angle = cur_Yaw * math.pi / 180
        if y == 0:
            move_angle = 0 if x > 0 else math.pi
        else:
            move_angle = x / y
        dis = math.sqrt(x * x + y * y)
        world_angle = angle + move_angle
        move_x = int(dis * math.cos(world_angle))
        move_y = int(dis * math.sin(world_angle))
        if self.initial_direction == "up":
            new_position = (position[0] - move_x, position[1] + move_y)
        elif self.initial_direction == "down":
            new_position = (position[0] + move_x, position[1] - move_y)
        elif self.initial_direction == "left":
            new_position = (position[0] + move_y, position[1] + move_x)
        else:
            new_position = (position[0] - move_y, position[1] - move_x)
        if new_position[0] <= self.rows and new_position[0] <= self.cols:
            self.change_position(new_position)
        # else:
        #     print("out of map", color="red", is_choose_color=True)

    def send_Command(self, arr, is_print=False):
        try:
            self.car_ser.write(arr.encode("ascii"))
            if is_print:
                print(f"向底盘发送指令：{arr}")
        except Exception as e:
            print("向底盘发送指令失败:", e)

    def Receiver(self):
        try:
            if self.car_ser.in_waiting > 0:
                message = self.car_ser.readline()
                if message:
                    message = message.decode(encoding="UTF-8")
                    message = message.strip()
                    datas = message.split("|")
                    if datas[0] == "$":
                        if datas[1] == "1":
                            self.is_move = True
                        elif datas[1] == "0":
                            self.is_move = False
                    if datas[0] == "#":
                        self.Yaw = float(datas[1])

        except Exception as e:
            if self.count < 10:
                self.count += 1
                print("Car_error %s" % e)

    def Car_Receiver(self):
        while True:
            if self.count < 15:
                self.Receiver()
                # if Car_Yaw is not None:
                #     self.Yaw = Car_Yaw
                if self.is_close:
                    break
            else:
                self.count = 0
                self.is_close = True
                break

    def run_thread(self):
        car_receive_thread = threading.Thread(target=self.Car_Receiver, daemon=True)
        car_receive_thread.start()
        self.get_cur_Yaw()

    def reboot(self):
        self.init(self.Option)
        self.run_thread()

    def get_cur_Yaw(self, is_print=False):
        """
        获取当前角度
        :param is_print:
        :return:
        """
        arr = "@|5|0|0|0|#"
        self.send_Command(arr=arr, is_print=is_print)
        time.sleep(0.1)
        return self.Yaw

    def go_distance(self, dis, speed=0.2, is_print=False):
        if abs(dis) < 0.5:
            speed = 0.1
        elif abs(dis) > 1:
            speed = 0.3

        is_success = self.move(dis, 0, speed=speed, is_print=is_print)
        if is_success:
            # color_print_("  目前 position: " + str(self.get_position()))
            pass
        else:
            print("False")
        time.sleep(0.2)
        return is_success

    def yaw_adjustment(self, yaw, is_print=False):
        cur_yaw = self.get_cur_Yaw()
        time.sleep(0.1)
        # print(cur_yaw)
        arrive_angle = int(cur_yaw + yaw)
        arr = f"@|2|0|{arrive_angle}|0|#"
        if arrive_angle > 180:
            arrive_angle = arrive_angle - 360
        if arrive_angle < -180:
            arrive_angle = arrive_angle + 360
        self.Yaw = arrive_angle
        self.send_Command(arr=arr, is_print=is_print)
        start_time = time.time()
        need_time = abs(yaw) * 0.2
        while True:
            end_time = time.time()
            if (end_time - start_time) > need_time + 3:
                return False
            if self.is_move:
                self.is_move = None
                time.sleep(0.1)
                return True
            if self.is_move is False:
                self.send_Command(arr=arr, is_print=is_print)
            time.sleep(0.01)

    def move(self, x_dis, y_dis, speed=0.1, error=0.01, is_print=False):
        # 小车以屁股朝前
        x_dis = -int(x_dis * 1000) / 1000
        y_dis = -int(y_dis * 1000) / 1000

        if 0.005 < abs(x_dis) < error and x_dis != 0:
            x_dis = x_dis / abs(x_dis) * error
        if 0.005 < abs(y_dis) < error and y_dis != 0:
            y_dis = y_dis / abs(y_dis) * error

        # self.move_(x_dis, y_dis)
        arr = f"@|6|{speed}|{x_dis}|{y_dis}|#"
        self.send_Command(arr=arr, is_print=is_print)

        start_time = time.time()
        need_time = abs(x_dis) / 0.15
        while True:
            end_time = time.time()
            if (end_time - start_time) > need_time + 5:
                return False
            if self.is_move:
                self.is_move = None
                return True
            if self.is_move is False:
                self.send_Command(arr=arr, is_print=is_print)
            time.sleep(0.05)

    def go_sideway(self, angle_degrees, known_side, is_forward=True):
        """
        侧向移动， 不是我写的，感觉不好用
        :param angle_degrees: 要侧移时旋转的角度
        :param known_side: 平移量
        :param is_forward: 是否前进
        :return:
        """
        cur_yaw = int(self.get_cur_Yaw())
        # 将角度转换为弧度
        angle_radians = math.radians(angle_degrees)

        # 计算另一条直角边的长度
        # other_side = known_side / math.tan(angle_radians)
        # 计算斜边的长度
        hypotenuse = known_side / math.sin(angle_radians)

        # 四舍五入到小数点后四位
        # other_side_rounded = round(other_side, 3)
        hypotenuse_rounded = round(hypotenuse, 3)

        # 根据已知边的正负来调整小车动作
        if is_forward:
            self.yaw_adjustment(angle_degrees)
            self.go_distance(abs(hypotenuse_rounded), 0.1)
            time.sleep(0.1)
            self.yaw_adjustment(cur_yaw)
        else:
            self.yaw_adjustment(angle_degrees)
            self.go_distance(-abs(hypotenuse_rounded), 0.1)
            time.sleep(0.1)
            self.yaw_adjustment(cur_yaw)

        return True

    def sideway(self, distance, side):
        """
        侧向移动，distance为前进的距离，side为平移的距离，正数为右移，负数为左移
        不要两个参数同时为负，否则车会旋转180，再前进
        因为没用到，懒得管了
        :param distance: 前进的距离
        :param side: 平移距离
        :return:
        """
        cur_yaw = int(self.get_cur_Yaw())
        distance = int(distance * 1000) / 1000
        side = int(side * 1000) / 1000
        rotate_angle = math.atan2(side, distance) * 180 / math.pi
        hypotenuse = math.sqrt(distance ** 2 + side ** 2)

        self.yaw_adjustment(rotate_angle)
        time.sleep(0.2)
        self.go_distance(hypotenuse)
        time.sleep(0.2)
        self.yaw_adjustment(-rotate_angle)
        return True

    def close(self):
        self.is_close = True
        self.car_ser.close()

    def stop(self, is_print=False):
        print("s 停下小车运动")
        arr = "@|66|0|0|0|#"
        self.send_Command(arr=arr, is_print=is_print)
        time.sleep(0.1)
        print("s 已停下")

    def go_distance_control(self, speed=0.2, is_print=False):
        print('-- 持续直行 ')
        # speed为正则前进，为负则后退
        arr = f"@|66|{speed}|0|0|#"
        self.send_Command(arr=arr, is_print=is_print)
        time.sleep(0.1)

    def yaw_adjustment_control(self, speed=1, is_print=False):
        print(f'() 持续转角')
        # speed为角速度；正则为逆时针，负则为顺时针
        arr = f"@|66|0|0|{speed}|#"
        self.send_Command(arr=arr, is_print=is_print)
        time.sleep(0.1)


a = "12@"
if "@" in a:
    print(1)