import serial
import threading  # 创建线程、控制线程的执行顺序、进行线程间的通信
import time
import math

Option = {
        "Car_Serial": {"port": "COM4", "baudrate": 115200},  # "/dev/car" 实际上应该是串口设备的具体路径，用于告诉程序连接哪个串口设备。
        "k210_Serial": {"port": "/dev/k210", "baudrate": 115200},
        "Arm_Serial": {"port": "/dev/arm", "baudrate": 115200},
    }  # 串口配置
class Car_Bridge:
    def __init__(self):  # 初始化
        self.car_ser: serial.Serial
        self.Yaw = 0
        self.is_close = False
        self.count = 0
        self.is_move = None

    def init(self, Option) -> bool:  # Option 是一个字典，用于传递配置选项。
        self.Option = Option

        try:
            self.car_ser = serial.Serial(
                Option["Car_Serial"]["port"], Option["Car_Serial"]["baudrate"]  # 从option参数中获取串口信息
            )
            if self.car_ser.is_open == False:
                self.car_ser.open()  # 打开串口
                self.is_close = False
            print("成功打开底盘串口")
            self.car_ser.flushInput()  # type: ignore # 清空串口缓冲区
            return True  # 表示初始化成功
        except Exception as e:
            print("底盘串口打开失败:", e)
            return False  # 初始化串口过程中发生异常，捕获异常并打印错误信息

    def send_Command(self, arr, is_print=False):  # arr为发送的字符串 is_print 是一个布尔型参数，默认为 False
        try:
            self.car_ser.write(arr.encode("ascii"))  # 将 arr 字符串编码为 ASCII 格式并发送到底盘。
            if is_print:
                print(f"向底盘发送指令：{arr}")
        except Exception as e:
            print("向底盘发送指令失败:", e)

    def Receiver(self): # 上位机接收指令
        try:
            if self.car_ser.in_waiting > 0:  # 检查是否有数据可读
                message = self.car_ser.readline()
                if message:
                    message = message.decode(encoding="UTF-8")
                    message = message.strip()  # 去掉首尾空白符
                    # print(message)
                    datas = message.split("|")  # 用竖线分隔信息
                    if datas[0] == "$":
                        if datas[1] == "1":
                            self.is_move = True
                        elif datas[1] == "0":
                            self.is_move = False
                    if datas[0] == "#":
                        self.Yaw = -float(datas[1])

        except Exception as e:  # 捕获异常并处理
            if self.count < 10:
                self.count += 1
                print("Car_error %s" % e)  # %s 字符串 (采用str()的显示)

    def Car_Receiver(self):  # 接受self参数
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
        car_receive_thread = threading.Thread(target=self.Car_Receiver)
        car_receive_thread.start()
        # self.get_cur_Yaw()

    def reboot(self):
        self.init(self.Option)
        self.run_thread()

    def get_cur_Yaw(self, is_print=False):
        arr = "@|5|0|0|0|#"
        self.send_Command(arr=arr, is_print=is_print)
        time.sleep(0.2)
        return self.Yaw

    def go_distance(self, dis, speed=0.2, is_print=False):
        if abs(dis) < 0.5:
            speed = 0.1
        elif abs(dis) > 1:
            speed = 0.3
        self.move(dis, 0, speed=speed, is_print=is_print)

    def yaw_adjustment(self, yaw, is_print=False):
        cur_yaw = self.get_cur_Yaw()
        time.sleep(0.2)
        # print(cur_yaw)
        arrive_angle = round(cur_yaw + yaw, 2)  # 将当前偏航角度 cur_yaw 和传入的 yaw 相加，并且四舍五入保留两位小数
        arr = f"@|2|0|{arrive_angle}|0|#"
        self.send_Command(arr=arr, is_print=is_print)
        while True:
            if self.is_move:
                self.is_move = None
                break
            if self.is_move is False:
                self.send_Command(arr=arr, is_print=is_print)
            time.sleep(0.01)

    def move(self, x_dis, y_dis, speed=0.1, error=0.01, is_print=False):
        x_dis = -int(x_dis * 1000) / 1000  # 这两行代码将传入的 x_dis 和 y_dis 分别乘以1000后取整，再除以1000，这样做是为了确保精度到小数点后三位。负号表示对坐标进行反向调整
        y_dis = -int(y_dis * 1000) / 1000

        if 0.005 < abs(x_dis) < error and x_dis != 0:
            x_dis = x_dis / abs(x_dis) * error
        if 0.005 < abs(y_dis) < error and y_dis != 0:
            y_dis = y_dis / abs(y_dis) * error
        arr = f"@|6|{speed}|{x_dis}|{y_dis}|#"
        self.send_Command(arr=arr, is_print=is_print)
        while True:
            if self.is_move:
                self.is_move = None
                break
            elif self.is_move is False:
                self.send_Command(arr=arr, is_print=is_print)
            time.sleep(0.1)

    def go_sideway(self, dis, speed=0.1, is_print=False):
        self.move(0, dis, speed=speed, is_print=is_print)

    def close(self):
        self.car_ser.close()
        self.is_close = True
        print("关闭car串口")


def perform_action2(angle_degrees, known_side, A):
    # 将角度转换为弧度
    angle_radians = math.radians(angle_degrees)

    # 计算另一条直角边的长度
    other_side = known_side / math.tan(angle_radians)
    # 计算斜边的长度
    hypotenuse = known_side / math.sin(angle_radians)

    # 四舍五入到小数点后四位
    other_side_rounded = round(other_side, 3)
    hypotenuse_rounded = round(hypotenuse, 3)

    print(known_side)
    print(angle_radians)
    print(hypotenuse_rounded)

    # 根据已知边的正负来调整小车动作
    if A == 1:
        car.yaw_adjustment(angle_degrees)
        car.go_distance(abs(hypotenuse_rounded), 0.1)
        car.yaw_adjustment(0)
       # car.go_distance(-(abs(other_side_rounded) + 0.001) if abs(other_side_rounded) > 5 else -abs(other_side_rounded), 0.1)
    else:
        car.yaw_adjustment(angle_degrees)
        car.go_distance(-abs(hypotenuse_rounded), 0.1)
        car.yaw_adjustment(0)
      # car.go_distance(abs(other_side_rounded) + 0.001 if abs(other_side_rounded) > 5 else abs(other_side_rounded), 0.1)

if __name__ == "__main__":
    car = Car_Bridge()
    car.init(Option)
    car.run_thread()
    perform_action2(20, 0.1,1)

    # car.go_distance(0.05, 0.1)





