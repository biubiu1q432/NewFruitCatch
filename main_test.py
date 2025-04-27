import os
import cv2
import time
import math
import serial
import colorama
import threading
import numpy as np
import multiprocessing
from utils.Camera import *
from utils.Run import *
# from utils.Arm_Control import *
from utils.Car import *
from utils.Cross_Detect import *
from utils.Arm_Control import Arm
import rclpy
import contextlib
import builtins
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from concurrent.futures import ThreadPoolExecutor, as_completed
from playsound import playsound

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

        results = self.camera.detect(mode=None, is_show=is_show)
        x_move = None
        result = results[0]
        if "red" in result["color"] or "green" in result["color"]:
            x_move = result["x_move"]
        return x_move

    def aim_vegetable(self, is_show=False):
        mode = "vegetable"
        for i in range(3):
            self.camera.get_frame()
            results = self.camera.detect(mode=mode, is_show=is_show)
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

                    if abs(x_distance) > 45 + i * 5 or abs(
                            y_distance) > 45 + i * 5:  # 中心点的阈值，离物体越近，阈值愈大

                        if self.arm.local == "left":
                            move_X = int(X + x_move)
                        else:
                            move_X = int(X - x_move)

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
            results = self.camera.detect(mode=mode, is_show=is_show)
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
                            if abs(need_rotate_angle) > 4 and i <= 2:
                                self.arm.rotate(need_rotate_angle, times=200)
                                time.sleep(0.4)
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
                            status = self.arm.up(-y_move, times=500, forward_num=20)
                            if not status:
                                self.arm.up(-y_move, times=500, need_forward=True, forward_num=30)

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

    def catch_vegetable(self, result, num=3):
        # result = self.camera.detect(num=5)
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
                        bias = height / math.tan(abs(angle-17)*math.pi/180) + 40
                        catch_Y = int(Y) + bias * math.cos(XY_angle)
                        catch_X = int(X) + bias * math.sin(XY_angle) * flag_x

                    else:
                        catch_Y = int(Y) - y_move + 80
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
                            self.arm.send_pwm(5, 1900, times=700)
                        time.sleep(0.6)
                        self.arm.add_pwm(3, -200, times=500)
                        time.sleep(0.2)

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

    def catch_fruit(self, result, num=3):
        # result = self.camera.detect(num=5)
        print(f"catch_result:  {result}")
        if result["color"] is not None:
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
                catch_Z = int(Z + y_move)
                catch_angle = math.atan2(Z - self.l0, math.sqrt(catch_X ** 2 + catch_Y ** 2))
                catch_angle = catch_angle * 180 / pi - 15
                print(f"catch_X: {catch_X}, catch_Y: {catch_Y}, catch_Z: {catch_Z}, catch_angle: {catch_angle}")
                # if self.arm.local == "right":

                if self.arm.local == "right":
                    if catch_Y < 400:
                        if abs(x_move) < 3 or abs(y_move) <= 3:
                            catch_Y += 25
                        else:
                            catch_Y += 10
                        bias = 55
                    else:
                        bias = 65
                else:
                    bias = 75
                catch_Y -= bias * math.cos(XY_angle)
                catch_X -= bias * math.sin(XY_angle) * flag_x
                catch_Z += 10

                print(f"修改后catch_X:{catch_X}, catch_Y:{catch_Y}, catch_Z:{catch_Z}, catch_angle:{catch_angle}")
                if catch_Y < 290:
                    catch_Y = 290

                for i in range(num):    # 多次发送坐标
                    status = self.arm.send_ordinate(catch_X, catch_Y, catch_Z, catch_angle, times=600, mode=0, pos_ward=True)
                    if status:
                        time.sleep(0.55)
                        if self.arm.local == "right":
                            self.arm.send_pwm(5, 2100, times=700)
                        else:
                            self.arm.send_pwm(5, 1900, times=700)
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
                                arm_long -= 40
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

    def detect_apple(self, choose_one=0, is_show=False, is_reverse=False):  # next_one取第二靠边的或者
        frame, results = self.camera.model_detect(conf=0.8, is_show=False, limit_area=8000)

        red_apple_list = []
        for result in results:
            if "red" in result["color"]:
                red_apple_list.append(result)

        if red_apple_list:
            if self.arm.local == "left":
                red_apple_list.sort(key=lambda x: x["cx"])  # 按照中心点排序，最靠左的在第一个
            else:
                red_apple_list.sort(key=lambda x: x["cx"], reverse=True)

            if not is_reverse:
                result = red_apple_list[-1]
            else:
                result = red_apple_list[choose_one]

            cx = result["cx"]
            cy = result["cy"]
            w = result["w"]
            h = result["h"]
            x1 = cx - 0.5 * w
            y1 = cy - 0.5 * h
            cv2.putText(frame,
                            'goal_apple', (int(x1), int(y1 - 2)),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.60, (0, 255, 255),
                            thickness=1)
            if is_show:
                cv2.imshow("goal_apple_image", frame)
                cv2.waitKey(0)
            return result
        else:
            return {"color": "None"}

    def aim_apple(self, is_show=False, mode=None):
        for i in range(3):      # 多次尝试
            # if is_detect:
            self.camera.get_frame()     # 避免摄像头延时
            if mode is not None:
                if mode == "no next" or "all":
                    result = self.detect_apple(is_show=is_show, is_reverse=True)
                else:
                    result = self.detect_apple(is_show=is_show)
            else:
                result = self.detect_apple(is_show=is_show)
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

                    if self.arm.local == "right":
                        if abs(need_rotate_angle) > 3 and i <= 2:

                            self.arm.rotate(need_rotate_angle, times=200)
                            time.sleep(0.4)
                    else:
                        if abs(need_rotate_angle) > 4 and i <= 2:
                            self.arm.rotate(need_rotate_angle, times=200, mode="ordinate")  # 左机械臂莫名奇妙10度内
                            time.sleep(0.4)
                        else:
                            bias = 30
                            now_angle = abs(now_angle)
                            add_x = bias * math.cos(now_angle * math.pi / 180) * (-1 if x_distance > 0 else 1)
                            add_y = bias * math.sin(now_angle * math.pi / 180)
                            move_z = int(Z - y_move / 4)

                            self.arm.send_ordinate(X - add_x, Y + add_y, move_z, angle, times=300, mode=0)

                    if abs(y_move) > 20:
                        status = self.arm.up(-y_move, times=500)
                        if not status:
                            self.arm.up(-y_move, times=500, need_forward=True, forward_num=20)

                    time.sleep(0.4)

                else:
                    print("水果成功对齐")
                    return True, result

            elif result["color"] is False:
                print("摄像头出现问题，请检查摄像头")

            else:   # 代表无成熟的情况
                return False, None


        if "red" in result["color"]:    # 对齐次数过多直接抓
            return True, result
        else:
            print(f"{self.arm.local}: 摄像头出问题了")
            return None, None

    def catch_apple(self, num=3, is_show=False, mode=None):
        if mode is not None:
            if mode == "no next":
                result = self.detect_apple(is_show=is_show)
            else:
                result = self.detect_apple(is_show=is_show)
        else:
            result = self.detect_apple(is_show=is_show)
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
            catch_Z = int(Z + y_move)
            catch_angle = math.atan2(Z - self.l0, math.sqrt(catch_X ** 2 + catch_Y ** 2))
            # catch_angle = catch_angle * 180 / pi - 10
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
                catch_angle = 15
                bias = 40
            catch_Y += 10
            catch_Y -= bias * math.cos(XY_angle)
            catch_X -= bias * math.sin(XY_angle) * flag_x
            catch_Z += 10

            print(f"修改后catch_X:{catch_X}, catch_Y:{catch_Y}, catch_Z:{catch_Z}, catch_angle:{catch_angle}")
            if catch_X < 290:
                catch_X = 290

            if catch_X > 580:
                return None

            for i in range(num):    # 多次发送坐标
                status = self.arm.send_ordinate(catch_X, catch_Y, catch_Z, catch_angle, times=600, mode=0, pos_ward=True)
                time.sleep(0.4)
                if status:
                    time.sleep(0.25)
                    if self.arm.local == "right":
                        self.arm.send_pwm(5, 2100, times=700)
                    else:
                        self.arm.send_pwm(5, 1900, times=700)

                    time.sleep(0.7)
                    # 大臂下拉
                    if self.arm.local == "right":
                        self.arm.add_pwm(1, 250, times=600)
                    else:
                        self.arm.add_pwm(1, -250, times=600)
                    time.sleep(0.6)
                    # 小臂回收
                    # self.arm.add_pwm(2, 100, times=600)
                    # time.sleep(0.3)
                    # if self.arm.local == "left":
                    #     time.sleep(0.3)
                    self.arm.reset_except_lower_and_claw(times=1000)
                    return True

            return False

        elif result["color"] is False:
            print("摄像头出问题了")
            return None

        else:
            return False


px = 0.0014  # 像素点大小
f = 1.8  # 摄像头焦距
camera_to_claw = 60  # 爪子离相机距离
height = 115  # 云台离地高度

class ThreadPool:
    def __init__(self):
        self.pool = ThreadPoolExecutor(max_workers=5)
        self.tasks = {}
        self.create_group(["catch", "aim", "put_down", "socket", "detect"])

    def create_group(self, groups):
        if isinstance(groups, str):
            self.tasks[groups] = {}
        elif isinstance(groups, list):
            for group in groups:
                self.tasks[group] = {}
        else:
            raise ValueError("groups must be a string or a list")

    def add_task(self, func, *args, **kwargs):
        if "info" in kwargs:
            info = kwargs.pop("info")
        else:
            info = None
        if "group" in kwargs:
            group = kwargs.pop("group")
        else:
            group = None
        future = self.pool.submit(func, *args, **kwargs)

        if info is not None and group is not None:
            self.tasks[group][info] = future

        elif info is not None:
            self.tasks[info] = future

    def remove_task(self, info, group):
        if group in self.tasks:
            if info in self.tasks[group]:
                del self.tasks[group][info]

    def wait(self, info, group):
        # print(self.tasks)
        if group in self.tasks:
            if info in self.tasks[group]:
                result = self.tasks[group][info].result()
                self.remove_task(info, group)
                return result
        return None

    def wait_completion(self):
        self.pool.shutdown(wait=True)

pool = ThreadPool()

def detect_and_aim_threading(catcher: Catcher, mode="fruit", is_show=False):
    time.sleep(0.1)
    if mode == "fruit":
        status, result = catcher.aim_fruit(is_show=is_show)
    else:
        status, result = catcher.aim_vegetable(is_show=is_show)

    if status:  # 对齐成功
        time.sleep(0.1)
        if mode == "fruit":
            catch_status = catcher.catch_fruit(result)
        else:
            catch_status = catcher.catch_vegetable(result)

        if catch_status:  # 抓取成功
            # threading.Thread(target=catcher.arm.put_down, args=(), name=catcher.arm.local).start()
            # catcher.arm.put_down()
            return True
        else:  # 抓取失败
            catcher.arm.reset()
            return False

    elif status is False:   # 没有成熟水果的情况
        catcher.arm.reset()
        return False
    else:   # 对齐失败的情况
        return None



def class_init(Option):
    Img_Receiver = Image_Receiver(name=Option["video"]["node_name"], topic=Option["video"]["img_topic"])
    executor = rclpy.executors.SingleThreadedExecutor()
    camera = Camera(Option["video"]["video_ser"], img_receiver=Img_Receiver, executor=executor, system_type="Linux")
    arm = Arm(local=Option["local"])
    arm.init(Option["arm_ser"])
    arm.run_thread()
    catcher = Catcher(arm, camera)

    return catcher, camera, arm

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

rclpy.init()
Option1 = Option["left"]
Img_Receiver_left = Image_Receiver(name=Option1["video"]["node_name"], topic=Option1["video"]["img_topic"])
executor_left = rclpy.executors.SingleThreadedExecutor()
Option2 = Option["right"]
Img_Receiver_right = Image_Receiver(name=Option2["video"]["node_name"], topic=Option2["video"]["img_topic"])
executor_right = rclpy.executors.SingleThreadedExecutor()
camera_left = Camera(Option1["video"]["video_ser"], img_receiver=Img_Receiver_left, executor=executor_left, system_type="Linux")
camera_right = Camera(Option2["video"]["video_ser"], img_receiver=Img_Receiver_right, executor=executor_right, system_type="Linux")
arm_left = Arm(local=Option1["local"])
arm_left.init(Option1["arm_ser"])
arm_left.run_thread()
arm_right = Arm(local=Option2["local"])
arm_right.init(Option2["arm_ser"])
arm_right.run_thread()
catcher_left = Catcher(arm_left, camera_left)
catcher_right = Catcher(arm_right, camera_right)
car = Car_Bridge()
car.init(Option["Car_Serial"])
car.run_thread()
cross_detect = Cross_Detect()

def detect_cross(mode="left", is_show=False):
    point_list = []

    for i in range(4):
        if mode == "right":
            frame, point = cross_detect.get_cross_point(camera_right.get_frame(), is_show=is_show)
        else:
            frame, point = cross_detect.get_cross_point(camera_left.get_frame(), is_show=is_show)
        if point is not None:
            point_list.append(point)
    print(point_list)
    if len(point_list) > 0:
        point = np.mean(point_list, axis=0)
        if mode == "right":
            for i in range(3):
                _, _, Z, _ = catcher_right.arm.get_ordinate()
        else:
            for i in range(3):
                _, _, Z, _ = catcher_left.arm.get_ordinate()
        camera_height = Z + height  # 摄像头离地高度
        print(f"point: {point}")
        x_dis = 640 / 2 - point[0]
        y_dis = 480 / 2 - point[1]
        xy_dis = math.sqrt((x_dis * px) ** 2 + (y_dis * px) ** 2)
        cos = x_dis * px / xy_dis
        sin = y_dis * px / xy_dis
        dis = xy_dis * camera_height / f
        x_move = dis * sin * 2.2
        y_move = dis * cos * 1.9
        print(f"x_move: {x_move}, y_move: {y_move}")

        return True, x_move, y_move

    else:
        return False, None, None


def detect_cross_and_run(count=0, bias=0.03, goal_x=0, goal_y=0, clockwise_rotate=True, mode="left"):
    status, x_move, y_move = detect_cross(mode=mode)
    right_arm_to_mid_dis = 90
    left_arm_to_mid_dis = 90
    arm_to_car_mid_dis = 100
    x_bias = -50
    y_bias = -20
    rotate_x_bias = -70
    rotate_y_bias = -50
    if mode == "left":
        arm = arm_left
    else:
        arm = arm_right
    if status:

        x, y, z, angle = arm.get_ordinate()

        if mode == "left":
            x_dis = x_move + x + arm_to_car_mid_dis + goal_x + rotate_x_bias
            y_dis = y_move + left_arm_to_mid_dis + goal_y + rotate_y_bias
            final_x = x_dis / 1000
            final_y = y_dis / 1000
        else:
            x_dis = x_move + x + arm_to_car_mid_dis + goal_x+ rotate_x_bias
            y_dis = y_move - right_arm_to_mid_dis + goal_y+ rotate_y_bias
            final_x = x_dis / 1000
            final_y = -y_dis / 1000
        print(f"x_move(摄像头离十字标距离): {x_move}, y_move: {y_move}, y_dis: {y_dis}, x_dis: {x_dis}")
        print(f"当前车离十字标距离：{x_move + x + arm_to_car_mid_dis}")
        print(f"final_x: {final_x}, final_y: {final_y}")
        arm.reset()

        car.go_distance(final_x+bias)
        time.sleep(0.5)
        if clockwise_rotate:
            car.yaw_adjustment(90)
        else:
            car.yaw_adjustment(-90)
            # final_y = -final_y
        time.sleep(1)
        car.go_distance(int(final_y))
        return True
    else:
        if count > 4:
            os.system("v4l2-ctl --device=/dev/video_right --set-ctrl=exposure_time_absolute=200")
            if mode == "right":
                arm_right.reset()
            else:
                arm_left.reset()
            if clockwise_rotate:
                car.yaw_adjustment(90)
            else:
                car.yaw_adjustment(-90)
            car.go_distance(0.2)
            print("未检测到交叉点")
            return True

        car.go_distance(0.1)
        time.sleep(1)
        result = detect_cross_and_run(count + 1, goal_x=goal_x, goal_y=goal_y, clockwise_rotate=clockwise_rotate)
        return result

def pre_detect_apple(camera: Camera, is_show=False):
    """

    :param camera:
    :return: 返回值的使用：先判断红苹果数，有的话后面的值才有意义，middle意味着两边可同时抓，None表示有多个需要抓，且不能同时
    """
    frame, results = camera.model_detect(conf=0.7, is_show=is_show, limit_area=5500)
    results.sort(key=lambda x: x["cx"])
    red_apple_num = 0
    green_apple_num = 0
    red_apple_index_list = []
    green_apple_index_list = []
    print(results)

    if results[0]["color"] == None:     # 没有东西的情况，正常不会发生
        return True, red_apple_num, None

    for i, result in enumerate(results):
        if result["cx"] < 30 or result["cx"] > 620:     # 过于靠边抛弃
            continue
        if "red" in result["color"]:
            red_apple_num += 1
            red_apple_index_list.append(i)
        elif "green" in result["color"]:
            green_apple_num += 1
            green_apple_index_list.append(i)

    if red_apple_num == 0:
        return True, red_apple_num, None
    elif red_apple_num == 1:
        if red_apple_index_list[0] == 0 or red_apple_index_list[0] == 1:
            return True, red_apple_num, "left"
        else:
            return True, red_apple_num, "right"
    elif red_apple_num == 2:
        if red_apple_index_list[0] + 1 == red_apple_index_list[1]:  # 两个红色苹果紧挨
            if green_apple_index_list:
                if green_apple_index_list[0] == 0:  # 绿色最左
                    return True, red_apple_num, "right next"
                else:
                    return True, red_apple_num, "left next"
            else:
                return True, red_apple_num, "left next"
        else:
            return True, red_apple_num, "no next"
    elif red_apple_num == 3:
        return True, red_apple_num, "all"

    # if red_apple_num > 0:
    #     if len(results) == 3:   # 正常情况
    #         if red_apple_num == 1:
    #             if red_apple_index_list[0] == 0 or red_apple_index_list[0] == 1:
    #                 return True, red_apple_num, "left"
    #             else:
    #                 return True, red_apple_num, "right"
    #         elif red_apple_num == 2:
    #             if green_apple_index_list[0] == 1:  # 绿色在中间，两边可同时抓
    #                 return True, red_apple_num,  "next"
    #             else:   # 两个红色苹果紧挨
    #                 return True, red_apple_num, "beside"
    #         else:
    #             return True, red_apple_num, "all"
    #     # 一般不会进入以下判断
    #     elif len(results) > 3:   # 多识别了东西
    #         print("多识别到东西")
    #         if red_apple_num > 1:
    #             return True, red_apple_num, None
    #         else:   # 只有一个红色
    #             if red_apple_index_list[0] <= 1:
    #                 return True, red_apple_num, "left"
    #             else:
    #                 return True, red_apple_num, "right"
    #
    #     else:
    #         print("少识别东西")
    #         if red_apple_num > 1:
    #             return True, red_apple_num, None
    #         else:   # 只有一个红色
    #             if red_apple_index_list[0] < 1:
    #                 return True, red_apple_num, "left"
    #             else:
    #                 return True, red_apple_num, "right"
    #
    # else:
    #     return True, red_apple_num, None

def prepare_aim(red_apple_num, mode):
    # arm_left.send_pwm(5, 1700)
    # time.sleep(0.5)
    # arm_left.reset()
    # arm_right.reset()
    # time.sleep(1)
    if red_apple_num == 0:
        return True

    elif red_apple_num == 1:
        if mode == "left":
            arm_left.send_pwm(0, 800, times=300)
            time.sleep(0.5)
            pool.add_task(arm_left.start_aim, mode="apple", info="left_thread", group="aim")
        else:
            arm_right.send_pwm(0, 2000, times=300)
            time.sleep(0.5)
            pool.add_task(arm_right.start_aim, mode="apple", info="right_thread", group="aim")

    elif red_apple_num == 2 or red_apple_num == 3:

        time.sleep(0.5)
        if mode == "no next" or mode == "all":
            arm_right.send_pwm(0, 2000, times=300)
            arm_left.send_pwm(0, 700, times=300)
            pool.add_task(arm_left.start_aim, mode="apple", info="left_thread", group="aim")
            pool.add_task(arm_right.start_aim, mode="apple", info="right_thread", group="aim")
            pool.wait("right_thread", group="aim")
            pool.wait("left_thread", group="aim")
        elif mode == "left next":
            arm_left.send_pwm(0, 700, times=300)
            pool.add_task(arm_left.start_aim, mode="apple", info="left_thread", group="aim")
            pool.wait("left_thread", group="aim")
        elif mode == "right next":
            arm_right.send_pwm(0, 2000, times=300)
            pool.add_task(arm_right.start_aim, mode="apple", info="right_thread", group="aim")
            pool.wait("right_thread", group="aim")

    time.sleep(0.2)

    return True

def detect_and_aim_apple_threading(catcher: Catcher, is_show=False, mode=None):
    time.sleep(0.1)
    status, result = catcher.aim_apple(is_show=is_show, mode=mode)

    if status:  # 对齐成功
        time.sleep(0.1)
        catch_status = catcher.catch_apple()

        if catch_status:  # 抓取成功
            # threading.Thread(target=catcher.arm.put_down, args=(), name=catcher.arm.local).start()
            # catcher.arm.put_down()
            return True
        else:  # 抓取失败
            catcher.arm.reset_except_lower()
            return False

    elif status is False:   # 没有成熟水果的情况
        catcher.arm.reset_except_lower()
        return False
    else:   # 对齐失败的情况
        return None

def catch_apple_all_action(red_apple_num, mode):
    right_is_put = False
    left_is_put = False
    if red_apple_num > 0:
        if mode == "no next" or mode == "all":    # (青苹果在中间或是全是红色，两边瞄准后同时抓）
            pool.add_task(catcher_right.aim_apple, is_show=False, mode=mode, info='Right Arm', group="catch")
            pool.add_task(catcher_left.aim_apple, is_show=False, mode=mode, info='Left Arm', group="catch")
            # right_status, right_result = pool.wait(info="Right Arm", group="catch")
            # left_status, left_status = pool.wait(info="Left Arm", group="catch")

            for i in range(2):
                for future in as_completed(pool.tasks["catch"].values()):
                    info = [key for key, value in pool.tasks["catch"].items() if value == future][0]  # 得到当前完成线程的名字
                    result = future.result()  # 获得线程函数完成的返回值
                    if isinstance(result, tuple):
                        status = result[0]
                    else:
                        status = result
                    if status:
                        if info == "Left Arm":
                            pool.remove_task(info, group="catch")
                            pool.add_task(catcher_left.catch_apple, mode=mode, info="Left Arm", group="catch")
                            left_is_put = True
                        elif info == "Right Arm":
                            pool.remove_task(info, group="catch")
                            pool.add_task(catcher_right.catch_apple, mode=mode, info="Right Arm", group="catch")
                            right_is_put = True
                    elif result is None:
                        if i == 0:
                            if info == 'Right Arm':
                                pool.add_task(catcher_right.aim_apple, is_show=False, mode=mode, info='Right Arm', group="catch")
                            elif info == 'Left Arm':
                                pool.add_task(catcher_left.aim_apple, is_show=False, mode=mode, info='Left Arm', group="catch")
                        else:
                            if info == 'Right Arm':
                                arm_right.reset()
                            elif info == 'Left Arm':
                                arm_left.reset()

            # if right_status and left_status:    # 正常情况 （两边对准完毕）
            #     pool.add_task(catcher_left.catch_apple, mode=mode, info="Left Arm", group="catch")
            #     pool.add_task(catcher_right.catch_apple, mode=mode, info="Right Arm", group="catch")
            #
            #     left_status = pool.wait(info="Left Arm", group="catch")
            #     right_status = pool.wait(info="Right Arm", group="catch")
            #     if left_status:  # 抓取成功
            #         # threading.Thread(target=catcher.arm.put_down, args=(), name=catcher.arm.local).start()
            #         # catcher.arm.put_down()
            #         left_is_put = True
            #
            #     if right_status:
            #         right_is_put = True
            #
            # elif right_result:
            #     pool.add_task(catcher_right.catch_apple, mode=mode, info="Right Arm", group="catch")
            #     right_status = pool.wait(info="Right Arm", mode=mode, group="catch")
            #     if right_status:
            #         right_is_put = True
            #
            # elif left_status:
            #     pool.add_task(catcher_left.catch_apple, mode=mode, info="Left Arm", group="catch")
            #     left_status = pool.wait(info="Left Arm", group="catch")
            #     if left_status:
            #         left_status = True
            #
            # else:
            #     pass

        elif mode == "left":
            pool.add_task(catcher_left.aim_apple, is_show=False, info='Left Arm', group="catch")
            left_status, left_result = pool.wait(info="Left Arm", group="catch")

            if left_status:
                pool.add_task(catcher_left.catch_apple, info="Left Arm", group="catch")
                left_status = pool.wait(info="Left Arm", group="catch")
                if left_status:
                    left_is_put = True

        elif mode == "right":
            pool.add_task(catcher_right.aim_apple, is_show=False, info='Right Arm', group="catch")
            right_status, right_result = pool.wait(info="Right Arm", group="catch")
            if right_status:
                pool.add_task(catcher_right.catch_apple, info="Right Arm", group="catch")
                right_status = pool.wait(info="Right Arm", group="catch")
                if right_status:
                    right_is_put = True

        elif mode == "left next":
            pool.add_task(detect_and_aim_apple_threading, catcher_left, mode=mode, is_show=False, info='Left Arm', group="catch")
            left_status = pool.wait(info="Left Arm", group="catch")
            time.sleep(0.5)
            if left_status:
                left_is_put = True
            elif left_status is None:
                pool.add_task(detect_and_aim_apple_threading, catcher_left, mode=mode, info="Left Arm", group="catch")
                left_status = pool.wait(info="Left Arm", group="catch")
                if left_status:
                    left_is_put = True

            arm_left.put_down(500)
            time.sleep(0.5)
            pool.add_task(arm_left.start_aim, mode="apple", info="left_thread", group="aim")
            pool.wait("left_thread", group="aim")

            time.sleep(0.5)
            pool.add_task(detect_and_aim_apple_threading, catcher_left, mode=mode, is_show=False, info='Left Arm', group="catch")
            left_status = pool.wait(info="Left Arm", group="catch")
            time.sleep(0.5)

            if left_status:
                left_is_put = True
            elif left_status is None:
                pool.add_task(detect_and_aim_apple_threading, catcher_left, mode=mode, info="Left Arm", group="catch")
                left_status = pool.wait(info="Left Arm", group="catch")
                if left_status:
                    left_is_put = True


        elif mode == "right next":
            pool.add_task(detect_and_aim_apple_threading, catcher_right, mode=mode, is_show=False, info='Right Arm', group="catch")
            right_status = pool.wait(info="Right Arm", group="catch")
            time.sleep(0.5)

            if right_status:
                right_is_put = True
            elif right_status is None:  # 如果第一次失败
                pool.add_task(detect_and_aim_apple_threading, catcher_right, mode=mode, is_show=False, info='Right Arm',
                              group="catch")
                right_status = pool.wait(info="Right Arm", group="catch")
                if right_status:
                    right_is_put = True

            time.sleep(0.5)
            pool.add_task(arm_left.start_aim, mode="apple", info="left_thread", group="aim")
            pool.wait("left_thread", group="aim")

            time.sleep(0.5)
            pool.add_task(detect_and_aim_apple_threading, catcher_left, mode=mode, is_show=False, info='Left Arm', group="catch")
            left_status = pool.wait(info="Left Arm", group="catch")
            if left_status:
                left_is_put = True
            elif left_status is None:
                pool.add_task(detect_and_aim_apple_threading, catcher_left, mode=mode, info="Left Arm", group="catch")
                left_status = pool.wait(info="Left Arm", group="catch")
                if left_status:
                    left_is_put = True


        pool.wait("Right Arm", group="catch")
        pool.wait("Left Arm", group="catch")

        if right_is_put:
            pool.add_task(arm_right.put_down, info="down_right", group="put_down")
            right_is_put = False

        if left_is_put:
            pool.add_task(arm_left.put_down, info="down_left", group="put_down")
            left_is_put = False

        pool.wait(info="down_right", group="put_down")
        pool.wait(info="down_left", group="put_down")

        time.sleep(1)
        arm_left.reset()
        arm_right.reset()
        time.sleep(1)

        if mode == "all":
            arm_left.send_pwm(0, 2000, times=300)
            time.sleep(0.3)
            pool.add_task(arm_left.start_aim, mode="apple", info="left_thread", group="aim")
            pool.wait("left_thread", group="aim")
            pool.add_task(catcher_left.aim_apple, is_show=False, info='Left Arm', group="catch")
            left_status, left_result = pool.wait(info="Left Arm", group="catch")
            if left_status:
                pool.add_task(catcher_left.catch_apple, info="Left Arm", group="catch")
                left_status = pool.wait(info="Left Arm", group="catch")
                if left_status:
                    left_is_put = True
            arm_left.reset_except_lower_and_claw(times=500)
            time.sleep(1)
            if left_is_put:
                pool.add_task(arm_left.put_down, info="down_left", group="put_down")
                pool.wait(info="down_left", group="put_down")
                time.sleep(1)

    arm_left.reset()
    arm_right.reset()

    # 无论是否成功，都进行下一步
    return True

if __name__ == "__main__":
    # catcher, camera, arm = class_init(Option
    print("初始化完毕")
    input("请按回车键继续")
    procedure = Procedure()
    action = Procedure_Action(procedure)
    Action = action.Action
    task = {
        "十字标对准动作": Action(arm_left.send_ordinate, x=280, y=20, z=240, angle=-75, mode=0, times=700,
                                 sleep_time=1),
        "十字标对正": Action(detect_cross_and_run, goal_x=-300, goal_y=0, clockwise_rotate=False, sleep_time=1),
        "前进0.7": Action(car.go_distance, 0.7, sleep_time=0.2),
        "苹果预瞄准动作": Action(arm_left.start_aim, mode="apple", sleep_time=1),
        "苹果预识别": Action(pre_detect_apple, camera_left, sleep_time=0.5),
        "前进0.25": Action(car.go_distance, 0.25, sleep_time=0.2),
        "苹果抓取动作": Action(prepare_aim, is_get_result=True, sleep_time=0.5),
        "苹果抓取": Action(catch_apple_all_action, is_get_result=True, sleep_time=0.5),
        "顺时针旋转90": Action(car.yaw_adjustment, 90, sleep_time=0.5),
        "后退0.4": Action(car.go_distance, -0.4, sleep_time=0.2),
        "苹果预瞄准动作1": Action(arm_left.start_aim, mode="apple", sleep_time=1),
        "苹果预识别1": Action(pre_detect_apple, camera_left, sleep_time=0.5),
        "平移0.1，前进0.2": Action(car.sideway, 0.2, -0.1),
        "苹果抓取动作1": Action(prepare_aim, is_get_result=True, sleep_time=0.5),
        "苹果抓取1": Action(catch_apple_all_action, is_get_result=True, sleep_time=0.5),
        "后退0.45": Action(car.go_distance, -0.45, sleep_time=0.2),
        "逆时针旋转90": Action(car.yaw_adjustment, -90, sleep_time=0.5),
        "前进1.6": Action(car.go_distance, 1.6, sleep_time=0.2),
        "顺时针旋转90度": Action(car.yaw_adjustment, 90, sleep_time=0.5),
        "前进0.45": Action(car.go_distance, 0.45, sleep_time=0.2),
    }
    procedure.count = 0
    procedure.create_tasks(task)
    # apple_num = 3
    # status = "all"
    # prepare_aim(apple_num, status)
    time.sleep(2)
    try:
        # catch_apple_all_action(red_apple_num=apple_num, mode=status)
        procedure.run()
    except KeyboardInterrupt:
        arm_right.close()
        arm_left.close()
    time.sleep(3)
    arm_right.reset()
    arm_left.reset()
    time.sleep(2)
    arm_right.close()
    arm_left.close()
