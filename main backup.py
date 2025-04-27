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
from utils.Arm_Control import *
from utils.Car import *
from utils.Cross_Detect import *
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from utils.Socket_Serve import *
from playsound import playsound
import rclpy
import traceback
import argparse
from functools import wraps

px = 0.0014  # 像素点大小
f = 1.8  # 摄像头焦距
w = 640
h = 480
camera_to_claw = 60  # 爪子离相机距离
height = 115  # 云台离地高度
procedure = Procedure()
action = Procedure_Action(procedure)
Action = action.Action
colorama.init(autoreset=True)
count_num = {
    "apple": 0,
    "fruit": 0,
    "vegetable": 0,
}

@contextlib.contextmanager
def replace_print_with_procedure():
    global procedure
    original_print = builtins.print
    procedure.original_print = original_print
    builtins.print = procedure.color_print


class Media_Player:
    def __init__(self):
        self.Step = 0
        self.path = "/home/dianfei/catch_robot_v2/media/"
        self.media_dict = {"start": "start1.mp3",
                           "small_car_end": "small_end.mp3",
                           "big_car_end": "big_end.mp3",
                           "apple": "8apples.mp3",}
        self.is_running = False

    def play(self, media, block=False, is_in_dic=True):
        if not self.is_running:
            self.is_running = True
            if is_in_dic:
                playsound(self.path + self.media_dict[media],  block=block)
            else:
                playsound(self.path + media, block=block)
            self.is_running = False
            return True
        else:
            return False

    def play_(self, video):
        os.system("play " + self.path + video)


def check_socket(socket_server: Socket_Server, media_player: Media_Player):
    while not socket_server.is_end:
        time.sleep(3)
    while True:
        print("播放小车结束语音")
        result = media_player.play("small_car_end", block=True)
        if result:
            break


def check_is_play():
    if socket_server.is_end:
        return True
    else:
        while True:
            print("播放小车结束语音")
            result = media_player.play("small_car_end", block=True)
            break
        return True


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
            camera_height = Z + height + camera_to_claw
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
        x_move = dis * sin * 2.5
        y_move = dis * cos * 2.5
        print(f"x_move: {x_move}, y_move: {y_move}")

        return True, x_move, y_move

    else:
        return False, None, None


def detect_cross_and_run(count=0, bias=0.03, goal_x=0, goal_y=0, clockwise_rotate=True, mode="left", is_move=True):
    status, x_move, y_move = detect_cross(mode=mode)
    right_arm_to_mid_dis = 90
    left_arm_to_mid_dis = 90
    arm_to_car_mid_dis = 100
    x_bias = -50
    y_bias = -20
    rotate_x_bias = -70

    rotate_y_bias = -70
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
            x_dis = x_move + x + arm_to_car_mid_dis + goal_x + rotate_x_bias
            y_dis = y_move - right_arm_to_mid_dis + goal_y + rotate_y_bias
            final_x = x_dis / 1000
            final_y = -y_dis / 1000

        print(f"x_move(摄像头离十字标距离): {x_move}, y_move: {y_move}, y_dis: {y_dis}, x_dis: {x_dis}")
        print(f"当前车离十字标距离：{x_move + x + arm_to_car_mid_dis}")
        print(f"final_x: {final_x}, final_y: {final_y}")
        if is_move:
            arm.reset()
            time.sleep(1)
            car.go_distance(final_x, is_print=True)
            time.sleep(1)
            if clockwise_rotate:
                car.yaw_adjustment(90, is_print=True)
                final_y = -final_y
            else:
                car.yaw_adjustment(-90, is_print=True)

            time.sleep(1)
            car.go_distance(final_y, is_print=True)
            time.sleep(1)
        return True
    else:
        if count > 4:
            print("未检测到交叉点")
            # os.system("v4l2-ctl --device=/dev/video_right --set-ctrl=exposure_time_absolute=200")
            if mode == "right":
                arm_right.reset()
            else:
                arm_left.reset()

            if clockwise_rotate:
                car.go_distance(0.15)
                car.yaw_adjustment(90)
                car.go_distance(0.1)
            else:
                car.go_distance(-0.55)
                car.yaw_adjustment(-90)
                car.go_distance(-0.3)

            return True

        car.go_distance(0.1)
        time.sleep(1)
        result = detect_cross_and_run(count + 1, goal_x=goal_x, goal_y=goal_y, clockwise_rotate=clockwise_rotate)
        return result


def detect_cross_and_run_A_to_B(count=0, bias=-0.08, is_show=False):
    # camera_right.change_auto_exposure(False)
    # time.sleep(1)
    # os.system("v4l2-ctl --device=/dev/video_right --set-ctrl=exposure_time_absolute=100")  # 减低曝光
    # time.sleep(1)
    status, x_move, y_move = detect_cross(mode="right", is_show=is_show)
    right_arm_to_mid_dis = 90
    arm_to_car_mid_dis = 100
    rotate_x_bias = 80
    if status:
        x, y, z, angle = arm_right.get_ordinate()
        x_dis = x_move + x + arm_to_car_mid_dis + rotate_x_bias
        y_dis = y_move - right_arm_to_mid_dis
        final_x = x_dis / 1000
        final_y = -y_dis / 1000
        print(f"final_x: {final_x}, final_y: {final_y}")
        arm_right.reset()
        # os.system("v4l2-ctl --device=/dev/video_right --set-ctrl=exposure_time_absolute=200")
        # camera_right.change_auto_exposure(False)
        car.go_distance(final_x+bias)
        time.sleep(0.2)
        car.yaw_adjustment(90)
        time.sleep(0.2)
        car.go_distance(final_y)
        return True
    else:
        if count > 4:
            # os.system("v4l2-ctl --device=/dev/video_right --set-ctrl=exposure_time_absolute=200")
            arm_right.reset()
            # camera_right.change_auto_exposure(False)
            car.yaw_adjustment(90)
            time.sleep(1)
            car.go_distance(0.1)
            print("未检测到交叉点")
            return True
        time.sleep(1)
        car.go_distance(0.2)
        result = detect_cross_and_run_A_to_B(count + 1)
        return result


def detect_and_aim_threading(catcher: Catcher, mode="fruit", is_show=False):
    time.sleep(0.1)
    if mode == "fruit":
        status, result = catcher.aim_fruit(is_show=is_show)
    else:
        status, result = catcher.aim_vegetable(is_show=is_show)

    if status:  # 对齐成功
        time.sleep(0.1)
        if mode == "fruit":
            catch_status = catcher.catch_fruit()
        else:
            catch_status = catcher.catch_vegetable()

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


def pre_aim_action():
    """
    在AB区确定抓取的是水果还是蔬菜
    :return:
    """
    pool.wait("down_right", group="put_down")
    pool.wait("down_left", group="put_down")
    # arm_right.reset()
    # arm_left.reset()
    # time.sleep(0.5)
    pool.add_task(arm_right.pre_aim, times=800, action=0, info="right_thread", group="aim")
    pool.add_task(arm_left.pre_aim, times=800, action=0,info="left_thread", group="aim")
    pool.wait("right_thread", group="aim")
    pool.wait("left_thread", group="aim")
    time.sleep(1)
    pool.add_task(catcher_right.camera.detect, mode=None, info="right_thread", group="detect")
    pool.add_task(catcher_left.camera.detect, mode=None, info="left_thread", group="detect")

    right_mode = None
    left_mode = None

    for future in as_completed(pool.tasks["detect"].values()):
        info = [key for key, value in pool.tasks["detect"].items() if value == future][0]  # 得到当前完成线程的名字
        result = future.result()  # 获得线程函数完成的返回值
        print(result)
        if info == "left_thread":
            if result[0]["color"] == "red_vegetable":
                left_mode = "vegetable"
            elif result[0]["color"] == "red_fruit":
                left_mode = "fruit"
            elif result[0]["color"] == "green_vegetable" or result[0]["color"] == "green_fruit":
                right_mode = "green"
            else:
                left_mode = None
        elif info == "right_thread":
            if result[0]["color"] == "red_vegetable":
                right_mode = "vegetable"
            elif result[0]["color"] == "red_fruit":
                right_mode = "fruit"
            elif result[0]["color"] == "green_vegetable" or result[0]["color"] == "green_fruit":
                right_mode = "green"
            else:
                right_mode = None

    if left_mode is None:
        pool.add_task(arm_left.pre_aim, times=800, action=1, info="left_thread", group="aim")
        pool.wait("left_thread", group="aim")
        time.sleep(1)
        pool.add_task(catcher_left.camera.detect, mode=None, info="left_thread", group="detect")
    if right_mode is None:
        pool.add_task(arm_right.pre_aim, times=800, action=1, info="right_thread", group="aim")
        pool.wait("right_thread", group="aim")
        time.sleep(1)
        pool.add_task(catcher_right.camera.detect, mode=None, info="right_thread", group="detect")

    for future in as_completed(pool.tasks["detect"].values()):
        info = [key for key, value in pool.tasks["detect"].items() if value == future][0]  # 得到当前完成线程的名字
        result = future.result()  # 获得线程函数完成的返回值
        print(result)
        if info == "left_thread":
            if result[0]["color"] == "red_vegetable":
                left_mode = "vegetable"
            elif result[0]["color"] == "red_fruit":
                left_mode = "fruit"
            else:
                left_mode = None
        elif info == "right_thread":
            if result[0]["color"] == "red_vegetable":
                right_mode = "vegetable"
            elif result[0]["color"] == "red_fruit":
                right_mode = "fruit"
            else:
                right_mode = None

    return True, left_mode, right_mode


def aim_and_catch_action(left_mode, right_mode, is_move=True):
    """
    AB区瞄准水果动作
    :param left_mode:
    :return:
    """
    # left_mode, right_mode = pre_aim_action()

    if left_mode == "vegetable":
        pool.add_task(arm_left.start_aim, mode=left_mode, info="left_thread", group="aim")
        pool.add_task(arm_right.start_aim, mode=right_mode, info="right_thread", group="aim")
        pool.wait("right_thread", group="aim")
        pool.wait("left_thread", group="aim")
        time.sleep(1)
        result = catch_all_action(left_mode=left_mode, right_mode=right_mode, is_move=is_move)
        return result

    elif left_mode == "fruit":
        if right_mode != "fruit":  # 一边水果一边蔬菜
            pool.add_task(arm_left.start_aim, mode=left_mode, info="left_thread", group="aim")
            pool.add_task(arm_right.start_aim, mode=right_mode, info="right_thread", group="aim")
            pool.wait("right_thread", group="aim")
            pool.wait("left_thread", group="aim")
            time.sleep(1)
            result = catch_all_action(left_mode=left_mode, right_mode=right_mode, is_move=is_move)
            return result
        else:  # 两边都是水果，需要一边抓完再抓另一边
            left_mode = "fruit"
            right_mode = "fruit"
            arm_right.reset()
            pool.add_task(arm_left.start_aim, mode=left_mode, info="left_thread", group="aim")
            pool.wait(info="left_thread", group="aim")
            time.sleep(0.2)
            catch_all_action(left_mode=left_mode, right_mode=None)

            pool.add_task(arm_right.start_aim, mode=right_mode, info="right_thread", group="aim")
            pool.wait(info="right_thread", group="aim")
            time.sleep(0.2)
            catch_all_action(left_mode=None, right_mode=right_mode, is_move=is_move)
    else:
        arm_left.reset()
        arm_right.reset()
    time.sleep(0.2)

    # pool.add_task(arm_right.start_aim, mode=right_mode, info="right_thread", group="aim")
    # time.sleep(0.2)
    # pool.add_task(arm_left.start_aim, mode=left_mode, info="left_thread", group="aim")
    #
    # pool.wait("right_thread", group="aim")
    # pool.wait("left_thread", group="aim")
    return True


def catch_all_action(left_mode, right_mode, is_move=True):
    # if left_mode == "fruit":
    #     right_mode = "vegetable"
    # else:
    #     left_mode = "vegetable"
    #     right_mode = "fruit"

    right_is_put = False
    left_is_put = False

    if left_mode is not None and right_mode is not None:

        right_x_dis = None
        left_x_dis = None
        pool.add_task(catcher_left.detect_x_dis, is_show=False, info="left_thread", group="detect")
        pool.add_task(catcher_right.detect_x_dis, is_show=False, info="right_thread", group="detect")
        for future in as_completed(pool.tasks["detect"].values()):
            info = [key for key, value in pool.tasks["detect"].items() if value == future][0]  # 得到当前完成线程的名字
            result = future.result()  # 获得线程函数完成的返回值
            if info == "left_thread":
                left_x_dis = result
            else:
                right_x_dis = result

        if right_x_dis is not None:
            if left_x_dis is not None:
                move = (left_x_dis - right_x_dis) / 2
            else:
                move = right_x_dis
        else:
            if left_x_dis is not None:
                move = left_x_dis
            else:
                move = None

        if move is not None:
            print("即将移动的距离：", move)
            if abs(move) > 20:
                move = move / 1000
                if is_move:
                    car.go_distance(move)

        pool.add_task(detect_and_aim_threading, catcher_right, mode=right_mode, is_show=False, info='Right Arm', group="catch")
        pool.add_task(detect_and_aim_threading, catcher_left, mode=left_mode, is_show=False, info='Left Arm', group="catch")

    elif left_mode is not None:
        pool.add_task(detect_and_aim_threading, catcher_left, mode=left_mode, is_show=False, info='Left Arm',
                      group="catch")
    elif right_mode is not None:
        pool.add_task(detect_and_aim_threading, catcher_right, mode=right_mode, is_show=False, info='Right Arm',
                      group="catch")
    else:
        arm_left.reset()
        arm_right.reset()

    for i in range(2):
        for future in as_completed(pool.tasks["catch"].values()):
            info = [key for key, value in pool.tasks["catch"].items() if value == future][0]    # 得到当前完成线程的名字
            result = future.result()    # 获得线程函数完成的返回值
            if result:
                if info == 'Right Arm':
                    pool.remove_task(info, group="catch")
                    arm_right.reset_except_claw()
                    right_is_put = True
                elif info == 'Left Arm':
                    pool.remove_task(info, group="catch")
                    arm_left.reset_except_claw()
                    left_is_put = True
                time.sleep(1.5)
            elif result is None:
                if i == 0:
                    if info == 'Right Arm':
                        pool.add_task(detect_and_aim_threading, catcher_right, mode=right_mode, is_show=False, info='Right Arm', group="catch")
                    elif info == 'Left Arm':
                        pool.add_task(detect_and_aim_threading, catcher_left, mode=left_mode, is_show=False, info='Left Arm', group="catch")
                else:
                    if info == 'Right Arm':
                        arm_right.reset()
                    elif info == 'Left Arm':
                        arm_left.reset()

    pool.wait("Right Arm", group="catch")
    pool.wait("Left Arm", group="catch")
    if right_is_put:
        count_num[right_mode] += 1
        pool.add_task(arm_right.put_down, info="down_right", group="put_down")
        time.sleep(3)

    if left_is_put:
        count_num[left_mode] += 1
        pool.add_task(arm_left.put_down, info="down_left", group="put_down")

    # 无论是否成功，都进行下一步
    return True


def proof_location(result, goal_dis):
    print(result)
    bias = 0
    deep = result["deep"]
    x_dis = result["x_distance"]
    y_dis = result["y_distance"]
    if abs(x_dis) > 250:
        bias = 0.03
    elif abs(x_dis) < 45:
        bias = -0.02
    if abs(y_dis) > 120 or abs(x_dis) > 230:
        bias += 0.05
    side = x_dis * 0.0014 * deep / 1.8
    print(side)
    A = -74.79063
    B = 2.64051
    goal_side = A + B * -side

    dis = int(deep) / 1000 - goal_dis
    side = int(goal_side) / 1000 - bias*goal_side/abs(goal_side)
    dis = int(dis * 1000) / 1000
    side = int(side * 1000) / 1000
    print(f"dis: {dis}, side: {side}")
    car.sideway(dis, -side)


def pre_detect_apple(camera: Camera, goal_dis=0.30, mode="left", is_show=False):
    """

    :param mode: 那一边没有苹果，就跟偏向那边，避免误识别
    :return: 返回值的使用：先判断红苹果数，有的话后面的值才有意义，middle意味着两边可同时抓，None表示有多个需要抓，且不能同时
    """
    frame, results = camera.model_detect(conf=0.6, is_show=is_show, limit_area=3000)

    red_apple_num = 0
    green_apple_num = 0
    red_apple_index_list = []
    green_apple_index_list = []
    status = None

    if results[0]["color"] == "None":     # 没有东西的情况，正常不会发生
        print("没有识别到东西")
        return True, red_apple_num, None

    results.sort(key=lambda x: x["area"], reverse=True)
    if len(results) >= 4:
        results = results[:4]   # 现以面积筛选掉小面积的
        if mode == "left":
            results.sort(key=lambda x: x["cx"])
        else:
            results.sort(key=lambda x: x["cx"], reverse=True)
        goal_results = results[:3]
    else:
        goal_results = results

    # if results[0]["area"] < 5500:
    #     car.go_distance(0.03)
    goal_results.sort(key=lambda x: x["cx"])
    arm_left.reset()
    print(goal_results)
    for i, result in enumerate(goal_results):

        if i == 0:
            cx = result["cx"]
            cy = result["cy"]
            w = result["w"]
            h = result["h"]
            x1 = cx - 0.5 * w
            y1 = cy - 0.5 * h
            x2 = cx + 0.5 * w
            y2 = cy + 0.5 * h
            cv2.putText(frame,
                            f'goal_apple', (int(x1), int(y2 - 2)),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.60, (255, 0, 0),
                            thickness=1)
            # 获取当前时间，并格式化为文件名
            current_time = time.strftime("%m-%d %H:%M:%S", time.localtime())
            cv2.imwrite(f"/home/dianfei/catch_robot_v2/picture/pre_detect_{current_time}.jpg", frame)
            proof_location(result=result, goal_dis=goal_dis)

        if result["cx"] < 30 or result["cx"] > 620:     # 过于靠边抛弃
            continue
        if "red" in result["color"]:
            red_apple_num += 1
            red_apple_index_list.append(i)
        elif "green" in result["color"]:
            green_apple_num += 1
            green_apple_index_list.append(i)

    if red_apple_num == 0:
        status = None
    elif red_apple_num == 1:
        if red_apple_index_list[0] == 0 or red_apple_index_list[0] == 1:
            status = "left"
        else:
            status = "right"
    elif red_apple_num == 2:
        if red_apple_index_list[0] + 1 == red_apple_index_list[1]:  # 两个红色苹果紧挨
            if green_apple_index_list:
                if green_apple_index_list[0] == 0:  # 绿色最左
                    status = "right next"
                else:
                    status = "left next"
            else:
                status = "left next"
        else:
            status = "no next"
    elif red_apple_num == 3:
        status = "all"

    print(f"red_apple_num: {red_apple_num}, status: {status}")

    return True, red_apple_num, status

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
        arm_left.reset()
        arm_right.reset()
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
            time.sleep(0.3)
            arm_left.send_pwm(5, 1700, times=300)
            pool.add_task(arm_left.start_aim, mode="apple", info="left_thread", group="aim")
            pool.add_task(arm_right.start_aim, mode="apple", info="right_thread", group="aim")
            pool.wait("right_thread", group="aim")
            pool.wait("left_thread", group="aim")
        elif mode == "left next":
            arm_right.reset()
            arm_left.send_pwm(0, 700, times=300)
            arm_right.add_pwm(3, 200)
            pool.add_task(arm_left.start_aim, mode="apple", info="left_thread", group="aim")
            pool.wait("left_thread", group="aim")
        elif mode == "right next":
            arm_left.reset()
            arm_right.send_pwm(0, 2000, times=300)
            arm_left.add_pwm(3, 200)
            pool.add_task(arm_right.start_aim, mode="apple", info="right_thread", group="aim")
            pool.wait("right_thread", group="aim")

    time.sleep(0.5)

    return True


def detect_and_aim_apple_threading(catcher: Catcher, is_show=False):
    time.sleep(0.1)
    status, result = catcher.aim_apple(is_show=is_show)

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


def catch_apple_all_action(red_apple_num, mode, side="right"):
    right_is_put = False
    left_is_put = False
    if red_apple_num > 0:

        if mode == "no next" or mode == "all":    # (青苹果在中间或是全是红色，两边瞄准后同时抓）
            pool.add_task(catcher_right.aim_apple, is_show=False, info='Right Arm', group="catch")
            pool.add_task(catcher_left.aim_apple, is_show=False, info='Left Arm', group="catch")
            right_status, right_result = pool.wait(info="Right Arm", group="catch")
            left_status, left_status = pool.wait(info="Left Arm", group="catch")

            # for i in range(2):
            #     for future in as_completed(pool.tasks["catch"].values()):
            #         info = [key for key, value in pool.tasks["catch"].items() if value == future][0]  # 得到当前完成线程的名字
            #         result = future.result()  # 获得线程函数完成的返回值
            #         if isinstance(result, tuple):
            #             status = result[0]
            #         else:
            #             status = result
            #         if status:
            #             if info == "Left Arm":
            #                 pool.remove_task(info, group="catch")
            #                 pool.add_task(catcher_left.catch_apple, mode=mode, info="Left Arm", group="catch")
            #                 left_is_put = True
            #             elif info == "Right Arm":
            #                 pool.remove_task(info, group="catch")
            #                 pool.add_task(catcher_right.catch_apple, mode=mode, info="Right Arm", group="catch")
            #                 right_is_put = True
            #         elif result is None:
            #             if i == 0:
            #                 if info == 'Right Arm':
            #                     pool.add_task(catcher_right.aim_apple, is_show=False, mode=mode, info='Right Arm', group="catch")
            #                 elif info == 'Left Arm':
            #                     pool.add_task(catcher_left.aim_apple, is_show=False, mode=mode, info='Left Arm', group="catch")
            #             else:
            #                 if info == 'Right Arm':
            #                     arm_right.reset()
            #                 elif info == 'Left Arm':
            #                     arm_left.reset()

            if right_status and left_status:    # 正常情况 （两边对准完毕）
                pool.add_task(catcher_left.catch_apple, info="Left Arm", group="catch")
                pool.add_task(catcher_right.catch_apple, info="Right Arm", group="catch")

                left_status = pool.wait(info="Left Arm", group="catch")
                right_status = pool.wait(info="Right Arm", group="catch")
                if left_status:  # 抓取成功
                    # threading.Thread(target=catcher.arm.put_down, args=(), name=catcher.arm.local).start()
                    # catcher.arm.put_down()
                    left_is_put = True

                if right_status:
                    right_is_put = True

            elif right_result:
                pool.add_task(catcher_right.catch_apple, info="Right Arm", group="catch")
                right_status = pool.wait(info="Right Arm", group="catch")
                if right_status:
                    right_is_put = True

            elif left_status:
                pool.add_task(catcher_left.catch_apple, info="Left Arm", group="catch")
                left_status = pool.wait(info="Left Arm", group="catch")
                if left_status:
                    left_is_put = True

            else:
                pass

        elif mode == "left":
            pool.add_task(catcher_left.aim_apple, is_show=False, info='Left Arm', group="aim")
            left_status, left_result = pool.wait(info="Left Arm", group="aim")

            if left_status:
                pool.add_task(catcher_left.catch_apple, info="Left Arm", group="catch")
                left_status = pool.wait(info="Left Arm", group="catch")
                if left_status:
                    left_is_put = True

        elif mode == "right":
            pool.add_task(catcher_right.aim_apple, is_show=False, info='Right Arm', group="aim")
            right_status, right_result = pool.wait(info="Right Arm", group="aim")
            if right_status:
                pool.add_task(catcher_right.catch_apple, info="Right Arm", group="catch")
                right_status = pool.wait(info="Right Arm", group="catch")
                if right_status:
                    right_is_put = True

        elif mode == "left next" or mode == "right next":
            if mode == "left next":
                pool.add_task(detect_and_aim_apple_threading, catcher_left, is_show=False, info='Left Arm', group="catch")
                left_status = pool.wait(info="Left Arm", group="catch")
                time.sleep(0.5)
                if left_status:
                    left_is_put = True
                elif left_status is None:
                    pool.add_task(detect_and_aim_apple_threading, catcher_left, info="Left Arm", group="catch")
                    left_status = pool.wait(info="Left Arm", group="catch")
                    if left_status:
                        left_is_put = True
                if left_is_put:
                    count_num["apple"] += 1
                    pool.add_task(arm_left.put_down, info="down_left", group="put_down")
                    left_is_put = False
                pool.wait(info="down_left", group="put_down")
                time.sleep(0.5)

            else:
                pool.add_task(catcher_right.aim_apple, is_show=False, info='Right Arm', group="aim")
                right_status, right_result = pool.wait(info="Right Arm", group="aim")
                if right_status:
                    pool.add_task(catcher_right.catch_apple, info="Right Arm", group="catch")
                    right_status = pool.wait(info="Right Arm", group="catch")
                    if right_status:
                        right_is_put = True

                time.sleep(0.5)
                if right_is_put:
                    count_num["apple"] += 1
                    pool.add_task(arm_right.put_down, info="down_right", group="put_down")
                    right_is_put = False

                pool.wait(info="down_right", group="put_down")
                time.sleep(0.5)

            if side == "right":
                arm_right.send_pwm(0, 2000, times=300)
                time.sleep(0.2)
                pool.add_task(arm_right.start_aim, mode="apple", info="right_thread", group="aim")
                pool.wait("right_thread", group="aim")

                time.sleep(0.5)
                pool.add_task(detect_and_aim_apple_threading, catcher_right, is_show=False, info='Right Arm', group="catch")
                right_status = pool.wait(info="Right Arm", group="catch")

                if right_status:
                    right_is_put = True
                elif right_status is None:
                    pool.add_task(detect_and_aim_apple_threading, catcher_right, info="Right Arm", group="catch")
                    right_status = pool.wait(info="Right Arm", group="catch")
                    if right_status:
                        right_is_put = True

            else:
                arm_left.send_pwm(0, 700, times=300)
                time.sleep(0.5)
                pool.add_task(arm_left.start_aim, mode="apple", info="left_thread", group="aim")
                pool.wait("left_thread", group="aim")

                time.sleep(0.5)
                pool.add_task(catcher_left.aim_apple, is_show=False, info='Left Arm', group="catch")
                left_status, left_result = pool.wait(info="Left Arm", group="catch")

                if left_status:  # 对齐成功
                    pool.add_task(catcher_left.catch_apple, info="Left Arm", group="catch")
                    left_status = pool.wait(info="Left Arm", group="catch")
                    if left_status:
                        left_is_put = True

        # elif mode == "right next":
        #
        #     arm_left.send_pwm(0, 700, times=300)
        #     time.sleep(0.5)
        #     pool.add_task(arm_left.start_aim, mode="apple", info="left_thread", group="aim")
        #     pool.wait("left_thread", group="aim")
        #
        #     time.sleep(0.5)
        #     pool.add_task(catcher_left.aim_apple, is_show=False, info='Left Arm', group="catch")
        #     left_status, left_result = pool.wait(info="Left Arm", group="catch")
        #
        #     if left_status:  # 对齐成功
        #         pool.add_task(catcher_left.catch_apple, info="Left Arm", group="catch")
        #         left_status = pool.wait(info="Left Arm", group="catch")
        #         if left_status:
        #             left_is_put = True

        pool.wait("Right Arm", group="catch")
        pool.wait("Left Arm", group="catch")

        if right_is_put:
            count_num["apple"] += 1
            pool.add_task(arm_right.put_down, info="down_right", group="put_down")
            right_is_put = False

        if left_is_put:
            count_num["apple"] += 1
            pool.add_task(arm_left.put_down, info="down_left", group="put_down")
            left_is_put = False

        pool.wait(info="down_right", group="put_down")
        pool.wait(info="down_left", group="put_down")

        time.sleep(1)
        arm_left.reset()
        arm_right.reset()
        time.sleep(2)

        if mode == "all":
            if side == "left":
                arm_left.send_pwm(0, 700, times=300)
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

                # elif left_status is False:
                #     arm_right.send_pwm(0, 2000, times=300)
                #     time.sleep(0.3)
                #     pool.add_task(arm_right.start_aim, mode="apple", info="right_thread", group="aim")
                #     pool.wait("right_thread", group="aim")
                #     pool.add_task(catcher_right.aim_apple, is_show=False, info='Right Arm', group="catch")
                #     right_status, right_result = pool.wait(info="Right Arm", group="catch")
                #
                #     if right_status:
                #         pool.add_task(catcher_right.catch_apple, info="Right Arm", group="catch")
                #         right_status = pool.wait(info="Right Arm", group="catch")
                #         if right_status:
                #             right_is_put = True
                #
                #     arm_right.reset_except_lower_and_claw(times=500)
                #     time.sleep(1)
                #     if right_is_put:
                #         count_num["apple"] += 1
                #         pool.add_task(arm_right.put_down, info="down_right", group="put_down")
                #         pool.wait(info="down_right", group="put_down")
                #         time.sleep(1)

                arm_left.reset_except_lower_and_claw(times=500)
                time.sleep(1)
                if left_is_put:
                    count_num["apple"] += 1
                    pool.add_task(arm_left.put_down, info="down_left", group="put_down")
                    pool.wait(info="down_left", group="put_down")
                    time.sleep(1)

            elif side == "right":
                arm_right.send_pwm(0, 2000, times=300)
                time.sleep(0.3)
                pool.add_task(arm_right.start_aim, mode="apple", info="right_thread", group="aim")
                pool.wait("right_thread", group="aim")
                pool.add_task(catcher_right.aim_apple, is_show=False, info='Right Arm', group="catch")
                right_status, right_result = pool.wait(info="Right Arm", group="catch")

                if right_status:
                    pool.add_task(catcher_right.catch_apple, info="Right Arm", group="catch")
                    right_status = pool.wait(info="Right Arm", group="catch")
                    if right_status:
                        right_is_put = True
                # elif right_status is False:
                #     arm_left.send_pwm(0, 700, times=300)
                #     time.sleep(0.3)
                #     pool.add_task(arm_left.start_aim, mode="apple", info="left_thread", group="aim")
                #     pool.wait("left_thread", group="aim")
                #     pool.add_task(catcher_left.aim_apple, is_show=False, info='Left Arm', group="catch")
                #     left_status, left_result = pool.wait(info="Left Arm", group="catch")
                #     if left_status:
                #         pool.add_task(catcher_left.catch_apple, info="Left Arm", group="catch")
                #         left_status = pool.wait(info="Left Arm", group="catch")
                #         if left_status:
                #             left_is_put = True
                #     arm_left.reset_except_lower_and_claw(times=500)
                #     time.sleep(1)
                #     if left_is_put:
                #         count_num["apple"] += 1
                #         pool.add_task(arm_left.put_down, info="down_left", group="put_down")
                #         pool.wait(info="down_left", group="put_down")
                #         time.sleep(1)

                arm_right.reset_except_lower_and_claw(times=500)
                time.sleep(1)

                if right_is_put:
                    count_num["apple"] += 1
                    pool.add_task(arm_right.put_down, info="down_right", group="put_down")
                    pool.wait(info="down_right", group="put_down")
                    time.sleep(1)

    arm_left.reset()
    arm_right.reset()
    time.sleep(0.5)
    # 无论是否成功，都进行下一步
    return True


def play_by_num(block=False):
    apple_videos = {
        3: "3apples.mp3",
        4: "4apples.mp3",
        5: "5apples.mp3",
        6: "6apples.mp3",
        7: "7apples.mp3",
        8: "8apples.mp3",
    }
    if count_num["apple"] < 3:
        video = "8apples.mp3"
    else:
        video = apple_videos[count_num["apple"]]
    media_player.play(video, block=block, is_in_dic=False)
    return True


def class_init(Option):
    img_receiver = Image_Receiver(name=Option["video"]["node_name"], topic=Option["video"]["img_topic"])
    executor = rclpy.executors.SingleThreadedExecutor()
    camera = Camera(Option["video"]["video_ser"], img_receiver=img_receiver, executor=executor, system_type="Linux")
    arm = Arm(local=Option["local"])
    arm.init(Option["arm_ser"])
    arm.run_thread()
    catcher = Catcher(arm, camera)
    return catcher, camera, arm


import sys

class DualOutput:
    def __init__(self, filename):
        self.terminal = sys.stdout  # 保存控制台标准输出
        self.log_file = open(filename, "a")  # 打开日志文件

    def write(self, message):
        self.terminal.write(message)  # 输出到控制台
        self.log_file.write(message)  # 输出到文件

    def flush(self):
        self.terminal.flush()
        self.log_file.flush()



if __name__ == '__main__':
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
    parameter = argparse.ArgumentParser()
    parameter.add_argument("--need_socket", type=bool, default=False)
    args = parameter.parse_args()
    rclpy.init()
    pool = ThreadPool()
    catcher_left, camera_left, arm_left = class_init(Option["left"])
    catcher_right, camera_right, arm_right = class_init(Option["right"])
    cross_detect = Cross_Detect()
    car = Car_Bridge()
    car.init(Option["Car_Serial"])
    car.run_thread()
    media_player = Media_Player()
    replace_print_with_procedure()
    camera_right.model_detect()
    camera_left.model_detect()
    socket_server = Socket_Server(ip="192.168.90.164", port=6666)
    # # 将 sys.stdout 重定向到 DualOutput
    # sys.stdout = DualOutput("print_log.txt")
    input("按任意键开始")
    if args.need_socket:
        print("socket_server start")
        pool.add_task(socket_server.start_server_socket, info="socket_server", group="socket")
        pool.add_task(check_socket, socket_server, media_player, info="check", group="socket")
        while not socket_server.is_start:
            time.sleep(2)
            print("waiting...")
        print("即将移动")
        time.sleep(8)

    task = {
        # "前进0.23米": Action(car.go_distance, 0.23),
        # "旋转-90度": Action(car.yaw_adjustment, -90),
        # "播报开始语音": Action(media_player.play, "start"),
        # "前进0.54米": Action(car.go_distance, 0.54),
        # "A区第一组": {
        #     # "瞄准动作": Action(pre_aim_action),
        #     "抓取": Action(aim_and_catch_action, left_mode="fruit", right_mode="vegetable")
        # },

        # "A区第二组": {
        #     "前进0.97米": Action(car.go_distance, 0.95),
        #     # "瞄准动作": Action(pre_aim_action),
        #     "抓取": Action(aim_and_catch_action, left_mode="vegetable", right_mode="fruit"),
        # },

        # "A区第三组": {
        #     "前进1米": Action(car.go_distance, 1),
        #     # "瞄准动作": Action(pre_aim_action),
        #     "抓取": Action(aim_and_catch_action, left_mode="fruit", right_mode="vegetable"),
        # },
        # "A区到B区": {
        #     "前进0.6米": Action(car.go_distance, 0.6, sleep_time=0.2),
        #     "转向90": Action(car.yaw_adjustment, 90, sleep_time=0.2),
        #     "前进0.58米": Action(car.go_distance, 0.58, sleep_time=0.15),
        #     "十字标对准动作": Action(arm_right.send_ordinate, x=280, y=45, z=240, angle=-100, mode=0, times=700, sleep_time=2),
        #     "十字标对正": Action(detect_cross_and_run_A_to_B, is_show=False, sleep_time=0.2),
        #     "前进0.4": Action(car.go_distance, 0.4, sleep_time=0.2),
        # },

        # "B区第一组": {
        #     # "瞄准动作": Action(pre_aim_action),
        #     "抓取": Action(aim_and_catch_action, left_mode="fruit", right_mode="vegetable"),
        # },

        # "B区第二组": {
        #     "前进0.98米": Action(car.go_distance, 1),
        #     # "瞄准动作": Action(pre_aim_action),
        #     "抓取": Action(aim_and_catch_action, left_mode="vegetable", right_mode="fruit"),
        # },

        # "B区第三组": {
        #     "前进1米": Action(car.go_distance, 1),
        #     # "瞄准动作": Action(pre_aim_action),
        #     "抓取": Action(aim_and_catch_action, left_mode="fruit", right_mode="vegetable"),
        # },
        # "B区到C区": {
        #     "前进0.52米": Action(car.go_distance, 0.52, sleep_time=0.15),
        #     "转向-90": Action(car.yaw_adjustment, -90),
        #     "前进3.43米": Action(car.go_distance, 3.43, sleep_time=0.2),
        #     "右爪复位": Action(arm_right.reset),
        #     "播报结束语音": Action(media_player.play, "big_car_end", block=False),
        #     # "转向0": Action(car.yaw_adjustment, 0),
        # }
    }
    # task_D_1 = {
    #     "十字标对准动作": Action(arm_left.send_ordinate, x=280, y=20, z=240, angle=-75, mode=0, times=700, sleep_time=2),
    #     "十字标对正": Action(detect_cross_and_run, goal_x=-200, goal_y=-20, clockwise_rotate=False, sleep_time=0.5),
    #     "前进0.61": Action(car.go_distance, 0.61, sleep_time=0.2),
    #     "苹果预瞄准动作": Action(arm_left.start_aim, mode="apple", sleep_time=1),
    #     "苹果预识别": Action(pre_detect_apple, camera_left, goal_dis=0.33, mode="left", sleep_time=0.5),
    #     "苹果抓取动作": Action(prepare_aim, is_get_result=True, sleep_time=1.5),
    #     "苹果抓取": Action(catch_apple_all_action, side="right", is_get_result=True, sleep_time=0.5),
    #     "前进0.11": Action(car.go_distance, 0.11, sleep_time=0.2),
    #     "顺时针旋转90": Action(car.yaw_adjustment, 90, sleep_time=0.5),
    #     "后退0.45m": Action(car.go_distance, -0.45, sleep_time=0.2),
    #     "苹果预瞄准动作1": Action(arm_left.start_aim, mode="apple", sleep_time=1),
    #     "苹果预识别1": Action(pre_detect_apple, camera_left, goal_dis=0.33, mode="right", sleep_time=0.5),
    #     "苹果抓取动作1": Action(prepare_aim, is_get_result=True, sleep_time=1),
    #     "苹果抓取1": Action(catch_apple_all_action, side="left", is_get_result=True, sleep_time=0.5),
    #     "后退0.45": Action(car.go_distance, -0.45, sleep_time=0.2),
    #     "逆时针旋转90": Action(car.yaw_adjustment, -90, sleep_time=0.5),
    #     "前进1.7": Action(car.go_distance, 1.7, sleep_time=0.2),
    #     "顺时针旋转90度": Action(car.yaw_adjustment, 90, sleep_time=0.5),
    #     "前进0.45": Action(car.go_distance, 0.45, sleep_time=0.2),
    # }
    # task_D_2 = {
    #     "十字标对准动作": Action(arm_left.send_ordinate, x=280, y=20, z=240, angle=-75, mode=0, times=700, sleep_time=1),
    #     "十字标对正": Action(detect_cross_and_run, goal_x=315, goal_y=-30, clockwise_rotate=True, sleep_time=0.5),
    #     "前进0.56": Action(car.go_distance, 0.56, sleep_time=0.2),
    #     "苹果预瞄准动作": Action(arm_left.start_aim, mode="apple", sleep_time=1),
    #     "苹果预识别": Action(pre_detect_apple, camera_left, goal_dis=0.33, mode="left", sleep_time=0.5),
    #     "苹果抓取动作": Action(prepare_aim, is_get_result=True, sleep_time=1),
    #     "苹果抓取": Action(catch_apple_all_action, side="right", is_get_result=True, sleep_time=0.5),
    #     "前进0.13": Action(car.go_distance, 0.13, sleep_time=0.2),
    #     "顺时针旋转90": Action(car.yaw_adjustment, 90, sleep_time=0.5),
    #     "后退0.47": Action(car.go_distance, -0.47
    #                        , sleep_time=0.2),
    #     "苹果预瞄准动作1": Action(arm_left.start_aim, mode="apple", sleep_time=1),
    #     "苹果预识别1": Action(pre_detect_apple, camera_left, goal_dis=0.33,mode="right", sleep_time=0.5),
    #     "苹果抓取动作1": Action(prepare_aim, is_get_result=True, sleep_time=1),
    #     "苹果抓取1": Action(catch_apple_all_action, side="left", is_get_result=True, sleep_time=0.5),
    #     "后退0.65": Action(car.go_distance, -0.65, sleep_time=0.2),
    #     "顺时针旋转90度": Action(car.yaw_adjustment, 90, sleep_time=0.5),
    #     "前进1.42": Action(car.go_distance, 1.42, sleep_time=0.2),
    #     "播放苹果结束语音": Action(play_by_num, block=True),
    #     "检查是否播报了小车语音": Action(check_is_play, sleep_time=0.5),

    # }
    task_D = {"task_D1": task_D_1, "task_D_2": task_D_2}
    all_task = {
        "task_AB": task,
        "task_D": task_D
    }
    procedure.create_tasks(all_task)
    # car.Yaw = 0
    try:
        procedure.run()  # 1.7.4
    except Exception as e:
        # traceback.print_exc()
        error_message = traceback.format_exc()  # 获取完
        # 整的错误堆栈信息

        # 打印错误信息
        print("捕获到异常！详细信息如下：")
        print(error_message)

        # 将错误信息写入文件
        # with open("error_log.txt", "a") as file:
        #     file.write(error_message + "\n")  # 写入错误信息并换行

    finally:
        print(f"共抓取{count_num['apple']}个苹果, 共抓取{count_num['fruit']}个水果, 共抓取{count_num['vegetable']}个蔬菜")
        arm_left.reset()
        arm_right.reset()
        time.sleep(2)
        arm_left.close()
        arm_right.close()
        car.close()
        print("结束")
        time.sleep(3)
        # # 恢复 sys.stdout 到控制台标准输出，并关闭文件
        # sys.stdout.log_file.close()
        # sys.stdout = sys.stdout.terminal

    # time.sleep(5)
    #
    # arm_left.close()
    # arm_right.close()
    # car.close()
    # time.sleep(3)
