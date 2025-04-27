import cv2
import os
import math
import imutils
import time
from math import pi
import numpy as np
import threading
from collections import Counter
from utils.Run import *

from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from ultralytics import YOLO

apple_model = YOLO("/home/dianfei/catch_robot_v2/model/apple_v11n4.onnx", task="detect")
fruit_vegetable_model = YOLO(
    "/home/dianfei/catch_robot_v2/model/fruit_vegetable_v11n2.onnx", task="detect"
)


class Image_Receiver(Node):
    def __init__(self, name, topic="Image", parameter_overrides=[]):
        super().__init__(name, parameter_overrides=parameter_overrides)
        self.create_subscription(Image, topic, self.image_receive, 1)
        self.image = None

    def image_receive(self, msg):
        self.get_logger().info("Receive Image")
        frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(
            msg.height, msg.width, -1
        )
        if frame is not None:
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        # frame = cv2.flip(frame, -1)
        self.image = frame

    def getImage(self, executor):
        self.image = None
        # rclpy.spin_once(self, timeout_sec=5)
        executor.spin_once(timeout_sec=3)
        return self.image


def forecast_dis(dis, h, fruit_Z):
    # 抓水果时预测距离
    # dis为物体距离相机中心距离，h为框的长或高

    px = 0.0014
    f = 1.8
    dis = abs(dis)
    if dis == 0:
        deep = int(f * fruit_Z / (h * px)) * 2
        move = 0
    elif dis > h:
        dy = dis - h / 2
        angle_a = math.atan(dy * px / f)
        angle_b = math.atan(dis * px / f)
        c = math.sin(pi / 2 - angle_b) * fruit_Z / math.sin(angle_b - angle_a)
        y = c * math.sin(angle_a)
        move = int(y + fruit_Z)
        # deep = int(abs(c * math.cos(angle_a)))
        deep = int(f * move / (dis * px))

    elif dis < h:
        dy = dis + h / 2
        angle_a = math.atan(dy * px / f)
        angle_b = math.atan(dis * px / f)
        c = math.sin(pi / 2 - angle_a) * fruit_Z / math.sin(angle_a - angle_b)
        y = c * math.sin(angle_b)
        move = y
        # deep = int(abs(c * math.cos(angle_b)))
        deep = int(f * move / (dis * px))

    else:
        move = fruit_Z
        deep = fruit_Z * f / (dis * px)

    return move, deep / 2


def measure_dis(x_distance, y_distance, w, h, mode="fruit"):

    if mode == "fruit":
        fruit_w = 40
        fruit_h = 36
    elif mode == "vegetable":
        fruit_w = 38
        fruit_h = 38
    else:
        fruit_w = 40
        fruit_h = 35
    x_move, x_deep = forecast_dis(x_distance, w, fruit_w)
    y_move, y_deep = forecast_dis(y_distance, h, fruit_h)
    if x_distance > 0:  # 往右走为正
        x_move = x_move + 2
    elif x_distance < 0:
        x_move = -x_move - 2
    else:
        x_move = 0
    if y_distance > 0:  # 往上动为正
        y_move = y_move
    elif y_distance < 0:
        y_move = -y_move
    else:
        y_move = 0

    # if x_distance > 0:
    #     print(f"向右移动{x_move}，x方向深度为{x_deep}")
    # elif x_distance < 0:
    #     print(f"向左移动{x_move}，x方向深度为{x_deep}")
    # else:
    #     print(f"在正中，x方向深度为{x_deep}")
    # if y_distance > 0:
    #     print(f"向下移动{y_move}，y方向深度为{y_deep}")
    # elif y_distance < 0:
    #     print(f"向上移动{y_move}，y方向深度为{y_deep}")
    # else:
    #     print(f"在正中，y方向深度为{y_deep}")
    # print("deep:", (x_deep + y_deep) / 2)

    return x_move, y_move, ((x_deep + y_deep) / 2 )


class Camera:
    def __init__(self, video_ser, img_receiver, executor, system_type="Linux"):
        self.cap = None
        self.video_ser = video_ser
        self.system_type = system_type
        # self.init_camera()
        self.frame = None
        self.tickle = 40
        self.is_stop = False
        self.W = 640
        self.H = 480
        self.count = 0
        # self.fruit_color = ([160, 108, 110], [255, 230, 255])
        # self.vegetable_color = ([0, 167, 152], [141, 255, 255])
        self.apple_red_color = ([151, 32, 101], [185, 196, 255])
        self.apple_green_color = ([62, 42, 94], [92, 132, 191])
        # self.fruit_red_color = ([148, 107, 121], [180, 216, 255])   # 之前调试
        self.fruit_red_color = ([18, 0, 137], [120, 90, 255])
        self.fruit_green_color = ([62, 42, 94], [100, 132, 191])
        # self.vegetable_red_color = ([0, 66, 167], [18, 238, 255])     #
        self.vegetable_red_color = ([0, 172, 203], [32, 255, 255])

        self.vegetable_green_color = ([42, 77, 0], [255, 162, 145])
        self.color_dic = {
            "fruit_red": self.fruit_red_color,
            "fruit_green": self.fruit_green_color,
            "vegetable_red": self.vegetable_red_color,
            "vegetable_green": self.vegetable_green_color,
        }
        self.img_reveiver = img_receiver
        self.executor = executor
        self.executor.add_node(self.img_reveiver)
        # self.lower_color = np.array([160, 108, 110])
        # self.upper_color = np.array([255, 230, 255])
        self.apple_model = apple_model
        self.fruit_vegetable_model = fruit_vegetable_model

    def init_camera(self):
        if self.system_type == "Windows":
            self.cap = cv2.VideoCapture(self.video_ser)
        else:
            part = "/dev/"
            self.cap = cv2.VideoCapture(part + self.video_ser)

    def start(self):
        t = threading.Thread(target=self.read, daemon=True)
        t.start()

    def read(self):
        while not self.is_stop:
            ret, frame = self.cap.read()
            if ret:
                self.frame = frame
            time.sleep(1 / self.tickle)
        self.cap.release()

    def stop(self):
        self.is_stop = True

    def get_frame(self):
        frame = self.img_reveiver.getImage(self.executor)
        if frame is not None:
            self.W = frame.shape[1]
            self.H = frame.shape[0]
            return frame
        else:
            self.count += 1
            print("未接收到图像")
            if self.count > 5:
                self.count = 0
                self.reboot_camera()
            return None

    def reboot_camera(self):
        camera_thread = threading.Thread(target=self.restart)
        camera_thread.start()

    def restart(self):
        print("重新开启摄像头程序")
        restart_camera_cmd = "python3 /home/dianfei/catch_robot_v2/check_camera.py"
        os.system(restart_camera_cmd)

    def change_exposure_time(self, exposure_time):
        cmd = f"v4l2-ctl --device={self.video_ser} --set-ctrl=exposure_time_absolute={exposure_time}"
        os.system(cmd)

    def change_auto_exposure(self, is_auto_exposure):
        if is_auto_exposure:
            cmd = f"v4l2-ctl --device={self.video_ser} --set-ctrl=auto_exposure=3"
        else:
            cmd = f"v4l2-ctl --device={self.video_ser} --set-ctrl=auto_exposure=1"
        os.system(cmd)

    def remove_extremes_and_average(self, data):
        if len(data) <= 2:
            # raise ValueError("List must contain more than two elements to remove extremes and calculate an average.")
            trimmed_data = data
        else:
            # 排序并去除最大值和最小值
            sorted_data = sorted(data)
            trimmed_data = sorted_data[1:-1]

        # 计算平均值
        average_value = sum(trimmed_data) / len(trimmed_data)
        return average_value

    def deal_img(self, img, mode=None):
        hsv_image = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        # hsv_image = cv2.erode(hsv_image, np.ones((3, 3), np.uint8), iterations=1)
        # hsv_image = cv2.dilate(hsv_image, np.ones((3, 3), np.uint8), iterations=1)

        # 模糊处理
        hsv_image = cv2.blur(hsv_image, (3, 3))
        biggest_area = 0
        index = None
        goal_contour = None
        final_color = None
        final_list = []

        if mode == "fruit":
            color_dic = {"red": self.fruit_red_color, "green": self.fruit_green_color}
        elif mode == "vegetable":
            color_dic = {
                "red": self.vegetable_red_color,
                "green": self.vegetable_green_color,
            }
        elif mode == "apple":
            color_dic = {"red": self.apple_red_color, "green": self.apple_green_color}
        else:
            color_dic = self.color_dic

        for color, [lower, upper] in color_dic.items():
            lower = np.array(lower)
            upper = np.array(upper)
            mask = cv2.inRange(hsv_image, lower, upper)
            # png_num = len(os.listdir("/home/dianfei/catch_robot_v2/catch_picture"))
            # path1 = "/home/dianfei/catch_robot_v2/catch_picture/{}.png".format(png_num)
            # cv2.imwrite(path1, mask)
            # path = "/home/dianfei/catch_robot_v2/catch_picture/{}_{}.png".format(mode, png_num)
            # cv2.imwrite(path, img)

            # 寻找轮廓
            contours, _ = cv2.findContours(
                mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
            )

            if contours:
                for i, contour in enumerate(contours):  # 筛选出目标色块
                    area = cv2.contourArea(contour)
                    if area > 4200:
                        final_list.append(
                            {"area": area, "contour": contour, "color": color}
                        )

        if final_list:
            final_list.sort(
                key=lambda x: x["area"], reverse=True
            )  # 将大于一定面积的轮廓保留至列表，并以面积大小排序
            # 创建一个空白图像
            extracted_image = np.zeros(img.shape[:2], dtype="uint8")
            for i in final_list:
                contour = i["contour"]

                cv2.drawContours(extracted_image, [contour], -1, (255, 255, 255), -1)
            extracted_image = cv2.erode(extracted_image, np.ones((3, 3)), iterations=1)
            extracted_image = cv2.dilate(extracted_image, np.ones((3, 3)), iterations=1)

            # cv2.write(mask_path, mask)
            return extracted_image, final_list
        else:
            return None, None

    def detect_(self, img, obj_contour, color, area, mode):
        """
        :param img: 原图，传入目的是为了标记
        :param obj_contour: 轮廓
        :param color: 轮廓对应的颜色
        :param mode: 表示是蔬菜还是水果，用来选择实物的现实大小
        :return: 字典，若是成熟的，字典里有颜色，距离等信息，否则只有颜色，颜色要是False代表出错
        """
        if obj_contour is not None:
            x, y, w, h = cv2.boundingRect(obj_contour)
            cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 2)
            # 写上颜色标签
            cv2.putText(
                img,
                f"{color}",
                (int(x), int(y + 10)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.60,
                (0, 0, 255),
                thickness=1,
            )
            # mask_path = "/home/dianfei/catch_image/{}_{}_mask.png".format(mode, png_num)

            if w < 20 or h < 20:
                return {
                    "color": False,  # 代表图像读取有问题
                }

            cx = x + w / 2
            cy = y + h / 2
            # print(f'X:{X}, Y:{Y}, Z:{Z}, angle:{angle}')
            x_distance = cx - self.W / 2
            y_distance = cy - self.H / 2
            # print(f'x_distance:{x_distance}, y_distance:{y_distance},  w:{w}, h:{h}')
            x_move, y_move, deep = measure_dis(x_distance, y_distance, w, h, mode=mode)
            # color = "red"
            return {
                "color": color,
                "x_move": x_move,
                "y_move": y_move,
                "deep": deep,
                "x_distance": x_distance,
                "y_distance": y_distance,
                "w": w,
                "h": h,
                "cx": cx,
                "cy": cy,
                "area": area,
            }
        else:
            return {
                "color": None,
            }

        # print(color_list)
        # if len(color_list) >= (num-1):
        #     # 找到出现次数最多的元素
        #     counter = Counter(color_list)
        #     color, most_common_count = counter.most_common(1)[0]
        #     print(f"出现次数最多的颜色是: {color}，出现次数为: {most_common_count}")
        #     if "red" in color:
        #         x_move_list = []
        #         y_move_list = []
        #         deep_list = []
        #         x_distance_list = []
        #         y_distance_list = []
        #         w_list = []
        #         h_list = []
        #         for result in result_list:
        #             if result["color"] == color:
        #                 x_move_list.append(result["x_move"])
        #                 y_move_list.append(result["y_move"])
        #                 deep_list.append(result["deep"])
        #                 x_distance_list.append(result["x_distance"])
        #                 y_distance_list.append(result["y_distance"])
        #                 w_list.append(result["w"])
        #                 h_list.append(result["h"])
        #
        #         # 去除最大值和最小值后求平均
        #         x_move = self.remove_extremes_and_average(x_move_list)
        #         y_move = self.remove_extremes_and_average(y_move_list)
        #         deep = self.remove_extremes_and_average(deep_list)
        #         x_distance = self.remove_extremes_and_average(x_distance_list)
        #         y_distance = self.remove_extremes_and_average(y_distance_list)
        #         w = self.remove_extremes_and_average(w_list)
        #         h = self.remove_extremes_and_average(h_list)
        #
        #         return {
        #             "x_move": x_move,       # 测的实际距离
        #             "y_move": y_move,
        #             "deep": deep,
        #             "color": color,
        #             "x_distance": x_distance,       # 图片上像素点的距离
        #             "y_distance": y_distance,
        #             "w": w,
        #             "h": h,
        #         }
        #     elif "green" in color:
        #         return {
        #             "color": color,
        #         }
        #     else:
        #         return {
        #             "color": None,
        #         }
        # else:
        #     return {
        #         "color": False,     # 代表图像读取有问题
        #     }
        # cv2.putText(frame, f"FPS: {fps}", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 0, 255), 2)
        #
        # cv2.imshow(f"{self.video_ser}", frame)
        #
        # k = cv2.waitKey(1) & 0xFF
        # # time.sleep(0.1)
        # if k == 27:
        #     break

    def detect(self, conf=0.8, is_show=False, limit_area=3000, mode="fruit"):
        """比赛时改成模型识别，颜色识别在下方"""
        frame = self.get_frame()
        results = self.fruit_vegetable_model(frame, conf=conf)
        current_time = time.strftime("%m-%d %H:%M:%S", time.localtime())

        path = f"/home/dianfei/catch_robot_v2/picture/model_detect_fruit_{current_time}.jpg"
        cv2.imwrite(path, frame)
        item_list = []
        itemlist = results[0].cpu().numpy().boxes
        if len(itemlist):
            for item in itemlist:
                xywh = item.xywhn[0].tolist()
                cls = item.cls[0]
                name = results[0].names[cls]
                conf = item.conf[0]
                cx = xywh[0] * frame.shape[1]
                cy = xywh[1] * frame.shape[0]
                w = xywh[2] * frame.shape[1]
                h = xywh[3] * frame.shape[0]
                # print(xywh)
                x1 = cx - 0.5 * w
                y1 = cy - 0.5 * h
                x2 = cx + 0.5 * w
                y2 = cy + 0.5 * h
                area = int(w * h)
                # print(f'x1:{x1},y1:{y1},x2:{x2},y2:{y2}')
                cv2.rectangle(
                    frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 0, 255), 2
                )
                # print(f'{name}:{int(conf * 100)}')
                if area < limit_area:
                    cv2.putText(
                        frame,
                        "remove",
                        (int(x1), int(y1 + 10)),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.60,
                        (0, 0, 255),
                        thickness=1,
                    )
                    cv2.putText(
                        frame,
                        f"area: {area}",
                        (int(x1), int(cy)),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.60,
                        (0, 0, 255),
                        thickness=1,
                    )
                    continue
                cv2.putText(
                    frame,
                    f"{name}:{int(conf * 100)}",
                    (int(x1), int(y1 + 10)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.60,
                    (0, 0, 255),
                    thickness=1,
                )
                cv2.putText(
                    frame,
                    f"area: {area}",
                    (int(x1), int(cy)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.60,
                    (0, 0, 255),
                    thickness=1,
                )
                x_distance = cx - self.W / 2
                y_distance = cy - self.H / 2
                # print(f'x_distance:{x_distance}, y_distance:{y_distance},  w:{w}, h:{h}')
                if name == "red_fruit":
                    target = "fruit"
                else:
                    target = "vegetable"
                x_move, y_move, deep = measure_dis(
                    x_distance, y_distance, w, h, mode=target
                )
                result = {
                    "color": name,
                    "x_move": x_move,
                    "y_move": y_move,
                    "deep": deep,
                    "x_distance": x_distance,
                    "y_distance": y_distance,
                    "w": w,
                    "h": h,
                    "cx": cx,
                    "cy": cy,
                    "conf": float(conf),
                    "area": area,
                }
                item_list.append(result)
            if is_show:
                cv2.imshow("image", frame)
                cv2.waitKey(0)

            if item_list:
                if len(itemlist) > 2:
                    item_list.sort(key=lambda x: x["conf"], reverse=True)
                    item_list = item_list[:2]

                if mode == "vegetable":
                    item_list.sort(key=lambda x: x["cy"], reverse=True)
                else:
                    item_list.sort(key=lambda x: x["area"], reverse=True)
                return item_list

            else:
                return [{"color": None}]
        # cv2.imshow("image", frame)
        else:
            return [{"color": None}]

    def model_detect(self, conf=0.9, is_show=False, limit_area=6500):
        frame = self.get_frame()
        results = self.apple_model(frame, conf=conf)
        current_time = time.strftime("%m-%d %H:%M:%S", time.localtime())

        path = f"/home/dianfei/catch_robot_v2/picture/model_detect_apple_{current_time}.jpg"
        cv2.imwrite(path, frame)
        item_list = []
        itemlist = results[0].cpu().numpy().boxes
        if len(itemlist):
            for item in itemlist:
                xywh = item.xywhn[0].tolist()
                cls = item.cls[0]
                name = results[0].names[cls]
                conf = item.conf[0]
                cx = xywh[0] * frame.shape[1]
                cy = xywh[1] * frame.shape[0]
                w = xywh[2] * frame.shape[1]
                h = xywh[3] * frame.shape[0]
                # print(xywh)
                x1 = cx - 0.5 * w
                y1 = cy - 0.5 * h
                x2 = cx + 0.5 * w
                y2 = cy + 0.5 * h
                area = int(w * h)
                # print(f'x1:{x1},y1:{y1},x2:{x2},y2:{y2}')
                cv2.rectangle(
                    frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 0, 255), 2
                )
                # print(f'{name}:{int(conf * 100)}')
                if area < limit_area:
                    cv2.putText(
                        frame,
                        "remove",
                        (int(x1), int(y1 + 10)),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.60,
                        (0, 0, 255),
                        thickness=1,
                    )
                    cv2.putText(
                        frame,
                        f"area: {area}",
                        (int(x1), int(cy)),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.60,
                        (0, 0, 255),
                        thickness=1,
                    )
                    continue
                cv2.putText(
                    frame,
                    f"{name}:{int(conf * 100)}",
                    (int(x1), int(y1 + 10)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.60,
                    (0, 0, 255),
                    thickness=1,
                )
                cv2.putText(
                    frame,
                    f"area: {area}",
                    (int(x1), int(cy)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.60,
                    (0, 0, 255),
                    thickness=1,
                )
                x_distance = cx - self.W / 2
                y_distance = cy - self.H / 2
                # print(f'x_distance:{x_distance}, y_distance:{y_distance},  w:{w}, h:{h}')
                x_move, y_move, deep = measure_dis(
                    x_distance, y_distance, w, h, mode="apple"
                )
                result = {
                    "color": name,
                    "x_move": x_move,
                    "y_move": y_move,
                    "deep": deep,
                    "x_distance": x_distance,
                    "y_distance": y_distance,
                    "w": w,
                    "h": h,
                    "cx": cx,
                    "cy": cy,
                    "conf": float(conf),
                    "area": area,
                }
                item_list.append(result)
            if is_show:
                cv2.imshow("image", frame)
                cv2.waitKey(0)

            if item_list:
                return frame, item_list
            else:
                return frame, [{"color": "None"}]
        # cv2.imshow("image", frame)
        else:
            return frame, [{"color": "None"}]

    # camera = Camera("/dev/video_left", system_type="Linux")
    def detect_(self, mode, num=5, is_show=False):
        color_list = []
        result_list = []
        for i in range(num):
            img = self.get_frame()
            if img is not None:
                frame, obj_contour, color = self.deal_img(img, mode)
                if obj_contour is not None:
                    x, y, w, h = cv2.boundingRect(obj_contour)
                    cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 2)

                    # mask_path = "/home/dianfei/catch_image/{}_{}_mask.png".format(mode, png_num)
                    if is_show:
                        cv2.imshow("frame", frame)
                        cv2.imshow("img", img)
                        cv2.waitKey(0)

                    if w < 20 or h < 20:
                        continue

                    cx = x + w / 2
                    cy = y + h / 2
                    # print(f'X:{X}, Y:{Y}, Z:{Z}, angle:{angle}')
                    x_distance = cx - self.W / 2
                    y_distance = cy - self.H / 2
                    # print(f'x_distance:{x_distance}, y_distance:{y_distance},  w:{w}, h:{h}')
                    x_move, y_move, deep = measure_dis(
                        x_distance, y_distance, w, h, mode=mode
                    )
                    # color = "red"
                    color_list.append(color)
                    result_list.append(
                        {
                            "x_move": x_move,
                            "y_move": y_move,
                            "deep": deep,
                            "color": color,
                            "x_distance": x_distance,
                            "y_distance": y_distance,
                            "w": w,
                            "h": h,
                        }
                    )
                else:
                    color_list.append(None)

        if len(color_list) >= (num - 1):
            # 找到出现次数最多的元素
            counter = Counter(color_list)
            color, most_common_count = counter.most_common(1)[0]
            print(f"出现次数最多的颜色是: {color}，出现次数为: {most_common_count}")
            if "red" in color:
                x_move_list = []
                y_move_list = []
                deep_list = []
                x_distance_list = []
                y_distance_list = []
                w_list = []
                h_list = []
                for result in result_list:
                    if result["color"] == color:
                        x_move_list.append(result["x_move"])
                        y_move_list.append(result["y_move"])
                        deep_list.append(result["deep"])
                        x_distance_list.append(result["x_distance"])
                        y_distance_list.append(result["y_distance"])
                        w_list.append(result["w"])
                        h_list.append(result["h"])

                # 去除最大值和最小值后求平均
                x_move = self.remove_extremes_and_average(x_move_list)
                y_move = self.remove_extremes_and_average(y_move_list)
                deep = self.remove_extremes_and_average(deep_list)
                x_distance = self.remove_extremes_and_average(x_distance_list)
                y_distance = self.remove_extremes_and_average(y_distance_list)
                w = self.remove_extremes_and_average(w_list)
                h = self.remove_extremes_and_average(h_list)

                return {
                    "x_move": x_move,  # 测的实际距离
                    "y_move": y_move,
                    "deep": deep,
                    "color": color,
                    "x_distance": x_distance,  # 图片上像素点的距离
                    "y_distance": y_distance,
                    "w": w,
                    "h": h,
                }
            elif "green" in color:
                return {
                    "color": color,
                }
            else:
                return {
                    "color": None,
                }
        else:
            return {
                "color": False,  # 代表图像读取有问题
            }
        # cv2.putText(frame, f"FPS: {fps}", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 0, 255), 2)
        #
        # cv2.imshow(f"{self.video_ser}", frame)
        #
        # k = cv2.waitKey(1) & 0xFF
        # # time.sleep(0.1)
        # if k == 27:
        #     break

    def color_detect(self, mode=None, num=3, is_show=False):
        results_list = []
        for i in range(num):
            img = self.get_frame()
            if img is None:
                for _ in range(5):
                    img = self.get_frame()

            if img is not None:
                frame, results = self.deal_img(img, mode)
                result_list = []  # 一张图片结果存在一个列表，一个轮廓为一个字典
                if results is not None:
                    for result in results:
                        contour = result["contour"]
                        color = result["color"]
                        area = result["area"]
                        print(area)
                        detected_result = self.detect_(
                            img, contour, color, area=area, mode=mode
                        )
                        result_list.append(detected_result)
                    results_list.append(result_list)

                else:
                    print("未识别出东西")
                    results_list.append([{"color": None}])
                if is_show:
                    cv2.imshow("img", img)
                    if results is not None:
                        cv2.imshow("frame", frame)
                    cv2.waitKey(0)
            if i == num - 1:
                current_time = time.strftime("%m-%d %H:%M:%S", time.localtime())
                local = self.video_ser.split("/")[-1]
                path = f"/home/dianfei/catch_robot_v2/picture/{local}_detect_apple_{current_time}.jpg"
                if img is not None:
                    cv2.imwrite(path, img)
                # print(f"保存颜色识别的最后一张图片成功, 路径为：{path}")
        count = 0
        for results in results_list:
            if results[0]["color"] == None:
                count += 1
        if count >= num - 1:
            return [{"color": None}]

        merge_list = []
        if len(results_list) >= 2:
            for result in results_list[0]:
                # result 为第一张图片结果列表里的其中一个字典
                is_save = False  # 保证同一个结果只有一次保存进列表
                for other_results in results_list[1:]:
                    # other_results 为其他图片的结果列表
                    for other_result in other_results:
                        # other_result 为结果中的一个字典
                        if result["color"] == other_result["color"]:
                            if (
                                abs(result["cx"] - other_result["cx"]) < 15
                                and abs(result["cy"] - other_result["cy"]) < 15
                            ):
                                if not is_save:
                                    is_save = True
                                    merge_list.append(
                                        {
                                            "color": result["color"],
                                            "x_move": (
                                                result["x_move"]
                                                + other_result["x_move"]
                                            )
                                            / 2,
                                            "y_move": (
                                                result["y_move"]
                                                + other_result["y_move"]
                                            )
                                            / 2,
                                            "deep": (
                                                result["deep"] + other_result["deep"]
                                            )
                                            / 2,
                                            "x_distance": (
                                                result["x_distance"]
                                                + other_result["x_distance"]
                                            )
                                            / 2,
                                            "y_distance": (
                                                result["y_distance"]
                                                + other_result["y_distance"]
                                            )
                                            / 2,
                                            "w": (result["w"] + other_result["w"]) / 2,
                                            "h": (result["h"] + other_result["h"]) / 2,
                                            "cx": (result["cx"] + other_result["cx"])
                                            / 2,
                                            "cy": (result["cy"] + other_result["cy"])
                                            / 2,
                                            "area": (
                                                result["area"] + other_result["area"]
                                            )
                                            / 2,
                                        }
                                    )
                                else:
                                    merge_list[-1] = {
                                        "color": result["color"],
                                        "x_move": (
                                            merge_list[-1]["x_move"]
                                            + other_result["x_move"]
                                        )
                                        / 2,
                                        "y_move": (
                                            merge_list[-1]["y_move"]
                                            + other_result["y_move"]
                                        )
                                        / 2,
                                        "deep": (
                                            merge_list[-1]["deep"]
                                            + other_result["deep"]
                                        )
                                        / 2,
                                        "x_distance": (
                                            merge_list[-1]["x_distance"]
                                            + other_result["x_distance"]
                                        )
                                        / 2,
                                        "y_distance": (
                                            merge_list[-1]["y_distance"]
                                            + other_result["y_distance"]
                                        )
                                        / 2,
                                        "w": (merge_list[-1]["w"] + other_result["w"])
                                        / 2,
                                        "h": (merge_list[-1]["h"] + other_result["h"])
                                        / 2,
                                        "cx": (
                                            merge_list[-1]["cx"] + other_result["cx"]
                                        )
                                        / 2,
                                        "cy": (
                                            merge_list[-1]["cy"] + other_result["cy"]
                                        )
                                        / 2,
                                        "area": (
                                            merge_list[-1]["area"]
                                            + other_result["area"]
                                        )
                                        / 2,
                                    }

        else:
            merge_list = results_list
        if merge_list and merge_list[0]["color"] is not None:
            if mode == "vegetable":
                merge_list.sort(key=lambda x: x["cy"], reverse=True)
            else:
                merge_list.sort(key=lambda x: x["area"], reverse=True)

        else:
            merge_list = [{"color": None}]

        return merge_list
