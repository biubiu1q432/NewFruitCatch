import time
from ultralytics import YOLO
from utils.Cross_Detect import *

# from utils.Camera import *
import cv2
import os
import threading

# cap = cv2.VideoCapture(1)
cross_detect = Cross_Detect()
path = "./cross_test"
frame_list = os.listdir(path)
count = len(frame_list)
frame_list = [os.path.join(path, frame_path) for frame_path in frame_list]


def detect_cross(frame, mode="left", is_show=False):
    point_list = []

    for i in range(4):
        if mode == "right":
            frame, point = cross_detect.get_cross_point(frame, is_show=is_show)

        else:

            frame, point = cross_detect.get_cross_point(frame, is_show=is_show)
        if point is not None:
            point_list.append(point)
    print(point_list)


for i in range(count):
    frame = cv2.imread(frame_list[i])
    cv2.imshow("origin", frame)
    # results = model(frame_list[i], conf=0.8)
    item_list = []
    img = cross_detect.get_cross_point(frame, is_show=True, is_save=False, show_mode=0)
# model = YOLO("./model/fruit_vegetable_v11n2.onnx")
# for i in range(count):
#     frame = cv2.imread(frame_list[i])
#
#     results = model(frame_list[i], conf=0.8)
#     item_list = []
#
#     itemlist = results[0].cpu().numpy().boxes
#     if len(itemlist):
#         for item in itemlist:
#             xywh = item.xywhn[0].tolist()
#             cls = item.cls[0]
#             name = results[0].names[cls]
#             conf = item.conf[0]
#             cx = xywh[0] * frame.shape[1]
#             cy = xywh[1] * frame.shape[0]
#             w = xywh[2] * frame.shape[1]
#             h = xywh[3] * frame.shape[0]
#             # print(xywh)
#             x1 = cx - 0.5 * w
#             y1 = cy - 0.5 * h
#             x2 = cx + 0.5 * w
#             y2 = cy + 0.5 * h
#             area = int(w * h)
#             # print(f'x1:{x1},y1:{y1},x2:{x2},y2:{y2}')
#             cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 0, 255), 2)
#             # print(f'{name}:{int(conf * 100)}')
#             limit_area = 3000
#             if area < limit_area:
#                 cv2.putText(frame,
#                             'remove', (int(x1), int(y1 + 10)),
#                             cv2.FONT_HERSHEY_SIMPLEX,
#                             0.60, (0, 0, 255),
#                             thickness=1)
#                 cv2.putText(frame,
#                             f'area: {area}', (int(x1), int(cy)),
#                             cv2.FONT_HERSHEY_SIMPLEX,
#                             0.60, (0, 0, 255),
#                             thickness=1)
#                 continue
#             cv2.putText(frame,
#                         f'{name}:{int(conf * 100)}', (int(x1), int(y1 + 10)),
#                         cv2.FONT_HERSHEY_SIMPLEX,
#                         0.60, (0, 0, 255),
#                         thickness=1)
#             cv2.putText(frame,
#                         f'area: {area}', (int(x1), int(cy)),
#                         cv2.FONT_HERSHEY_SIMPLEX,
#                         0.60, (0, 0, 255),
#                         thickness=1)
#
#             result = {
#                 "color": name,
#                 "w": w,
#                 "h": h,
#                 "cx": cx,
#                 "cy": cy,
#                 "conf": float(conf),
#                 "area": area,
#             }
#             print(result)
#             item_list.append(result)
#
#         cv2.imshow("image", frame)
#         cv2.waitKey(0)
# #

# while True:
#     time.sleep(0.05)
#     if count >= length-1:
#         count = 0
#     else:
#         count += 1
#     print(count, length)
#     frame = cv2.imread(frame_list[count])
#     # 获取滑动条值
#     # Bl = cv2.getTrackbarPos('Bl', WindowName)
#     # Gl = cv2.getTrackbarPos('Gl', WindowName)
#     # Rl = cv2.getTrackbarPos('Rl', WindowName)
#     # Bh = cv2.getTrackbarPos('Bh', WindowName)
#     # Gh = cv2.getTrackbarPos('Gh', WindowName)
#     # Rh = cv2.getTrackbarPos('Rh', WindowName)
#     # ite = cv2.getTrackbarPos('iterations', WindowName)
#     # cv2.imshow("image", frame)


# while True:
#     ret, frame = cap.read()
#     if not ret:
#         print("摄像头未打开")
#         break
#
#     cv2.imshow("frame", frame)
#
#     # img = cross_detect.get_cross_point(frame, is_show=True, is_save=False, show_mode=1)
#     key = cv2.waitKey(1) & 0xFF
#     if key == ord('s'):
#         cv2.imwrite(f"./picture/{count}.jpg", frame)
#         count += 1
#         print(f"{count}.jpg照片已保存")
#
#         # 如果按下ESC键，则关闭窗口并结束循环
#     elif key == 27:
#         print("退出")
#         break
#     time.sleep(0.05)
#
# cv2.destroyAllWindows()

# apple_red_color = ([151, 32, 101], [185, 196, 255])
# apple_green_color = ([62, 42, 94], [92, 132, 191])
# fruit_red_color = ([18, 0, 137], [120, 90, 255])
# fruit_green_color = ([62, 42, 94], [100, 132, 191])
# vegetable_red_color = ([0, 172, 203], [32, 255, 255])
# vegetable_green_color = ([42, 77, 0], [255, 162, 145])
# Bl = 0
# Gl = 0
# Rl = 0
# Bh = 8
# Gh = 3
# Rh = 255
# # Bl = 0
# # Gl = 2
# # Rl = 129
# # Bh = 255
# # Gh = 56
# # Rh = 255
# lower_color = ([Bl, Gl, Rl],[Bh, Gh, Rh])
# # cap = None
# start,end = 0.0,0.0
# def nothing(x):
#     pass
# WindowName = 'result'
# mode = "fruit"
# color = "red"
# if mode == "fruit":
#     if color == "red":
#         color_tuple = fruit_red_color
#     else:
#         color_tuple = fruit_green_color
# elif mode == "vegetable":
#     if color == "red":
#         color_tuple = vegetable_red_color
#     else:
#         color_tuple = vegetable_green_color
# else:
#     color_tuple = lower_color
# Bl = color_tuple[0][0]
# Gl = color_tuple[0][1]
# Rl = color_tuple[0][2]
# Bh = color_tuple[1][0]
# Gh = color_tuple[1][1]
# Rh = color_tuple[1][2]
# ite = None
# cv2.namedWindow(WindowName, cv2.WINDOW_KEEPRATIO)  # 建立空窗口
# cv2.createTrackbar('Bl', WindowName, Bl, 255, nothing)  # 创建滑动条
# cv2.createTrackbar('Gl', WindowName, Gl, 255, nothing)  # 创建滑动条
# cv2.createTrackbar('Rl', WindowName, Rl, 255, nothing)  # 创建滑动条
# cv2.createTrackbar('Bh', WindowName, Bh, 255, nothing)  # 创建滑动条
# cv2.createTrackbar('Gh', WindowName, Gh, 255, nothing)  # 创建滑动条
# cv2.createTrackbar('Rh', WindowName, Rh, 255, nothing)  # 创建滑动条
# cv2.createTrackbar('iterations', WindowName, 0, 20, nothing)  # 创建滑动条
#
# def get_value_threading():
#     global Bl, Gl, Rl, Bh, Gh, Rh, ite
#     while True:
#         Bl = cv2.getTrackbarPos('Bl', WindowName)
#         Gl = cv2.getTrackbarPos('Gl', WindowName)
#         Rl = cv2.getTrackbarPos('Rl', WindowName)
#         Bh = cv2.getTrackbarPos('Bh', WindowName)
#         Gh = cv2.getTrackbarPos('Gh', WindowName)
#         Rh = cv2.getTrackbarPos('Rh', WindowName)
#         ite = cv2.getTrackbarPos('iterations', WindowName)
#         time.sleep(0.1)
# #%%
# threading.Thread(target=get_value_threading, args=()).start()
# path = "./color_detect_pic"
# frame_list = os.listdir(path)
# frame_list = [os.path.join(path, frame_path) for frame_path in frame_list]
# length = len(frame_list)
# count = 0
# print(frame_list)
# while True:
#     time.sleep(0.05)
#     if count >= length-1:
#         count = 0
#     else:
#         count += 1
#     print(count, length)
#     frame = cv2.imread(frame_list[count])
#     # 获取滑动条值
#     # Bl = cv2.getTrackbarPos('Bl', WindowName)
#     # Gl = cv2.getTrackbarPos('Gl', WindowName)
#     # Rl = cv2.getTrackbarPos('Rl', WindowName)
#     # Bh = cv2.getTrackbarPos('Bh', WindowName)
#     # Gh = cv2.getTrackbarPos('Gh', WindowName)
#     # Rh = cv2.getTrackbarPos('Rh', WindowName)
#     # ite = cv2.getTrackbarPos('iterations', WindowName)
#     # cv2.imshow("image", frame)
#     #色域设置
#     hsv_image = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
#     cv2.namedWindow("hsv_image", cv2.WINDOW_KEEPRATIO)
#     cv2.namedWindow("mask", cv2.WINDOW_KEEPRATIO)
#
#     while True:
#         lower_color = np.array([Bl, Gl, Rl])
#         upper_color = np.array([Bh, Gh, Rh])
#         hsv_image =frame
#         #开运算
#         hsv_image = cv2.erode(hsv_image, np.ones((3,3),np.uint8), iterations=ite)
#         hsv_image = cv2.dilate(hsv_image, np.ones((3,3),np.uint8), iterations=ite)
#
#         #模糊处理
#         hsv_image = cv2.blur(hsv_image, (3, 3))
#
#         #二值转换，进行颜色分割---》把色域内的像素点设为白色，其余像素点设为黑色
#         mask = cv2.inRange(hsv_image, lower_color, upper_color)
#
#
#         #获取色块轮廓（cv2.findContours()函数返回的轮廓列表是按轮廓大小排序的）
#         contours,hierarchy= cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
#
#         biggest_area = 0
#         if contours :
#             for contour in contours:#筛选出目标色块
#                 x, y, w, h = cv2.boundingRect(contour)
#                 area = cv2.contourArea(contour)
#                 if area > biggest_area:
#                     biggest_area = area
#                 # print(str(biggest_area) + " _____ " + str(x + w / 2) + " _____ " + str(y + h / 2))
#                 #绘制矩形框
#                 # cv2.rectangle(hsv_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
#
#
#         cv2.imshow("hsv_image", hsv_image)
#         cv2.imshow("mask", mask)
#         key = cv2.waitKey(1) & 0xFF
#         # 按下 'q' 键退出循环
#         if key == 27:
#             break
#         time.sleep(0.1)
#     if key == 27:
#         continue
#
#
# cv2.destroyAllWindows()
