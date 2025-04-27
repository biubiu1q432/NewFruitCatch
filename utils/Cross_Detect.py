import cv2
import time
import numpy as np
import math
# import skimage.morphology as morphology

class Cross_Detect:
    def __init__(self):
        self.kernel = np.ones((2, 2), np.uint8)
        np.bool = bool
        Bl = 0
        Gl = 73
        Rl = 0
        Bh = 255
        Gh = 255
        Rh = 255
        # Bl = 0
        # Gl = 2
        # Rl = 129
        # Bh = 255
        # Gh = 56
        # Rh = 255
        self.lower_color = np.array([Bl, Gl, Rl])
        self.upper_color = np.array([Bh, Gh, Rh])

    def points_to_hough(self, x1, y1, x2, y2):
        if x1 == x2:  # 垂直线
            theta = np.pi / 2
        else:
            theta = np.arctan2((y2 - y1), (x2 - x1))

        rho = x1 * np.cos(theta) + y1 * np.sin(theta)

        if rho < 0:
            rho = -rho
            theta += np.pi

        return rho, theta

    def hough_to_points(self, rho, theta):

        a = np.cos(theta)
        b = np.sin(theta)
        x0 = a * rho
        y0 = b * rho
        x1 = int(x0 + 1000 * (-b))
        y1 = int(y0 + 1000 * (a))
        x2 = int(x0 - 1000 * (-b))
        y2 = int(y0 - 1000 * (a))

        return x1, y1, x2, y2

    def get_cross_line_list(self, line_list):
        # 获得具有交点的直线组（角度差接近90°）
        cross_line_list = []
        temp = line_list.copy()
        for i in range(len(line_list)):
            line = temp.pop(0)
            if i != len(line_list) - 1:
                for j in line_list[i + 1:]:
                    result = self.cross_point(line, j, return_point=False)
                    if result is not None:
                        # cv2.circle(img1, (int(x), int(y)), 5, (255, 0, 0), 2)
                        cross_line_list.append((line, j))
        return cross_line_list

    def exclude_close_line(self, frame, lines, rho_threshold=30, theta_threshold=np.pi / 180 * 5):
        if lines is None:
            return []
        filtered_lines = []
        for line in lines:
            rho, theta = line[0]
            add_line = True
            for existing_line in filtered_lines:
                rho_existing, theta_existing = existing_line
                if abs(rho - rho_existing) < rho_threshold and abs(theta - theta_existing) < theta_threshold:
                    add_line = False
                    break
            if add_line:
                filtered_lines.append((rho, theta))
                # self.rho_theta_draw(frame, rho, theta)


        if len(filtered_lines) > 4:
            filtered_lines = filtered_lines[:4]

        for i in range(len(filtered_lines)):
            self.rho_theta_draw(frame, filtered_lines[i][0], filtered_lines[i][1])


        return filtered_lines

    def lines_angle(self, x1, y1, x2, y2, x3, y3, x4, y4):
        """计算两条直线的夹角"""
        # 定义两条直线上的点
        point1_line1 = np.array([x1, y1])  # 第一条直线上的点1
        point2_line1 = np.array([x2, y2])  # 第一条直线上的点2
        point1_line2 = np.array([x3, y3])  # 第二条直线上的点1
        point2_line2 = np.array([x4, y4])  # 第二条直线上的点2

        # 计算两条直线的向量
        vector_line1 = point2_line1 - point1_line1
        vector_line2 = point2_line2 - point1_line2

        # 计算向量的点积
        dot_product = np.dot(vector_line1, vector_line2)

        # 计算向量的长度
        length_line1 = np.linalg.norm(vector_line1)
        length_line2 = np.linalg.norm(vector_line2)

        # 计算夹角的余弦值
        cosine_angle = dot_product / (length_line1 * length_line2)

        # 计算夹角的度数（弧度转换为度）
        angle_in_degrees = np.degrees(np.arccos(np.clip(cosine_angle, -1, 1)))

        # 夹角取小的那一个
        if angle_in_degrees > 90:
            angle_in_degrees = 180 - angle_in_degrees

        return angle_in_degrees

    def cross_point(self, line1, line2, return_point=True):  # 计算交点函数
        # 是否存在交点（两条直线不接近平行的角点）
        x1, y1, x2, y2 = self.hough_to_points(line1[0], line1[1])
        x3, y3, x4, y4 = self.hough_to_points(line2[0], line2[1])
        point_is_exist = False
        x = 0
        y = 0

        angle = self.lines_angle(x1, y1, x2, y2, x3, y3, x4, y4)
        if return_point:
            if angle < 82:
                return False, (0, 0)
            else:
                if (x2 - x1) == 0:
                    k1 = None
                else:
                    k1 = (y2 - y1) * 1.0 / (x2 - x1)  # 计算k1,由于点均为整数，需要进行浮点数转化
                    b1 = y1 * 1.0 - x1 * k1 * 1.0  # 整型转浮点型是关键

                if (x4 - x3) == 0:  # L2直线斜率不存在操作
                    k2 = None
                    b2 = 0
                else:
                    k2 = (y4 - y3) * 1.0 / (x4 - x3)  # 斜率存在操作
                    b2 = y3 * 1.0 - x3 * k2 * 1.0

                if k1 is None:
                    if k2 is not None:
                        x = x1
                        y = k2 * x1 + b2
                        # point_is_exist=True
                elif k2 is None:
                    x = x3
                    y = k1 * x3 + b1
                elif not k2 == k1:
                    x = (b2 - b1) * 1.0 / (k1 - k2)
                    y = k1 * x * 1.0 + b1 * 1.0
                if (x > 0) & (y > 0):
                    point_is_exist = True

                return point_is_exist, (x, y)
        else:
            if angle < 82:
                return None
            else:
                return True

    def filter_close_points(self, points, threshold):
        """剔除距离过近的点"""
        filtered_points = []
        temp = points

        while True:
            if temp:
                del_point = temp.pop(0)
                filtered_points.append(del_point)
                temp_list = []
                for point in temp:
                    distance = math.sqrt((point[0] - del_point[0]) ** 2 + (point[1] - del_point[1]) ** 2)
                    if distance < threshold:
                        temp_list.append(point)

                for j in temp_list:
                    temp.remove(j)
            else:
                break

        return filtered_points

    def find_center_point(self, img, line_list):
        """找出中心点"""
        cross_point_list = []
        temp = line_list.copy()
        for i in range(len(line_list)):
            line = temp.pop(0)
            if i != len(line_list) - 1:
                for j in line_list[i + 1:]:
                    point_is_exist, (x, y) = self.cross_point(line, j)
                    if point_is_exist:
                        # cv2.circle(img1, (int(x), int(y)), 5, (255, 0, 0), 2)
                        cross_point_list.append([x, y])
        # print(cross_point_list, "111")
        point_list = self.filter_close_points(cross_point_list, 30)
        print("point_list:",point_list)

        if point_list:
            if len(point_list) < 4:
                return None
            center_x = 0
            center_y = 0
            for x, y in point_list:
                center_x += x
                center_y += y
                cv2.circle(img, (int(x), int(y)), 5, (255, 0, 0), 2)
                # cv2.putText(img1, "angle:{}".format(angle), (int(x), int(y)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)
            center_x /= len(point_list)
            center_y /= len(point_list)
            cv2.circle(img, (int(center_x), int(center_y)), 5, (0, 255, 0), 2)
            return (center_x, center_y)
        else:
            return None

    def get_cross_point(self, img, is_show=False, is_save=True, show_mode=0):
        """找出十字架中心点"""
        frame = img.copy()
        frame1 = frame.copy()
        lines = self.hough_line_transform(img, is_show=is_show, is_save=is_save, show_mode=show_mode)
        if lines is not None:
            lines = self.exclude_close_line(frame, lines)

            if len(lines) > 4:
                lines = lines[:4]
            elif len(lines) < 4:
                return frame, None
            center_point = self.find_center_point(frame, lines)
            if is_save:
                current_time = time.strftime("%m-%d-%H-%M-%S", time.localtime())
                cv2.imwrite(f"/home/dianfei/catch_robot_v2/picture/cross_point_{current_time}.jpg", frame)
                cv2.imwrite(f"/home/dianfei/catch_robot_v2/picture/origin_cross{current_time}.jpg", frame1)
            if is_show:
                cv2.imshow('frame', frame)
                cv2.waitKey(show_mode)
            if center_point is not None:
                return frame, center_point
            else:
                return frame, None

        # cv2.imshow("picture", picture)
        # # cv2.waitKey(0)
        # # cv2.destroyAllWindows()
        # clustered_center = self.find_clustered_center(point)

        else:
            return frame, None

    def non_max_suppression(self, lines, threshold):
        # 初始化结果列表
        result_lines = []

        if lines is not None:
            # 对霍夫直线检测结果按照峰值进行排序
            lines = lines.squeeze(axis=1)
            lines = lines[np.argsort(-lines[:, 1])]

            # 遍历直线
            for line in lines:
                # 判断当前直线是否与已选取的直线相近
                is_close = False
                for selected_line in result_lines:
                    if abs(selected_line[1] - line[1]) < threshold:
                        is_close = True
                        break

                # 如果不相近，则将该直线添加到结果列表中
                if not is_close:
                    result_lines.append(line)

        return result_lines

    def rho_theta_draw(self, src: np.ndarray, rho, theta, color=(0, 0, 255)):
        a = np.cos(theta)
        b = np.sin(theta)
        x0 = a * rho
        y0 = b * rho
        x1 = int(x0 + 1000 * (-b))
        y1 = int(y0 + 1000 * (a))
        x2 = int(x0 - 1000 * (-b))
        y2 = int(y0 - 1000 * (a))
        cv2.line(src, (x1, y1), (x2, y2), color, 1)  # 画直线，对象、起点、终点、颜色、线宽

    def hough_line_transform(self, frame, is_invert=True, is_show=False, is_save=False, show_mode=0):
        hsv_image = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        # 开运算
        hsv_image = cv2.erode(hsv_image, np.ones((3, 3), np.uint8), iterations=1)
        hsv_image = cv2.dilate(hsv_image, np.ones((3, 3), np.uint8), iterations=1)

        # 模糊处理
        hsv_image = cv2.blur(hsv_image, (3, 3))
        mask = cv2.inRange(hsv_image, self.lower_color, self.upper_color)
        if is_invert:
            mask = cv2.bitwise_not(mask)
        # if is_show:
            # cv2.imshow("mask", mask)
        # mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        # mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        # 寻找轮廓
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        biggest_area = 0
        index = None
        if contours:
            for i, contour in enumerate(contours):  # 筛选出目标色块
                area = cv2.contourArea(contour)
                if area > biggest_area and area > 2500:
                    biggest_area = area
                    index = i
            # 创建一个空白图像
            if not is_invert:
                extracted_image = np.zeros(frame.shape[:2], dtype="uint8")
            else:
                extracted_image = np.ones(frame.shape[:2], dtype="uint8")
            if index is not None:
                contour = [contours[index]]
                cv2.drawContours(extracted_image, contour, -1, (255, 255, 255), -1)
                if not is_invert:
                    extracted_image = cv2.dilate(extracted_image, np.ones((13, 13), np.uint8), iterations=1)
                    extracted_image = cv2.erode(extracted_image, np.ones((13, 13), np.uint8), iterations=1)
                else:
                    extracted_image = cv2.erode(extracted_image, np.ones((13, 13), np.uint8), iterations=1)
                    extracted_image = cv2.dilate(extracted_image, np.ones((13, 13), np.uint8), iterations=1)
                # extracted_image = cv2.dilate(extracted_image, np.ones((3, 3), np.uint8), iterations=3)
            edges = cv2.Canny(extracted_image, 50, 150, apertureSize=3)
            if is_save:
                current_time = time.strftime("%m-%d-%H-%M-%S", time.localtime())
                cv2.imwrite(f"/home/dianfei/catch_robot_v2/picture/cross_mask_{current_time}.jpg", mask)
                cv2.imwrite(f"/home/dianfei/catch_robot_v2/picture/cross_extracted_{current_time}.jpg", extracted_image)

            if is_show:
                cv2.imshow("mask", mask)
                cv2.imshow("extracted_image", extracted_image)

            lines = cv2.HoughLines(edges, 1, np.pi / 180, 50)  # 边缘检测之后霍夫变换得出直线
            return lines

        else:
            return None

    def get_rotate_angle(self, img):
        frame = img.copy()
        # img1 = img.copy()
        # img2 = img.copy()
        lines = self.hough_line_transform(img)
        # print(lines1)
        if lines is not None:
            # lines1 = lines[:, 0, :]
            # for rho, theta in lines1:
            #     self.rho_theta_draw(img1, rho, theta)
            # cv2.imshow("img1", img1)
            lines = self.exclude_close_line(frame, lines)   # rho
            if len(lines) > 4:
                return lines[:4]

            line_list = self.get_cross_line_list(lines)     #rho
            # print("line_list:", line_list, len(line_list))
            if line_list:
                if len(line_list) > 4:
                    line_list = line_list[:4]
                all_angle = 0
                length = len(line_list)
                # print("length:", length)
                for line1, line2 in line_list:
                    frame, angle = self.cal_angle(frame, line1, line2)
                    # print("angle:", angle)
                    if angle is not None:
                        all_angle += angle
                    else:
                        length -= 1
                return frame, all_angle/length
            else:
                return frame, None
        else:
            return frame, None

    def cal_angle(self, frame, line1, line2):
        rho1, theta1 = line1
        rho2, theta2 = line2
        # if 82 < abs(theta1 - theta2) * 180 / np.pi < 98:
        if rho1 < 0:
            temp1 = np.pi - theta1
            temp2 = theta2
        elif rho2 < 0:
            temp1 = theta1
            temp2 = np.pi - theta2
        else:
            temp1 = theta1
            temp2 = theta2

        if temp1 > temp2:
            self.rho_theta_draw(frame, rho1, theta1)
            self.rho_theta_draw(frame, rho2, theta2, color=(0, 255, 0))
            # print(f"line1: ({rho1}, {theta1 * 180 / np.pi}), line2（绿）: ({rho2}, {theta2 * 180 / np.pi})")
            if rho2 > 0:
                if theta2 < np.pi / 2:
                    temp2 = -temp2
            return frame, int(temp2 * 180 / np.pi)

        else:
            self.rho_theta_draw(frame, rho1, theta1, color=(0, 255, 0))
            self.rho_theta_draw(frame, rho2, theta2)
            # print(f"line1（绿）: ({rho1}, {theta1 * 180 / np.pi}), line2: ({rho2}, {theta2 * 180 / np.pi})")
            if rho1 > 0:
                if theta1 < np.pi / 2:
                    temp1 = -temp1

            return frame, int(temp1 * 180 / np.pi)