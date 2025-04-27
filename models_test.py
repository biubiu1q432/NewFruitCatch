import os, cv2
import time
from ultralytics import YOLO
import threading
def model_detect(model, frame):
    model(frame, conf=0.8)

model1 = YOLO("E:/common3/catch_robot_v2/model/apple_v11n4.onnx")
model2 = YOLO("E:/common3/catch_robot_v2/model/apple_v11n4.onnx")
path = "./model_test"
frame_list = os.listdir(path)
count = len(frame_list)
frame_list = [os.path.join(path, frame_path) for frame_path in frame_list]
start = 0
for i in range(10):
    start_time = time.time()
    frame = cv2.imread(frame_list[i])
    # results1 = model1(frame, conf=0.8)
    # results2 = model2(frame, conf=0.8)
    t1 = threading.Thread(target=model_detect, args=(model1, frame))
    t2 = threading.Thread(target=model_detect, args=(model2, frame))
    t1.start()
    t2.start()
    t1.join()
    t2.join()
    end_time = time.time()
    cost_time = end_time - start_time
    start += cost_time
    print(f"cost time:{cost_time}")
print(f"average time:{start / 10}")

# model1 = YOLO("E:/common3/catch_robot_v2/model/apple_v11n4.onnx")
# # model2 = YOLO("E:/common3/catch_robot_v2/model/apple_v11n4.onnx")
# path = "./model_test"
# frame_list = os.listdir(path)
# count = len(frame_list)
# frame_list = [os.path.join(path, frame_path) for frame_path in frame_list]
# start = 0
# for i in range(10):
#     start_time = time.time()
#     frame = cv2.imread(frame_list[i])
#     results1 = model1(frame, conf=0.8)
#     results2 = model2(frame, conf=0.8)
#     end_time = time.time()
#     cost_time = end_time - start_time
#     start += cost_time
#     print(f"cost time:{cost_time}")
# print(f"average time:{start / 10}")