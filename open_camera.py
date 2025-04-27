import os
import re
import threading
import time
import argparse

# v4l2-ctl --device=/dev/video_right --set-ctrl=exposure_time_absolute=250
# v4l2-ctl --device=/dev/video_left --set-ctrl=exposure_time_absolute=250


def open_v4l2(video_ser):
    # os.system(f'ros2 run usb_cam usb_cam_node_exe --ros-args --remap __ns:={video_ser} --remap image_raw:={video_ser} -p pixel_format:=mjpeg2rgb')
    ser = video_ser.split('/')[-1]
    os.system(f'ros2 run usb_cam usb_cam_node_exe --ros-args --remap image_raw:={video_ser} --params-file /home/dianfei/catch_robot/{ser}.yaml -p pixel_format:=mjpeg2rgb')
    # os.system(f'ros2 run usb_cam usb_cam_node_exe --ros-args --remap __ns:={video_ser} --remap image_raw:={video_ser}')
    # os.system(f'ros2 run v4l2_camera v4l2_camera_node --ros-args -p video_device:={video_ser} --remap image_raw:={video_ser}')


def set(video_ser, bright=180, is_auto_bright=False):
    time.sleep(2.5)
    os.system(f"v4l2-ctl --device={video_ser} --set-ctrl=white_balance_automatic=0")
    os.system(f"v4l2-ctl --device={video_ser} --set-ctrl=white_balance_temperature=4600")
    os.system(f"v4l2-ctl --device={video_ser} --set-ctrl=focus_automatic_continuous=0")
    os.system(f"v4l2-ctl --device={video_ser} --set-ctrl=focus_absolute=0")
    if not is_auto_bright:
        os.system(f"v4l2-ctl --device={video_ser} --set-ctrl=auto_exposure=1")
        os.system(f"v4l2-ctl --device={video_ser} --set-ctrl=exposure_time_absolute={bright}")      # 380 460 night 180 210 morning
    else:
        # 自动曝光时间调节
        os.system(f"v4l2-ctl --device={video_ser} --set-ctrl=auto_exposure=3")
    os.system(f"v4l2-ctl --device={video_ser} --list-ctrls")

def kill(goal="usb_cam_node_exe"):
    cmd = f"ps aux | grep {goal}"
    output = os.popen(cmd).readlines()
    origin = goal
    for i in output:
        i = i.strip()
        if origin in i:
            result = re.search(r"\b\d+\b", i)[0]
            kill_cmd = f"kill -9 {result}"
            print(kill_cmd)
            os.system(kill_cmd)
        print("杀死摄像头程序")


parameter = argparse.ArgumentParser(description="亮度设置")
parameter.add_argument("--is_auto_bright", type=bool, default=False)
parameter.add_argument("--right_bright", type=int, default=180)
parameter.add_argument("--left_bright", type=int, default=210)

args = parameter.parse_args()
is_auto_bright = args.is_auto_bright
right_bright = args.right_bright
left_bright = args.left_bright

kill()
# kill("open_camera")

thread_dic = {}

cmd = "ls /dev/video*"
output = os.popen(cmd).readlines()
video_list = ["/dev/video_right", "/dev/video_left"]
for i in output:
    i = i.strip()
    if i in video_list:
        thread_dic[i] = threading.Thread(target=open_v4l2, args=(i,))

os.system('echo 382009 | sudo -S chmod 777 /dev/video*')

for ser, thread in thread_dic.items():
    if ser == "/dev/video_right":
        bright = right_bright
    else:
        bright = left_bright
    os.system(f"v4l2-ctl --device={ser} --set-fmt-video=width=640,height=480,pixelformat=MJPG")
    thread.start()
    set(ser, bright, is_auto_bright=is_auto_bright)

# for ser, thread in thread_dic.items():
#     thread.join()

try:
    while True:
        time.sleep(10)
except KeyboardInterrupt as e:
    print("检测到CTRL+C，准备退出程序!")
    kill()