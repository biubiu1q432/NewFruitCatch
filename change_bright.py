import os
import argparse

parameter = argparse.ArgumentParser(description="亮度设置")
parameter.add_argument("--right_bright", type=int, default=None)
parameter.add_argument("--left_bright", type=int, default=None)

args = parameter.parse_args()
right_bright = args.right_bright
left_bright = args.left_bright
video_ser_dic = {"right": "/dev/video_right", "left": "/dev/video_left"}

for local, video_ser in video_ser_dic.items():
    if local == "right":
        bright = right_bright
    if local == "left":
        bright = left_bright
    else:
        bright = None
    if bright is not None:
        os.system(
            f"v4l2-ctl --device={video_ser} --set-ctrl=exposure_time_absolute={bright}"
        )
        print(f"将{video_ser}曝光时间设置为：", bright)
