import os
import re
import threading
import time


def kill(goal="usb_cam_node_exe"):
    cmd = f"ps aux | grep {goal}"
    output = os.popen(cmd).readlines()
    origin = goal
    for i in output:
        i = i.strip()
        if origin in i:
            result = re.search(r"\b\d+\b", i)[0]
            kill_cmd = f"kill -9 {result}"
            print(kill_cmd, i)
            os.system(kill_cmd)


# kill()
kill("open_camera")

os.system(
    "python3 /home/dianfei/catch_robot_v2/open_camera.py --right_bright 200 --left_bright 200 --is_auto_bright True"
)
