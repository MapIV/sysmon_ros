#!/usr/bin/env python3
# -*- coding: utf-8 -*- 

import os
from pathlib import Path

import rospy
from std_msgs.msg import Float32


def read_gpuload(e):
    for gpu in gpu_loads:
        if os.access(gpu['path'], os.R_OK):
            with open(gpu['path'], 'r') as f:
                load_msg = Float32()
                load_msg.data = float(f.read()) / 10.0   # %
                gpu['pub'].publish(load_msg)
        else:
            rospy.logerr(f"Cannot read load! {gpu['path']}")


def main():
    global IGPU_PATH
    global CHECK_INTERVAL
    global gpu_loads

    rospy.init_node('jetson_gpumon', anonymous=False)

    IGPU_PATH      = Path(rospy.get_param('~igpu_path', ''))
    CHECK_INTERVAL = 5
    
    gpu_loads = []
    for item in IGPU_PATH.iterdir():
        print(item)
        if item not in ['gv11b', 'gp10b', 'ga10b', 'gpu']:
            continue

        gpu_load = item / "device/load"
        print(gpu_load)
        if not gpu_load.exists() and not gpu_load.is_file():
            continue

        topicname = item.name.replace('.', '_')
        gpu_pub   = rospy.Publisher(topicname, Float32, queue_size=10)
        gpu_loads.append({'path': gpu_load, 'pub': gpu_pub})

    if len(gpu_loads) == 0:
        rospy.logerr("GPU Device Not Found!")
        exit(1)

    rospy.Timer(rospy.Duration(CHECK_INTERVAL), read_gpuload)
    rospy.spin()