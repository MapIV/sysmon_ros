#!/usr/bin/env python3
# -*- coding: utf-8 -*- 

import os
from pathlib import Path

import rospy
from std_msgs.msg import Float32


def read_gpuload():
    for gpu in gpu_loads:
        if os.access(gpu['path'], os.R_OK):
            with open(gpu['path'], 'r') as f:
                load_msg = Float32()
                load_msg.data = float(f.read()) / 10.0   # %
                gpu['pub'].publish(load_msg)
        else:
            rospy.logerr(f"Cannot read load! {gpu['path']}")


if __name__ == '__main__':
    global gpu_loads

    rospy.init_node('jetson_gpumon', anonymous=False)

    HZ        = float(rospy.get_param('~hz', ''))
    IGPU_PATH = Path(rospy.get_param('~igpu_path', ''))
    
    jetson_gpu = ['gv11b', 'gp10b', 'ga10b', 'gpu']
    gpu_loads = []
    for item in IGPU_PATH.iterdir():
        matched = False
        for gpuname in jetson_gpu:
            if gpuname in str(item):
                matched = True
        if not matched:
            continue

        gpu_load = item / "device/load"
        if not gpu_load.exists() and not gpu_load.is_file():
            continue

        topicname = item.name.strip().replace('.', '_')
        gpu_pub   = rospy.Publisher(f"gpumon/{topicname}", Float32, queue_size=10)
        gpu_loads.append({'path': gpu_load, 'pub': gpu_pub})
        rospy.loginfo(f"GPU Added: {gpu_load}")

    if len(gpu_loads) == 0:
        rospy.logerr("GPU Device Not Found!")
        exit(1)

    r = rospy.Rate(HZ)
    while not rospy.is_shutdown():
        read_gpuload()
        r.sleep()