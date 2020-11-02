#!/usr/bin/env python
import os
import sys
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image as ROSImage
from ros_picam.srv import *
from time import sleep
from datetime import datetime

from PIL import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np


def get_time_string(include_date=False):
    dt = datetime.now()
    dt_string = ""
    date = "{:02}-{:02}-{:02}".format(dt.month,dt.day,dt.year-2000)
    time = "{:02}-{:02}-{:02}-{:06}".format(dt.hour,dt.minute,dt.second,dt.microsecond)
    if include_date:
        dt_string = "{}--{}".format(date,time)
    else:
        dt_string = time
    return dt_string


def callback(img):
    node = img.header.frame_id
    print("Image received from: {}".format(node))
    label = "taskboard"
    bridge = CvBridge()
    try:
        cv_image = bridge.imgmsg_to_cv2(img, "bgr8")
        filename = '{}-{}-{}.png'.format(node,label,get_time_string())
        image = Image.fromarray(cv_image)
        image.save(filename)
        print("Image saved to: {}".format(filename))
        cv2.imshow('test',cv_image)
    except CvBridgeError as e:
        print(e)


def picam_receiver():
    rospy.init_node('picam_receiver')
    #global SAVE_DIR
    SAVE_DIR = '/home/pgavriel/ros_ws/src/ros_picam/captures'
    os.chdir(SAVE_DIR)
    sub = rospy.Subscriber("picam_output",ROSImage,callback)
    rospy.loginfo("Subscriber up.")
    rospy.spin()
    pass

if __name__ == '__main__':

    picam_receiver()

    print("\n\nQuitting.")
