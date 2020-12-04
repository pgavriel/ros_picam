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
    rospy.loginfo("Image received from: {}".format(node))
    label = "taskboard"
    bridge = CvBridge()
    try:
        cv_image = bridge.imgmsg_to_cv2(img, "bgr8")
        filename = '{}-{}-{}.png'.format(node,label,get_time_string())
        image = Image.fromarray(cv_image)
        image.save(filename)
        rospy.loginfo("Image saved to: {}".format(filename))
        cv2.imshow('test',cv_image)
    except CvBridgeError as e:
        print(e)


def picam_receiver():
    rospy.init_node('picam_receiver')
    rospy.loginfo("Starting picam receiver...")

    save_dir = rospy.get_param("~save_dir")
    topic = rospy.get_param("~topic")
    rospy.loginfo("Save Directory: {}".format(save_dir))
    rospy.loginfo("Topic: {}".format(topic))

    os.chdir(save_dir)
    sub = rospy.Subscriber(topic,ROSImage,callback)
    rospy.loginfo("Ready.")

    rospy.spin()

if __name__ == '__main__':

    picam_receiver()

    print("\n\nQuitting.")
