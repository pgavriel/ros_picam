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
import numpy as np

def callback(img):
    print("Image received.")
    bridge = CvBridge()
    try:
        cv_image = bridge.imgmsg_to_cv2(img, "rgb8")
        cv2.imshow('test',cv_image)
    except CvBridgeError as e:
        print(e)

def picam_receiver():
    rospy.init_node('picam_receiver')
    sub = rospy.Subscriber("picam_output",ROSImage,callback)
    rospy.loginfo("Subscriber up.")
    rospy.spin()
    pass

if __name__ == '__main__':

    picam_receiver()

    print("\n\nQuitting.")
