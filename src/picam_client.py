#!/usr/bin/env python
import os
import sys
from subprocess import call
from io import BytesIO
import rospy
import rosgraph
from std_msgs.msg import String
from sensor_msgs.msg import Image as ROSImage
from ros_picam.srv import *
from time import sleep
from datetime import datetime

from picamera import PiCamera
from PIL import Image
import cv2
from cv_bridge import CvBridge
import numpy as np
import taskboard_detection as tb

pub = rospy.Publisher("picam_output",ROSImage,queue_size=10)

def setup_camera(label,w=1920,h=1080,fps=30,rotation=0,iso=500):
    camera = PiCamera(resolution=(w, h), framerate=fps)
    # Set ISO to the desired value
    camera.iso = iso
    camera.rotation = rotation
    #camera.annotate_text = label
    # Wait for the automatic gain control to settle
    sleep(2)
    # Now fix the values
    camera.shutter_speed = camera.exposure_speed
    camera.exposure_mode = 'off'
    g = camera.awb_gains
    camera.awb_mode = 'off'
    camera.awb_gains = g
    return camera


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

def grab_still(req):
    success = False
    label = 'grab'  # make configurable?
    num = req.number
    if num < 1:
        num = 1
    rospy.loginfo("GRAB STILL: {}".format(num))
    for i in range(0,num):
        stream = BytesIO()
        camera.capture(stream, format='jpeg')
        # "Rewind" the stream to the beginning so we can read its content
        stream.seek(0)
        image = Image.open(stream)
        filename = '{}-{}-{}.jpg'.format(NODE_NAME,label,get_time_string())
        image.save(filename)
        rospy.loginfo("{} grabbing still {}/{} -> {}".format(NODE_NAME,i+1,num,filename))

    success = True
    print("")
    return GrabStillResponse(success)

def grab_taskboard(req):
        success = False
        publish = True
        savelocal = False
        label = 'taskboard'  # make configurable?
        num = req.number
        if num < 1:
            num = 1
        rospy.loginfo("GRAB TASKBOARD: {}".format(num))
        for i in range(0,num):
            # Start capture and convert to numpy array
            stream = BytesIO()
            camera.capture(stream, format='jpeg')
            data = np.fromstring(stream.getvalue(), dtype=np.uint8)
            np_image = cv2.imdecode(data, 1)
            np_image = np_image[:, :, ::-1]
            # Extract taskboard from image
            taskboard = tb.process_taskboard(np_image,80)
            if publish:
                bridge = CvBridge()
                image_message = bridge.cv2_to_imgmsg(taskboard, "bgr8")
                image_message.header.frame_id = NODE_NAME
                #pub = rospy.Publisher("picam_output",ROSImage,500)
                pub.publish(image_message)
                rospy.loginfo("Taskboard published.")
            if savelocal:
                # Save warped taskboard image
                filename = '{}-{}-{}.png'.format(NODE_NAME,label,get_time_string())
                image = Image.fromarray(taskboard)
                image.save(filename)
                rospy.loginfo("{} grabbing taskboard {}/{} -> {}".format(NODE_NAME,i+1,num,filename))

        success = True
        print("")
        return GrabStillResponse(success)

def start_recording(req):
    global recording, camera
    recording_time = "INDEFINITE" if req.seconds < 1 else "{} seconds".format(req.seconds)
    rospy.loginfo("START RECORDING")
    rospy.loginfo("Length: {}".format(recording_time))
    rospy.loginfo("Resolution: {}".format(camera.resolution))
    rospy.loginfo("FPS: {}".format(camera.framerate))
    success = False

    if not recording:
        label = "rec"  # make configurable?
        filename = "/video/{}-{}-{}.h264".format(NODE_NAME,label,get_time_string())
        recording = True
        camera.start_recording(SAVE_DIR+filename,format='h264')
        if req.seconds > 0:
            # Maybe implement timing manually
            camera.wait_recording(req.seconds)
            camera.stop_recording()
            recording = False
            rospy.loginfo("Done recording.")
        success = True

    else:
        rospy.logwarn("Node is already recording.")

    print("")
    return StartRecordingResponse(success)


def stop_recording(req):
    rospy.loginfo("STOP RECORDING")
    success = False
    global recording, camera
    if not recording:
        rospy.logwarn("Node was not recording.")
    else:
        rospy.loginfo("Recording has been stopped.")
        camera.stop_recording()
        recording = False
        success = True

    print("")
    return StopRecordingResponse(success)


def picam_client():
    global NODE_NAME, SAVE_DIR

    rospy.init_node(NODE_NAME)
    os.chdir(SAVE_DIR)

    #Get ROS params
    enable_video = rospy.get_param("~enable_video")
    cam_w = rospy.get_param("~im_width")
    cam_h = rospy.get_param("~im_height")
    cam_fps = rospy.get_param("~fps")
    cam_iso = rospy.get_param("~iso")
    cam_rotation = rospy.get_param("~rotation")

    rospy.loginfo("NODE_NAME: \"{}\"".format(NODE_NAME))
    rospy.loginfo("SAVE_DIR: \"{}\"".format(SAVE_DIR))
    rospy.loginfo("Video:{} Dim:{}x{} FPS:{} Rotation:{}".format(enable_video,cam_w,cam_h,cam_fps,cam_rotation))

    global recording, camera
    recording = False
    camera = setup_camera(NODE_NAME,cam_w,cam_h,cam_fps,cam_rotation,cam_iso)
    rospy.loginfo("Camera initialized.")

    # TODO: load parameters

    srv1 = rospy.Service(NODE_NAME+'/grab_still', GrabStill, grab_still)
    srv2 = rospy.Service(NODE_NAME+'/grab_taskboard', GrabStill, grab_taskboard)
    if enable_video:
        srv3 = rospy.Service(NODE_NAME+'/start_recording', StartRecording, start_recording)
        srv4 = rospy.Service(NODE_NAME+'/stop_recording', StopRecording, stop_recording)
    rospy.loginfo("Services ready.\n")

    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        master_online = rosgraph.is_master_online()
        if not master_online:
            rospy.logwarn("Master has gone offline. Shutting down.")
            rospy.signal_shutdown("Master offline.")
        rate.sleep()

    # rospy.spin()

    pass

if __name__ == '__main__':
    # Try setting global variables
    global NODE_NAME, SAVE_DIR
    try:
        NODE_NAME = sys.argv[len(sys.argv)-2][8:]
        # Expects save_dir to be sent as first/only argument from launch file [DELICATE]
        SAVE_DIR = sys.argv[1]
    except:
        rospy.logwarn("Something went wrong with arguments. Using Defaults.")
        NODE_NAME = 'picam1'
        SAVE_DIR = '/home/ubuntu/catkin_ws/src/ros_picam/captures/'
    # rospy.loginfo("NODE_NAME set to \"{}\"".format(NODE_NAME))
    # rospy.loginfo("SAVE_DIR set to \"{}\"".format(SAVE_DIR))

    picam_client()

    print("\n\nQuitting.")
