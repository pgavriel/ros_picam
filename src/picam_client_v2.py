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


class picam_client:
    def __init__(self):
        # ROS Node Setup
        self.name = sys.argv[len(sys.argv)-2][8:]
        rospy.init_node(self.name)
        rospy.loginfo("NODE_NAME: \"{}\"".format(self.name))

        # Output Parameters
        self.save_local = rospy.get_param("~save_local")
        self.save_dir = rospy.get_param("~save_dir")
        self.publish = rospy.get_param("~publish")
        self.publish_topic = rospy.get_param("~publish_topic")
        str = "Save Local: {}   ".format(self.save_local)
        if (self.save_local):
            str = str + "Directory: {}".format(self.save_dir)
            os.chdir(self.save_dir)
        rospy.loginfo(str)
        str = "Publish: {}   ".format(self.publish)
        if (self.publish):
            str = str + "Topic: {}".format(self.publish_topic)
            self.pub = rospy.Publisher(self.publish_topic,ROSImage,queue_size=100)
        rospy.loginfo(str)

        # Camera Parameters
        self.enable_video = rospy.get_param("~enable_video")
        self.cam_w = rospy.get_param("~im_width")
        self.cam_h = rospy.get_param("~im_height")
        self.cam_fps = rospy.get_param("~fps")
        self.cam_brightness = rospy.get_param("~brightness")
        self.cam_contrast = rospy.get_param("~contrast")
        self.cam_iso = rospy.get_param("~iso")
        self.cam_rotation = rospy.get_param("~rotation")
        # TODO: Rewrite to send self to setup_camera, that'd be cleaner
        self.camera = self.setup_camera(self.cam_w,self.cam_h,self.cam_fps,self.cam_brightness,self.cam_contrast,self.cam_iso,self.cam_rotation)
        rospy.loginfo("Camera initialized.")

        # Setup Services
        rospy.loginfo("Enable Video Services: {}".format(self.enable_video))
        srv1 = rospy.Service(self.name+'/grab_still', GrabStill, self.grab_still)
        srv2 = rospy.Service(self.name+'/grab_taskboard', GrabStill, self.grab_taskboard)
        if self.enable_video:
            srv3 = rospy.Service(self.name+'/start_recording', StartRecording, self.start_recording)
            srv4 = rospy.Service(self.name+'/stop_recording', StopRecording, self.stop_recording)
        rospy.loginfo("Services ready.\n")


    def get_time_string(self,include_date=False):
        dt = datetime.now()
        dt_string = ""
        date = "{:02}-{:02}-{:02}".format(dt.month,dt.day,dt.year-2000)
        time = "{:02}-{:02}-{:02}-{:06}".format(dt.hour,dt.minute,dt.second,dt.microsecond)
        if include_date:
            dt_string = "{}--{}".format(date,time)
        else:
            dt_string = time
        return dt_string


    def grab_still(self,req):
        success = False
        label = 'grab'  # make configurable?
        num = req.number
        if num < 1:
            num = 1
        rospy.loginfo("GRAB STILL: {}".format(num))
        for i in range(0,num):
            rospy.loginfo("{} grabbing still {}/{}".format(self.name,i+1,num))
            # Start capture and convert to numpy array
            stream = BytesIO()
            self.camera.capture(stream, format='jpeg')
            data = np.fromstring(stream.getvalue(), dtype=np.uint8)
            np_image = cv2.imdecode(data, 1)
            np_image = np_image[:, :, ::-1]
            if self.publish:
                # Publish still image
                bridge = CvBridge()
                image_message = bridge.cv2_to_imgmsg(np_image, "bgr8")
                image_message.header.frame_id = self.name
                self.pub.publish(image_message)
                rospy.loginfo("{} Image Published to \"{}\"".format(self.name,self.publish_topic))
            if self.save_local:
                # Save still image
                filename = '{}-{}-{}.png'.format(self.name,label,self.get_time_string())
                image = Image.fromarray(np_image)
                image.save(filename)
                rospy.loginfo("{} Image Saved -> {}".format(self.name,filename))
        success = True
        print("")
        return GrabStillResponse(success)

    def grab_taskboard(self,req):
            success = False
            label = 'taskboard'  # make configurable?
            num = req.number
            if num < 1:
                num = 1
            rospy.loginfo("GRAB TASKBOARD: {}".format(num))
            for i in range(0,num):
                rospy.loginfo("{} grabbing taskboard {}/{}".format(self.name,i+1,num))
                # Start capture and convert to numpy array
                stream = BytesIO()
                self.camera.capture(stream, format='jpeg')
                data = np.fromstring(stream.getvalue(), dtype=np.uint8)
                np_image = cv2.imdecode(data, 1)
                np_image = np_image[:, :, ::-1]
                # Extract taskboard from image
                taskboard = tb.process_taskboard(np_image,80)
                if self.publish:
                    # Publish taskboard image
                    bridge = CvBridge()
                    image_message = bridge.cv2_to_imgmsg(taskboard, "bgr8")
                    image_message.header.frame_id = self.name
                    self.pub.publish(image_message)
                    rospy.loginfo("{} Taskboard Published to \"{}\"".format(self.name,self.publish_topic))
                if self.save_local:
                    # Save warped taskboard image
                    filename = '{}-{}-{}.png'.format(self.name,label,self.get_time_string())
                    image = Image.fromarray(taskboard)
                    image.save(filename)
                    rospy.loginfo("{} Taskboard Saved -> {}".format(self.name,filename))

            success = True
            print("")
            return GrabStillResponse(success)

    def start_recording(self,req):
        recording_time = "INDEFINITE" if req.seconds < 1 else "{} seconds".format(req.seconds)
        rospy.loginfo("START RECORDING")
        rospy.loginfo("Length: {}".format(recording_time))
        rospy.loginfo("Resolution: {}".format(self.camera.resolution))
        rospy.loginfo("FPS: {}".format(self.camera.framerate))
        success = False

        if not self.camera.recording:
            label = "rec"  # make configurable?
            filename = "/video/{}-{}-{}.h264".format(self.name,label,self.get_time_string())
            recording = True
            self.camera.start_recording(self.save_dir+filename,format='h264')
            if req.seconds > 0:
                # Maybe implement timing manually
                self.camera.wait_recording(req.seconds)
                self.camera.stop_recording()
                recording = False
                rospy.loginfo("Done recording.")
            success = True

        else:
            rospy.logwarn("Node is already recording.")

        print("")
        return StartRecordingResponse(success)


    def stop_recording(self,req):
        rospy.loginfo("STOP RECORDING")
        success = False
        if not self.camera.recording:
            rospy.logwarn("Node was not recording.")
        else:
            rospy.loginfo("Recording has been stopped.")
            self.camera.stop_recording()
            recording = False
            success = True

        print("")
        return StopRecordingResponse(success)

    def setup_camera(self,w=1920,h=1080,fps=30,brightness=50,contrast=0,iso=100,rotation=0):
        # There are more PiCamera Parameters that can be implemented if needed.
        # Docs here: https://picamera.readthedocs.io/en/release-1.10/api_camera.html
        rospy.loginfo("Creating PiCamera object...")
        rospy.loginfo("Dim: {}x{}  FPS: {}".format(w,h,fps))
        rospy.loginfo("Brightness: {}  Contrast: {}".format(brightness,contrast))
        rospy.loginfo("ISO: {}  Rotation: {}".format(iso,rotation))
        camera = PiCamera(resolution=(w, h), framerate=fps)
        camera.brightness = brightness
        camera.contrast = contrast
        camera.iso = iso
        camera.rotation = rotation
        # Wait for the automatic gain control to settle
        sleep(2)
        # Now fix the values
        camera.shutter_speed = camera.exposure_speed
        camera.exposure_mode = 'off'
        g = camera.awb_gains
        camera.awb_mode = 'off'
        camera.awb_gains = g
        return camera


if __name__ == '__main__':
    # Create Picam Client
    client = picam_client()

    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        master_online = rosgraph.is_master_online()
        if not master_online:
            rospy.logwarn("Master has gone offline. Shutting down.")
            rospy.signal_shutdown("Master offline.")
        rate.sleep()

    client.camera.close()
    print("Camera object closed.")

    print("\n\nQuitting.")
