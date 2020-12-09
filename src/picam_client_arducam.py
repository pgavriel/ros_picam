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
from picamera.array import PiRGBArray
from PIL import Image
import cv2
from cv_bridge import CvBridge
import numpy as np
import taskboard_detection as tb

import smbus
bus = smbus.SMBus(0)

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
        # self.enable_video = rospy.get_param("~enable_video", True)
        # self.cam_w = rospy.get_param("~im_width", 1920)
        # self.cam_h = rospy.get_param("~im_height", 1080)
        # self.cam_fps = rospy.get_param("~fps", 30)
        # self.cam_brightness = rospy.get_param("~brightness", 50)
        # self.cam_contrast = rospy.get_param("~contrast", 0)
        # self.cam_iso = rospy.get_param("~iso", 200)
        # self.cam_rotation = rospy.get_param("~rotation", 0)
        # Create PiCamera object
        self.camera = self.setup_camera()
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
                #taskboard = tb.process_taskboard(np_image,80)
                taskboard = tb.process_taskboard(np_image,98)
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

    def setup_camera(self):
        # There are more PiCamera Parameters that can be implemented if needed.
        # Docs here: https://picamera.readthedocs.io/en/release-1.10/api_camera.html
        self.enable_video = rospy.get_param("~enable_video", True)
        self.cam_w = rospy.get_param("~im_width", 1920)
        self.cam_h = rospy.get_param("~im_height", 1080)
        self.cam_fps = rospy.get_param("~fps", 30)
        self.cam_brightness = rospy.get_param("~brightness", 50)
        self.cam_contrast = rospy.get_param("~contrast", 0)
        self.cam_iso = rospy.get_param("~iso", 200)
        self.cam_rotation = rospy.get_param("~rotation", 0)
        self.cam_focus = rospy.get_param("~focus", 0)
        rospy.loginfo("Creating PiCamera object...")
        rospy.loginfo("Dim: {}x{}  FPS: {}".format(self.cam_w,self.cam_h,self.cam_fps))
        rospy.loginfo("Brightness: {}  Contrast: {}".format(self.cam_brightness,self.cam_contrast))
        rospy.loginfo("ISO: {}  Rotation: {}".format(self.cam_iso,self.cam_rotation))
        rospy.loginfo("Focus: {}".format(self.cam_focus))
        camera = PiCamera(resolution=(self.cam_w, self.cam_h), framerate=self.cam_fps)
        camera.brightness = self.cam_brightness
        camera.contrast = self.cam_contrast
        camera.iso = self.cam_iso
        camera.rotation = self.cam_rotation

        # Set camera focus
        if self.cam_focus == 0:
            self.autofocus(camera)
            # Return to desired resolution
            camera.resolution = (self.cam_w, self.cam_h)
        elif self.cam_focus > 950:
            self.set_focus(950)
        elif self.cam_focus < 15:
            self.set_focus(15)
        else:
            self.set_focus(self.cam_focus)

        # Wait for the automatic gain control to settle
        sleep(3)
        # Now fix the values
        camera.shutter_speed = camera.exposure_speed
        camera.exposure_mode = 'off'
        #g = camera.awb_gains
        #camera.awb_mode = 'off'
        #camera.awb_gains = g
        camera.awb_mode = 'auto'
        return camera

    #Arducam Specific Functions
    def set_focus(self,val):
        value = (val << 4) & 0x3ff0
        data1 = (value >> 8) & 0x3f
        data2 = value & 0xf0
        # time.sleep(0.5)
        rospy.loginfo("Focus Set: {}".format(val))
        # bus.write_byte_data(0x0c,data1,data2)
        os.system("i2cset -y 0 0x0c %d %d" % (data1,data2))

    def calculate_focus(self,camera):
        rawCapture = PiRGBArray(camera)
        camera.capture(rawCapture,format="bgr", use_video_port=True)
        image = rawCapture.array
        rawCapture.truncate(0)
        img_gray = cv2.cvtColor(image,cv2.COLOR_RGB2GRAY)
        img_sobel = cv2.Laplacian(img_gray,cv2.CV_16U)
        #img_sobel = cv2.Sobel(img_gray,cv2.CV_16U,1,1)
        focus_score = cv2.mean(img_sobel)[0]
        rospy.loginfo("Focus Score: {}".format(focus_score))
        return focus_score

    def autofocus(self,camera):
        camera.resolution = (640, 480)
        sleep(0.1)
        rospy.loginfo("Starting Autofocus...")
        max_index = 10
        max_value = 0.0
        last_value = 0.0
        dec_count = 0
        focal_distance = 10
        while True:
            # Set focus
            self.set_focus(focal_distance)
            sleep(0.1)
            # Take image and calculate clarity
            val = self.calculate_focus(camera)
            # Find maximum image clarity
            if val > max_value:
                max_index = focal_distance
                max_value = val
            # If clarity starts to decrease
            if val < last_value:
                dec_count += 1
            else:
                dec_count = 0
            # Image clarity is reducted for six consecutive frames
            if dec_count > 6:
                break
            last_value = val
            # Increase the focal distance
            focal_distance += 15
            if focal_distance > 1000:
                break

        rospy.loginfo("Max Index: {}, Max Value: {}\n".format(max_index,max_value))
        rospy.loginfo("Fine Tuning...")
        fine_lb = max_index - 25
        if fine_lb < 10: fine_lb = 10
        fine_ub = max_index + 25
        if fine_ub > 1000: fine_ub = 1000
        focal_distance = fine_lb
        max_index = fine_lb
        max_value = 0.0
        rospy.loginfo("Range: {}-{}".format(fine_lb,fine_ub))
        sleep(0.2)
        while focal_distance <= fine_ub:
            #Adjust focus
            self.set_focus(focal_distance)
            sleep(0.1)
            #Take image and calculate image clarity
            val = self.calculate_focus(camera)
            #Find the maximum image clarity
            if val > max_value:
            	max_index = focal_distance
            	max_value = val
            focal_distance += 5

        self.set_focus(max_index)
        rospy.loginfo("Max Index: {}, Max Value: {}\n".format(max_index,max_value))
        sleep(0.1)


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
