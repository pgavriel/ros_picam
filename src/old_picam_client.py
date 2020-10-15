#!/usr/bin/env python

from io import BytesIO
from time import sleep
from picamera import PiCamera
from PIL import Image
import rospy
from std_msgs.msg import String

def setup_camera(label,iso=100):
    camera = PiCamera(resolution=(1640, 1232), framerate=30)
    # Set ISO to the desired value
    camera.iso = iso
    camera.annotate_text = label
    # Wait for the automatic gain control to settle
    sleep(2)
    # Now fix the values
    camera.shutter_speed = camera.exposure_speed
    camera.exposure_mode = 'off'
    g = camera.awb_gains
    camera.awb_mode = 'off'
    camera.awb_gains = g
    return camera

def set_state(new_state,current_state,states):
    print("Set state called.")
    s = str(new_state.data).upper()
    if s in states:
        current_state = s
        print("State set to "+current_state+".")
    else:
        print("State "+s+" not recognized.")
        print("Current state remains "+current_state+".")
    print("\n")
    return current_state

def grab_still():
    print("Grabbing still ")
    return

def picam_client():
    rospy.init_node('camera_listener', anonymous=True)
    state_topic = 'camera_state'
    #rospy.Subscriber(state_topic, String, set_state)

    STATES = ("IDLE","STILL","START","STOP","SHUTDOWN")
    STATE = STATES[0]
    print("States: "+str(STATES))
    print("State: "+STATE)
    print("\n")
    camera = setup_camera('picam1')
    print("Camera initialized.\n")

    while not rospy.is_shutdown():
        print("Current state: "+STATE)
        print("Waiting for message...")
        message = rospy.wait_for_message(state_topic,String, timeout=None)
        STATE = set_state(message,STATE,STATES)
        print("Current state: "+STATE)

        # IDLE
        if STATE == STATES[0]:
            pass
        # STILL    
        elif STATE == STATES[1]:
            camera.capture('testing.jpg')
            #grab_still(camera)
            STATE = STATES[0]
        # SHUTDOWN
        elif STATE == STATES[4]:
            rospy.signal_shutdown("Received shutdown message.")


        sleep(3)

    rospy.spin()
    pass

if __name__ == '__main__':
    #try:
    picam_client()
    #except Exception:
    print("\n\nQuitting.")
