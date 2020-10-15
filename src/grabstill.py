#!/usr/bin/env python

from io import BytesIO
from time import sleep
from picamera import PiCamera
from PIL import Image

STATES = ("IDLE","STILL","START","STOP")

# Create the in-memory stream
stream = BytesIO()
camera = PiCamera()
camera.start_preview()
camera.led = False
sleep(2)
camera.capture(stream, format='jpeg')
# "Rewind" the stream to the beginning so we can read its content
stream.seek(0)
image = Image.open(stream)
image.save('test.jpg')
