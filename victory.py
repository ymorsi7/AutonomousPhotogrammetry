#!/usr/bin/env python

import struct
# SPDX-FileCopyrightText: 2021 ladyada for Adafruit Industries
# SPDX-License-Identifier: MIT
import depthai
import cv2
import time
import os
from board import SCL, SDA
import busio

# Import the PCA9685 module. Available in the bundle and here:
#   https://github.com/adafruit/Adafruit_CircuitPython_PCA9685
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo


#camera set up 
pipeline = depthai.Pipeline()
cam = pipeline.createColorCamera()
cam.setPreviewSize(300, 300)  # Set the preview size (width, height)

# Create an XLinkOut node to access the camera frames
xout = pipeline.createXLinkOut()
xout.setStreamName("preview")
cam.preview.link(xout.input)

    # Get the output queue for the preview stream
i2c = busio.I2C(SCL, SDA)

# Create a simple PCA9685 class instance.
pca = PCA9685(i2c)
# You can optionally provide a finer tuned reference clock speed to improve the accuracy of the
# timing pulses. This calibration will be specific to each board and its environment. See the
# calibration.py example in the PCA9685 driver.
# pca = PCA9685(i2c, reference_clock_speed=25630710)
pca.frequency = 50

servo7 = servo.Servo(pca.channels[0])
servo_tilt = servo.Servo(pca.channels[1])

# setting the tilt servo to position
servo_tilt = 100

# We sleep in the loops to give the servo time to move into position.
count = 0
servo7.angle = 0
time.sleep(1)
servo7.angle = 180
time.sleep(1)
servo7.angle = 0
