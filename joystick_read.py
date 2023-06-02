#!/usr/bin/env python

import struct
# SPDX-FileCopyrightText: 2021 ladyada for Adafruit Industries
# SPDX-License-Identifier: MIT

import time

from board import SCL, SDA
import busio

# Import the PCA9685 module. Available in the bundle and here:
#   https://github.com/adafruit/Adafruit_CircuitPython_PCA9685
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo

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
"""
# We sleep in the loops to give the servo time to move into position.
for i in range(180):
    servo7.angle = i
    time.sleep(0.03)
for i in range(180):
    servo7.angle = 180 - i
    time.sleep(0.03)

# You can also specify the movement fractionally.
fraction = 0.0
while fraction < 1.0:
    servo7.fraction = fraction
    fraction += 0.01
    time.sleep(0.03)
"""

infile_path = "/dev/input/js0"
EVENT_SIZE = struct.calcsize("LhBB")
file = open(infile_path, "rb")
event = file.read(EVENT_SIZE)

# setting tilt servo down
servo_tilt.angle = 120

while event:
    print(struct.unpack("LhBB", event))
    #(tv_sec, tv_usec, type, code, value) = struct.unpack("LhBB", event)
    (tv_msec,  value, type, number) = struct.unpack("LhBB",event) 
    event = file.read(EVENT_SIZE)
    # if A button is pressed toggle view
    if number == 0:
        if type == 1:
            if value == 1:
                servo7.angle = 0
            if value == 0:
                servo7.angle = 180 
    # if B button is pressed exit program
    if number == 1:
        if type == 1:
            if value == 1:
                break
pca.deinit()
