#!/usr/bin/python3
# coding=utf8
import sys
import time
sys.path.append("..")
from sdk import pwm_servo

my_servo = pwm_servo.PWMServo(1)
my_servo.start()

print('servo1 move at different speed')

while True:
    try:
        position = 1300
        duration = 500
        my_servo.set_position(position, duration)
        time.sleep(duration/1000.0)

        position = 1700
        duration = 500
        my_servo.set_position(position, duration)
        time.sleep(duration/1000.0)

        position = 1300
        duration = 1000
        my_servo.set_position(position, duration)
        time.sleep(duration/1000.0)

        position = 1700
        duration = 1000
        my_servo.set_position(position, duration)
        time.sleep(duration/1000.0)
    except KeyboardInterrupt:
        position = 1500
        duratio = 1000
        my_servo.set_position(position, duration)
        time.sleep(duration/1000.0)
        break
