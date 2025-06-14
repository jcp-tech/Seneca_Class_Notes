#!/usr/bin/python3
# coding=utf8
import sys
import time
sys.path.append("..")
from sdk import pwm_servo

servo1 = pwm_servo.PWMServo(1)
servo2 = pwm_servo.PWMServo(2)
servo1.start()
servo2.start()

print('two servo move between 1300-1700')

while True:
    try:
        position = 1300
        duration = 1000
        servo1.set_position(position, duration)
        servo2.set_position(position, duration)
        time.sleep(duration/1000.0)

        position = 1700
        duration = 1000
        servo1.set_position(position, duration)
        servo2.set_position(position, duration)
        time.sleep(duration/1000.0)
    except KeyboardInterrupt:
        position = 1500
        duratio = 1000
        servo1.set_position(position, duration)
        servo2.set_position(position, duration)
        time.sleep(duration/1000.0)
        break
