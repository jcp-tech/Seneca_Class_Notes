#!/usr/bin/python3
# coding=utf8
import sys
import time
sys.path.append("..")
from sdk import encoder_motor

my_motor = encoder_motor.EncoderMotorController(1)
print('motor2 speed change 0 - 100 and 100 - 0')
   
while True:
    try:
        motor_id = 2
        for speed in range(101):
            my_motor.set_speed(speed, motor_id=motor_id)
            print('\rspeed: ', str(speed).ljust(5), end='', flush=True)
            time.sleep(0.1)
        for speed in range(101):
            my_motor.set_speed(100 - speed, motor_id=motor_id)
            print('\rspeed: ', str(100 - speed).ljust(5), end='', flush=True)
            time.sleep(0.1)
    except KeyboardInterrupt:
        my_motor.set_speed(0, motor_id=motor_id)
        break
