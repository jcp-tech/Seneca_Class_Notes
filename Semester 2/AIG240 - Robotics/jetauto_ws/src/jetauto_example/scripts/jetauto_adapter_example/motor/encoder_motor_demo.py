#!/usr/bin/python3
# coding=utf8
import sys
import time
sys.path.append("..")
from sdk import encoder_motor

my_motor = encoder_motor.EncoderMotorController(1)
print('motor1 forward and reverse')

while True:
    try:
        motor_id = 2
        speed = 50
        my_motor.set_speed(speed, motor_id=motor_id)
        print('\r<---', end='', flush=True)
        time.sleep(1)
        speed = -speed
        my_motor.set_speed(speed, motor_id=motor_id)
        print('\r--->', end='', flush=True)
        time.sleep(1)
    except KeyboardInterrupt:
        my_motor.set_speed(0, motor_id=motor_id)
        break
