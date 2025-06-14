#!/usr/bin/env python
# encoding: utf-8
import sys
import time
sys.path.append("..")
from sdk import hiwonder_servo_controller

servo_control = hiwonder_servo_controller.HiwonderServoController('/dev/ttyTHS1', 115200)

print('get serial servo voltage at 10hz')

while True:
    try:
        vol = servo_control.get_servo_vin(5)
        if vol is not None:
            print('\rvoltage: %sV'%(str(vol/1000.0).ljust(6)), end='', flush=True)
        time.sleep(0.1)
    except KeyboardInterrupt:
        break
