#!/usr/bin/python3
# coding=utf8
import time
from sdk import buzzer

print('buzzer di at 1hz')

try:
    while True:
        buzzer.on()  # buzzer di
        time.sleep(0.1)
        buzzer.off()  # buzzer off
        time.sleep(0.9)
except KeyboardInterrupt:
    buzzer.off()  # buzzer off
