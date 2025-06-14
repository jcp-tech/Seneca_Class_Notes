#!/usr/bin/env python
# encoding: utf-8
# @data:2022/11/04
# @author:aiden
"""
根据cpu，gpu温度控制风扇速度
关闭此功能，重启生效：sudo systemctl disable fan_control.service
"""
import time

while True:
    cpu_temp = open("/sys/class/thermal/thermal_zone1/temp", "r")
    gpu_temp = open("/sys/class/thermal/thermal_zone2/temp", "r")
    thermal1 = int(cpu_temp.read(10))
    thermal2 = int(gpu_temp.read(10))
    cpu_temp.close()
    gpu_temp.close()

    thermal1 = thermal1 / 1000
    thermal2 = thermal2 / 1000

    if thermal1 >= 37 or thermal2 >= 37:
        thermal = '80'
    else:
        thermal = '50'

    fw = open("/sys/devices/pwm-fan/target_pwm", "w")
    fw.write(thermal)
    fw.close()

    time.sleep(60)
