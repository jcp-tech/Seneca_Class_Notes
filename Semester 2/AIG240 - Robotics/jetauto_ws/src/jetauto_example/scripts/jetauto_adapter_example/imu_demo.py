#!/usr/bin/python3
# coding=utf8
import time
from sdk import imu

my_imu = imu.IMU()
print('read imu data at 10hz')

while True:
    try:
        ax, ay, az, gx, gy, gz = my_imu.get_data()  # 读取imu原始数据
        print('ax:{:<8} ay:{:<8} az:{:<8} gx:{:<8} gy:{:<8} gz:{:<8}'.format(round(ax, 5), round(ay, 5), round(az, 5), round(gx, 5), round(gy, 5), round(gz, 5)))
        time.sleep(0.1)
    except Exception as e:
        print(str(e))
