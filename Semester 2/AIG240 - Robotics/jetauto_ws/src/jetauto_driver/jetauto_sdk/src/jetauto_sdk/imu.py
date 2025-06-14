#!/usr/bin/python3
# coding=utf8
import time
import smbus

IMU_ADDR = 0x68  # imu address
PWR_MGMT_1 = 0x6b

ACCEL_CONFIG = 0x1C
ACCEL_XOUT_H = 0x3B
ACCEL_XOUT_L = 0x3C
ACCEL_YOUT_H = 0x3D
ACCEL_YOUT_L = 0x3E
ACCEL_ZOUT_H = 0x3F
ACCEL_ZOUT_L = 0x40

GYRO_CONFIG = 0x1B
GYRO_XOUT_H = 0x43
GYRO_XOUT_L = 0x44
GYRO_YOUT_H = 0x45
GYRO_YOUT_L = 0x46
GYRO_ZOUT_H = 0x47
GYRO_ZOUT_L = 0x48

class IMU():
    def __init__(self, port=1):
        self.bus = smbus.SMBus(port)
        count = 0
        while True:
            count += 1
            try:
                self.bus.write_byte_data(IMU_ADDR, PWR_MGMT_1, 0)
                break
            except Exception as e:
                print(str(e))
            if count > 100:
                count = 0 
                print('imu init timeout')
                break
            time.sleep(0.01)

    def read_word(self, adr):
        high = self.bus.read_byte_data(IMU_ADDR, adr)
        low = self.bus.read_byte_data(IMU_ADDR, adr+1)
        val = (high << 8) + low
        return val

    def read_word_2c(self, adr):
        val = self.read_word(adr)
        if (val >= 0x8000):
            return -((65535 - val) + 1)
        else:
            return val

    def get_data(self):  
        # Read the acceleration vals
        accel_x = self.read_word_2c(ACCEL_XOUT_H) / 16384.0
        accel_y = self.read_word_2c(ACCEL_YOUT_H) / 16384.0
        accel_z = self.read_word_2c(ACCEL_ZOUT_H) / 16384.0
        
        # Read the gyro vals
        gyro_x = self.read_word_2c(GYRO_XOUT_H) / 131.0
        gyro_y = self.read_word_2c(GYRO_YOUT_H) / 131.0
        gyro_z = self.read_word_2c(GYRO_ZOUT_H) / 131.0
        
        return accel_x*9.8, accel_y*9.8, accel_z*9.8, gyro_x*0.0174, gyro_y*0.0174, gyro_z*0.0174
