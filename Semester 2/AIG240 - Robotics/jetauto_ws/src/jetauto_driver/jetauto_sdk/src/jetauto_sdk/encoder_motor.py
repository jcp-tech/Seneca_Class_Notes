#!/usr/bin/env python
import os
import smbus2
import threading
import struct

ENCODER_MOTOR_MODULE_ADDRESS = 0x34

class EncoderMotorController:
    def __init__(self, i2c_port, motor_type=3):
        self.i2c_port = i2c_port
        self.machine_type = os.environ.get('MACHINE_TYPE')
        with smbus2.SMBus(self.i2c_port) as bus:
            bus.write_i2c_block_data(ENCODER_MOTOR_MODULE_ADDRESS, 20, [motor_type, ])

    def set_speed(self, speed, motor_id=None, offset=0):
        with smbus2.SMBus(self.i2c_port) as bus:
            # motor speed  control register address is 51 + motor_id - 1
            if motor_id is None:
                for id_index in range(len(speed)):
                    new_id = 1
                    if id_index == 0:
                        new_id = 3
                    elif id_index == 2:
                        new_id = 1
                    elif id_index == 1:
                        new_id = 4
                    elif id_index == 3:
                        new_id = 2
                    motor_speed = speed[id_index]
                    sp = motor_speed
                    if sp > 100:
                        sp = 100
                    elif sp < -100:
                        sp = -100
                    try:
                        bus.write_i2c_block_data(ENCODER_MOTOR_MODULE_ADDRESS, 50 + new_id, [sp, ])
                    except BaseException as e:
                        print(e)
                        bus.write_i2c_block_data(ENCODER_MOTOR_MODULE_ADDRESS, 50 + new_id, [sp, ])
            else:
                if 0 < motor_id <= 4:
                    if motor_id == 1:
                        motor_id = 3
                    elif motor_id == 3:
                        motor_id = 1
                    elif motor_id == 2:
                        motor_id = 4
                    elif motor_id == 4:
                        motor_id = 2
                    speed = 100 if speed > 100 else speed
                    speed = -100 if speed < -100 else speed
                    try:
                        bus.write_i2c_block_data(ENCODER_MOTOR_MODULE_ADDRESS, 50 + motor_id, [speed, ])
                    except BaseException as e:
                        print(e)
                        bus.write_i2c_block_data(ENCODER_MOTOR_MODULE_ADDRESS, 50 + motor_id, [speed, ])
                else:
                    raise ValueError("Invalid motor id")

    def clear_encoder(self, motor_id=None):
        with smbus2.SMBus(self.i2c_port) as bus:
            if motor_id is None:
                bus.write_i2c_block_data(ENCODER_MOTOR_MODULE_ADDRESS, 60, [0]*16)
            else:
                if 0 < motor_id < 4:
                    bus.write_i2c_block_data(ENCODER_MOTOR_MODULE_ADDRESS, 60 + motor_id*4, [0]*4)
                else:
                    raise ValueError("Invalid motor id")

    def read_encoder(self, motor_id=None):
        with smbus2.SMBus(self.i2c_port) as bus:
            if motor_id is None:
                return list(struct.unpack('<iiii',bytes(bus.read_i2c_block_data(ENCODER_MOTOR_MODULE_ADDRESS, 60, 16))))
            else:
                if 0 < motor_id < 4:
                    return struct.unpack('<i',bytes(bus.read_i2c_block_data(ENCODER_MOTOR_MODULE_ADDRESS, 60 + motor_id*4, 4)))[0]
                else:
                    raise ValueError("Invalid motor id")
