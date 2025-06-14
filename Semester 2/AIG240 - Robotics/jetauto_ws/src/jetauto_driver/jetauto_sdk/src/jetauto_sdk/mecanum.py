#!/usr/bin/python3
# coding=utf8
#
# 麦克纳姆轮底盘控制(Mecanum wheel chassis control)
#
import math
import time
import jetauto_sdk.encoder_motor as motor

class MecanumChassis:
    # A = 103  # mm
    # B = 97  # mm
    # WHEEL_DIAMETER = 96.5  # mm
    # PULSE_PER_CYCLE = 44

    def __init__(self, a=103, b=97, wheel_diameter=96.5, pulse_per_cycle=44 * 178):
        self.motor_controller = motor.EncoderMotorController(1)
        self.a = a
        self.b = b
        self.wheel_diameter = wheel_diameter
        self.pulse_per_cycle = pulse_per_cycle
        self.velocity = 0
        self.direction = 0
        self.angular_rate = 0

    def speed_covert(self, speed):
        """
        covert speed mm/s to pulse/10ms
        :param speed:
        :return:
        """
        return speed / (math.pi * self.wheel_diameter) * self.pulse_per_cycle * 0.01  # pulse/10ms

    def reset_motors(self):
        self.motor_controller.set_speed((0, 0, 0, 0))
        self.velocity = 0
        self.direction = 0
        self.angular_rate = 0

    def set_velocity(self, speed, direction, angular_rate, speed_up=False, fake=False):
        """
        Use polar coordinates to control moving
                    x
        v1 motor3|  ↑  |motor1 v2
          +  y - |     |
        v4 motor4|     |motor2 v3
        :param speed: mm/s
        :param direction: Moving direction 0~2pi, 1/2pi<--- ↑ ---> 3/2pi
        :param angular_rate:  The speed at which the chassis rotates rad/sec
        :param fake:
        :return:
        """
        vx = speed * math.sin(direction)
        vy = speed * math.cos(direction)
        vp = angular_rate * (self.a + self.b)
        v1 = vy - vx - vp
        v2 = vy + vx + vp
        v3 = vy - vx + vp
        v4 = vy + vx - vp
        v_s = [int(self.speed_covert(v)) for v in [-v2, v3, v1, -v4]]
        if fake:
            return v_s
        if speed_up:
            self.motor_controller.set_speed(v_s[:-1])
            time.sleep(0.03)
        self.motor_controller.set_speed(v_s)
        self.velocity = speed
        self.direction = direction
        self.angular_rate = angular_rate

    def translation(self, velocity_x, velocity_y, fake=False):
        velocity = math.sqrt(velocity_x ** 2 + velocity_y ** 2)
        if velocity_x == 0:
            direction = 90 if velocity_y >= 0 else 270  # pi/2 90deg, (pi * 3) / 2  270deg
        else:
            if velocity_y == 0:
                direction = 0 if velocity_x > 0 else 180
            else:
                direction = math.atan(velocity_y / velocity_x)  # θ=arctan(y/x) (x!=0)
                direction = direction * 180 / math.pi
                if velocity_x < 0:
                    direction += 180
                else:
                    if velocity_y < 0:
                        direction += 360
        if fake:
            return velocity, direction
        else:
            return self.set_velocity(velocity, direction, 0)

    def rotate(self, angular_rate):
        return self.set_velocity(0, 0, angular_rate)
