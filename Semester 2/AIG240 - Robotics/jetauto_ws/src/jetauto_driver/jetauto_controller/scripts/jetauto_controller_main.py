#!/usr/bin/env python
# -*- coding: utf-8 -*-
import os
import sys
import math
import rospy
from std_srvs.srv import Trigger
from jetauto_sdk.mecanum import MecanumChassis
from geometry_msgs.msg import Twist, TransformStamped

class JetAutoController:
    def __init__(self):
        rospy.init_node('jetauto_controller', anonymous=False)
        
        self.last_linear_x = 0
        self.last_linear_y = 0
        self.last_angular_z = 0

        self.mecanum = MecanumChassis(a =103, b=97, wheel_diameter=96.5, pulse_per_cycle=4320)#44.0 * 178.0
        
        self.machine_type = os.environ.get('MACHINE_TYPE')
        self.go_factor = rospy.get_param('~go_factor', 1.00)
        self.turn_factor = rospy.get_param('~turn_factor', 1.00)
        
        rospy.Subscriber('jetauto_controller/cmd_vel', Twist, self.cmd_vel_callback)
        rospy.Subscriber('cmd_vel', Twist, self.app_cmd_vel_callback)
        rospy.Service('jetauto_controller/load_calibrate_param', Trigger, self.load_calibrate_param)
          
    def load_calibrate_param(self, msg):
        self.go_factor = rospy.get_param('~go_factor', 1.00)
        self.turn_factor = rospy.get_param('~turn_factor', 1.00)
        rospy.loginfo('load_calibrate_param')
        
        return [True, 'load_calibrate_param']
    
    def app_cmd_vel_callback(self, msg):
        if msg.linear.x > 0.2:
            msg.linear.x = 0.2
        if msg.linear.x < -0.2:
            msg.linear.x = -0.2
        if msg.angular.z > 0.5:
            msg.angular.z = 0.5
        if msg.angular.z < -0.5:
            msg.angular.z = -0.5
        self.cmd_vel_callback(msg)

    def cmd_vel_callback(self, msg):
        linear_x = self.go_factor*msg.linear.x
        linear_y = self.go_factor*msg.linear.y
        angular_z = self.turn_factor*msg.angular.z
        
        speed_up = False
        if abs(self.last_linear_x - linear_x) > 0.2 or abs(self.last_linear_y - linear_y) > 0.2 or abs(self.last_angular_z - angular_z) > 1:
            speed_up = True
        
        self.last_linear_x = linear_x
        self.last_linear_y = linear_y
        self.last_angular_z = angular_z

        linear_x_, linear_y_ = linear_x * 1000.0, linear_y * 1000.0#mm to m
        speed = math.sqrt(linear_x_ ** 2 + linear_y_ ** 2)
        direction =  math.atan2(linear_y_, linear_x_)
        direction = math.pi * 2 + direction if direction < 0 else direction
        self.mecanum.set_velocity(speed, direction, angular_z, speed_up=speed_up)

if __name__ == "__main__":
    try:
        node = JetAutoController()
        rospy.spin()
    except Exception as e:
        rospy.logerr(str(e))
    finally:
        node.mecanum.reset_motors()
        sys.exit()
