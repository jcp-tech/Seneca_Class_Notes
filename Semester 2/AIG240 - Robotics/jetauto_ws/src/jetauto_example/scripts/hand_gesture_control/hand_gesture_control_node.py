#!/usr/bin/env python
# encoding: utf-8
# @data:2022/11/19
# @author:aiden
# 手势控制
import sys
import cv2
import math
import rospy
import signal
import numpy as np
from jetauto_interfaces.msg import Points
sys.path.append('/home/jetauto/jetauto_software/jetauto_arm_pc')
import action_group_controller as controller

class HandGestureControlNode:
    def __init__(self, name):
        rospy.init_node(name, anonymous=True)
        self.image = None
        self.points = []
        self.running = True
        signal.signal(signal.SIGINT, self.shutdown)
        rospy.Subscriber('/hand_trajectory/points', Points, self.get_hand_points_callback)
        rospy.sleep(0.2)

        controller.runAction('horizontal')
        self.hand_gesture_control()
    
    def shutdown(self, signum, frame):
        self.running = False
        rospy.loginfo('shutdown')

    def get_hand_points_callback(self, msg):
        points = []
        if len(msg.points) > 10:
            for i in msg.points:
                points.extend([(int(i.x), int(i.y))])
            self.points = np.array(points)

    def hand_gesture_control(self):
        while self.running:
            if self.points != []:
                line = cv2.fitLine(self.points, cv2.DIST_L2, 0, 0.01, 0.01)
                angle = int(abs(math.degrees(math.acos(line[0][0]))))
                print('>>>>>>', angle)
                if 90 >= angle > 60:
                    rospy.sleep(0.3)
                    controller.runAction('hand_control_pick')
                elif 30 > angle >= 0:
                    rospy.sleep(0.3)
                    controller.runAction('hand_control_place')
                self.points = []
            else:
                rospy.sleep(0.01)

        controller.runAction('horizontal')
        rospy.signal_shutdown('shutdown')

if __name__ == "__main__":
    HandGestureControlNode('hand_gesture_control')
