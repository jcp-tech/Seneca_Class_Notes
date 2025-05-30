#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
import sys
import tty
import termios

def get_key():
    """Read a single keypress from stdin."""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        key = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return key

def main():
    if len(sys.argv) < 2:
        print("Usage: rosrun project1_turtlesim turtle_controller.py <turtle_name>")
        return

    turtle_name = sys.argv[1]
    topic = '/' + turtle_name + '/cmd_vel'

    rospy.init_node('turtle_controller')
    pub = rospy.Publisher(topic, Twist, queue_size=10)

    print("Control the turtle using keys: w/a/s/d/q/e/z/c — press Ctrl+C to quit")

    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown():
        key = get_key()
        twist = Twist()

        if key == 'w':
            twist.linear.x = 2.0
        elif key == 's':
            twist.linear.x = -2.0
        elif key == 'a':
            twist.angular.z = 2.0
        elif key == 'd':
            twist.angular.z = -2.0
        elif key == 'q':
            twist.linear.x = 2.0
            twist.angular.z = 2.0
        elif key == 'e':
            twist.linear.x = 2.0
            twist.angular.z = -2.0
        elif key == 'z':
            twist.linear.x = -2.0
            twist.angular.z = 2.0
        elif key == 'c':
            twist.linear.x = -2.0
            twist.angular.z = -2.0
        else:
            twist.linear.x = 0.0
            twist.angular.z = 0.0

        pub.publish(twist)
        rate.sleep()

if __name__ == '__main__':
    main()