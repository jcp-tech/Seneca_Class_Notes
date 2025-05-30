#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import sys, select, termios, tty
from geometry_msgs.msg import Twist

settings = termios.tcgetattr(sys.stdin)

def get_key():
    tty.setraw(sys.stdin.fileno())
    select_ready, _, _ = select.select([sys.stdin], [], [], 0.1)
    if select_ready:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def main():
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    rospy.init_node('turtle_keyboard_controller')
    move_cmd = Twist()

    instructions = """
Control the turtle with:
  w: forward
  s: backward
  a: turn left
  d: turn right
  x: stop
  q: quit
"""
    print(instructions)

    while not rospy.is_shutdown():
        key = get_key()
        if key == 'w':
            move_cmd.linear.x = 2.0
            move_cmd.angular.z = 0.0
        elif key == 's':
            move_cmd.linear.x = -2.0
            move_cmd.angular.z = 0.0
        elif key == 'a':
            move_cmd.linear.x = 0.0
            move_cmd.angular.z = 2.0
        elif key == 'd':
            move_cmd.linear.x = 0.0
            move_cmd.angular.z = -2.0
        elif key == 'x':
            move_cmd.linear.x = 0.0
            move_cmd.angular.z = 0.0
        elif key == 'q':
            break
        else:
            continue

        pub.publish(move_cmd)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass