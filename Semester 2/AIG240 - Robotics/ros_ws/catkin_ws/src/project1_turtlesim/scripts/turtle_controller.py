#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
from pynput import keyboard

pressed_keys = set()

# Define motion combinations
def calculate_twist():
    twist = Twist()

    if 'w' in pressed_keys:
        twist.linear.x = 2.0
    elif 's' in pressed_keys:
        twist.linear.x = -2.0

    if 'a' in pressed_keys:
        twist.angular.z = 2.0
    elif 'd' in pressed_keys:
        twist.angular.z = -2.0

    return twist

def on_press(key):
    try:
        k = key.char.lower()
        if k in ['w', 'a', 's', 'd']:
            pressed_keys.add(k)
    except AttributeError:
        pass

def on_release(key):
    try:
        k = key.char.lower()
        if k in pressed_keys:
            pressed_keys.remove(k)
    except AttributeError:
        pass

def main():
    import sys
    if len(sys.argv) < 2:
        print("Usage: rosrun project1_turtlesim turtle_controller.py <turtle_name>")
        return

    turtle_name = sys.argv[1]
    topic = '/' + turtle_name + '/cmd_vel'

    rospy.init_node('turtle_controller_v2')
    pub = rospy.Publisher(topic, Twist, queue_size=10)
    rate = rospy.Rate(10)

    print("Multi-key controller active. Use W/A/S/D. Ctrl+C to quit.")

    listener = keyboard.Listener(on_press=on_press, on_release=on_release)
    listener.start()

    while not rospy.is_shutdown():
        twist = calculate_twist()
        pub.publish(twist)
        rate.sleep()

if __name__ == '__main__':
    main()
