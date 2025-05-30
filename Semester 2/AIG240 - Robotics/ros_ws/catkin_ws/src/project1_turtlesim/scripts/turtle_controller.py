#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
from pynput import keyboard # pip install pynput==1.6.3

pressed_keys = set()

def on_press(key):
    try:
        pressed_keys.add(key.char)
    except AttributeError:
        pass

def on_release(key):
    try:
        pressed_keys.discard(key.char)
    except AttributeError:
        pass

def main():
    import sys
    if len(sys.argv) < 2:
        print("Usage: rosrun project1_turtlesim turtle_controller.py <turtle_name>")
        return

    turtle_name = sys.argv[1]
    topic = '/' + turtle_name + '/cmd_vel'

    rospy.init_node('turtle_controller')
    pub = rospy.Publisher(topic, Twist, queue_size=10)

    print("Use keys w/a/s/d/q/e/z/c — or combinations like w+d — Ctrl+C to exit")

    listener = keyboard.Listener(on_press=on_press, on_release=on_release)
    listener.start()

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        twist = Twist()

        if ('w' in pressed_keys and 'd' in pressed_keys) or "e" in pressed_keys:
            twist.linear.x = 2.0
            twist.angular.z = -2.0
        elif ('w' in pressed_keys and 'a' in pressed_keys) or "q" in pressed_keys:
            twist.linear.x = 2.0
            twist.angular.z = 2.0
        elif ('s' in pressed_keys and 'a' in pressed_keys) or "z" in pressed_keys:
            twist.linear.x = -2.0
            twist.angular.z = 2.0
        elif ('s' in pressed_keys and 'd' in pressed_keys) or "c" in pressed_keys:
            twist.linear.x = -2.0
            twist.angular.z = -2.0
        elif 'w' in pressed_keys:
            twist.linear.x = 2.0
        elif 's' in pressed_keys:
            twist.linear.x = -2.0
        elif 'a' in pressed_keys:
            twist.angular.z = 2.0
        elif 'd' in pressed_keys:
            twist.angular.z = -2.0
        else:
            twist.linear.x = 0.0
            twist.angular.z = 0.0

        pub.publish(twist)
        rate.sleep()

    listener.stop()

if __name__ == '__main__':
    main()