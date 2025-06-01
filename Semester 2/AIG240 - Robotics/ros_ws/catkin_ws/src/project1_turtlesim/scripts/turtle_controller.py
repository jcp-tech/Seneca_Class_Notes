#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, sys # , tty, termios
from geometry_msgs.msg import Twist
from pynput import keyboard  # pip install pynput==1.6.3

pressed_keys = set()
listener = None  # Global so we can stop it later

# def get_key():
#     """Read a single keypress from stdin."""
#     fd = sys.stdin.fileno()
#     old_settings = termios.tcgetattr(fd)
#     try:
#         tty.setraw(fd)
#         key = sys.stdin.read(1)
#     finally:
#         termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
#     return key

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
    global listener

    if len(sys.argv) < 2:
        print("Usage: rosrun project1_turtlesim turtle_controller.py <turtle_name>")
        return

    turtle_name = sys.argv[1]
    topic = '/' + turtle_name + '/cmd_vel'

    rospy.init_node('turtle_controller')
    pub = rospy.Publisher(topic, Twist, queue_size=10)
    rospy.sleep(1)  # Wait a bit for publisher registration

    print("Use keys w/a/s/d/q/e/z/c — or combinations like `w+a` — Ctrl+C or `x` to exit")

    listener = keyboard.Listener(on_press=on_press, on_release=on_release)
    listener.start()

    rate = rospy.Rate(10)
    try:
        while not rospy.is_shutdown():
            # pressed_keys = get_key() # without pynput
            twist = Twist()

            if 'x' in pressed_keys: # Exit command
                print("Exiting...")
                rospy.signal_shutdown("User interrupted with 'x' key")
                # break
            elif 'e' in pressed_keys or ('w' in pressed_keys and 'd' in pressed_keys):
                twist.linear.x = 2.0
                twist.angular.z = -2.0
            elif 'q' in pressed_keys or ('w' in pressed_keys and 'a' in pressed_keys):
                twist.linear.x = 2.0
                twist.angular.z = 2.0
            elif 'z' in pressed_keys or ('s' in pressed_keys and 'a' in pressed_keys):
                twist.linear.x = -2.0
                twist.angular.z = 2.0
            elif 'c' in pressed_keys or ('s' in pressed_keys and 'd' in pressed_keys):
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

    except KeyboardInterrupt:
        print("\nShutting down turtle controller...")
        rospy.signal_shutdown("User interrupted with Ctrl+C")

    finally:
        if listener:
            listener.stop()
        pass

if __name__ == '__main__':
    main() # NOTE: pynput Listener Changes (curretly) not Working Over SSH