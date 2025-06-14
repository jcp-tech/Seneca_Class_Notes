#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

def move_square():
    rospy.init_node('move_square')
    pub = rospy.Publisher('/jetauto_controller/cmd_vel', Twist, queue_size=10)
    move = Twist()
    rate = rospy.Rate(10)

    # Move forward
    for _ in range(20):  # 2 seconds at 10Hz
        move.linear.x = 0.2
        move.angular.z = 0.0
        pub.publish(move)
        rate.sleep()

    # Turn
    for _ in range(10):  # 1 second at 10Hz
        move.linear.x = 0.0
        move.angular.z = 1.0
        pub.publish(move)
        rate.sleep()

    # Stop
    move.linear.x = 0.0
    move.angular.z = 0.0
    pub.publish(move)

if __name__ == "__main__":
    move_square()

