#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import math

class SquareMover:
    def __init__(self):
        rospy.init_node('jetauto_square_odom')
        self.pub = rospy.Publisher('/jetauto_controller/cmd_vel', Twist, queue_size=10)
        self.sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.x = 0
        self.y = 0
        self.theta = 0  # degrees
        self.rate = rospy.Rate(10)
        self.got_odom = False

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        self.theta = math.degrees(yaw)
        self.got_odom = True

    def move_to(self, goal_x, goal_y, goal_theta, rotate_while_moving):
        # Move straight to (goal_x, goal_y)
        while not self.got_odom:
            self.rate.sleep()
        print("Start move to ({:.2f},{:.2f}) θ={:.1f}° mode={}".format(goal_x, goal_y, goal_theta, rotate_while_moving))
        twist = Twist()
        while True:
            dx = goal_x - self.x
            dy = goal_y - self.y
            dist = math.sqrt(dx**2 + dy**2)
            target_heading = math.degrees(math.atan2(dy, dx))
            heading_error = (target_heading - self.theta + 540) % 360 - 180  # shortest angle

            if dist < 0.02:
                break
            twist.linear.x = 0.18 * math.cos(math.radians(heading_error))
            twist.linear.y = 0.18 * math.sin(math.radians(heading_error))
            if rotate_while_moving:
                # Gradually rotate toward goal_theta
                ang_error = (goal_theta - self.theta + 540) % 360 - 180
                twist.angular.z = 0.8 * math.radians(ang_error)
            else:
                twist.angular.z = 0
            self.pub.publish(twist)
            self.rate.sleep()
        # Stop movement
        self.pub.publish(Twist())
        rospy.sleep(0.2)

        if not rotate_while_moving:
            # Now turn to the goal heading
            while True:
                ang_error = (goal_theta - self.theta + 540) % 360 - 180
                if abs(ang_error) < 3:
                    break
                twist = Twist()
                twist.angular.z = 0.7 * math.radians(ang_error)
                self.pub.publish(twist)
                self.rate.sleep()
            self.pub.publish(Twist())
            rospy.sleep(0.2)

if __name__ == "__main__":
    mover = SquareMover()
    raw_input("Press Enter to start square movement with odometry...")
    waypoints = [
        (1, 0, 0, False),
        (1, 1, 0, False),
        (1, 1, -90, False),
        (0, 1, -90, False),
        (0, 0, 0, True)
    ]
    for _ in range(2):
        for wp in waypoints:
            mover.move_to(*wp)
    print("Finished square with odometry!")