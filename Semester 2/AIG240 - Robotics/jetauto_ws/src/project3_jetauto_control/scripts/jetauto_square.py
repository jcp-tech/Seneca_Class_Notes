#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import math

class SmartMover:
    def __init__(self, speed=0.2):
        rospy.init_node('smart_mover')
        self.pub = rospy.Publisher('/jetauto_controller/cmd_vel', Twist, queue_size=10)
        self.sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.rate = rospy.Rate(10)

        self.x = 0
        self.y = 0
        self.yaw = 0
        self.got_odom = False
        self.speed = speed

        rospy.loginfo("Waiting for odometry...")
        while not self.got_odom and not rospy.is_shutdown():
            rospy.sleep(0.1)

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (_, _, yaw) = euler_from_quaternion(orientation_list)
        self.yaw = math.degrees(yaw)
        self.got_odom = True

    def normalize_angle(self, angle):
        while angle > 180:
            angle -= 360
        while angle < -180:
            angle += 360
        return angle

    def rotate_to_angle(self, delta_angle_deg):
        start_yaw = self.yaw
        target_yaw = self.normalize_angle(start_yaw + delta_angle_deg)
        rospy.loginfo("Rotating from %.2f° to %.2f° (delta %.2f°)", start_yaw, target_yaw, delta_angle_deg)

        twist = Twist()
        twist.angular.z = 0.3 if delta_angle_deg > 0 else -0.3

        while not rospy.is_shutdown():
            error = self.normalize_angle(target_yaw - self.yaw)
            if abs(error) < 2:
                break
            self.pub.publish(twist)
            self.rate.sleep()

        twist.angular.z = 0
        self.pub.publish(twist)
        rospy.loginfo("Rotation done.")

    def move_at_angle(self, angle_relative_deg, distance):
        rospy.loginfo("Moving at %.2f° for %.2f meters", angle_relative_deg, distance)
        angle_rad = math.radians(angle_relative_deg)
        vx = self.speed * math.cos(angle_rad)
        vy = self.speed * math.sin(angle_rad)

        twist = Twist()
        twist.linear.x = vx
        twist.linear.y = vy

        start_x = self.x
        start_y = self.y

        moved = 0
        while not rospy.is_shutdown() and moved < distance:
            self.pub.publish(twist)
            moved = math.sqrt((self.x - start_x) ** 2 + (self.y - start_y) ** 2)
            self.rate.sleep()

        twist.linear.x = 0
        twist.linear.y = 0
        self.pub.publish(twist)
        rospy.loginfo("Move done: moved %.2f meters.", moved)

    def run(self, turn_angle_deg, move_angle_deg, move_distance):
        self.rotate_to_angle(turn_angle_deg)
        rospy.sleep(0.3)
        self.move_at_angle(move_angle_deg, move_distance)
        rospy.sleep(0.3)
        # Final stop
        twist = Twist()
        self.pub.publish(twist)
        rospy.loginfo("Segment complete.")

if __name__ == "__main__":
    try:
        mover = SmartMover(speed=0.2)
        raw_input("Press Enter to start square movement,")
        # Run Twice
        for _ in range(2):
            # Step 1
            mover.run(turn_angle_deg=0, move_angle_deg=0, move_distance=1)
            # Step 2
            mover.run(turn_angle_deg=0, move_angle_deg=90, move_distance=1)
            # Step 3
            mover.run(turn_angle_deg=-90, move_angle_deg=0, move_distance=0)
            # Step 4
            mover.run(turn_angle_deg=0, move_angle_deg=-90, move_distance=1)
            # Step5
            mover.run(turn_angle_deg=0, move_angle_deg=0, move_distance=0.5) 
            mover.run(turn_angle_deg=45, move_angle_deg=-45, move_distance=0.5) 
            mover.run(turn_angle_deg=45, move_angle_deg=0, move_distance=0)
        rospy.loginfo("All segments complete.")
        print("All segments complete...")
    except rospy.ROSInterruptException:
        pass