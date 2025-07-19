#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import math, time

class SmartMover:
    def __init__(self, speed=0.2):
        rospy.init_node('smart_mover')
        self.pub = rospy.Publisher('/jetauto_controller/cmd_vel', Twist, queue_size=10)
        self.sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.rate = rospy.Rate(10)
        self.start_log_pos = None
        self.end_log_pos = None
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
            rospy.loginfo("Rotating: Current Yaw=%.2f°, Target=%.2f°, Error=%.2f°", self.yaw, target_yaw, error)
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
            rospy.loginfo("Current Location: x=%.3f, y=%.3f, yaw=%.2f°, moved=%.3f", self.x, self.y, self.yaw, moved)
            self.rate.sleep()

        twist.linear.x = 0
        twist.linear.y = 0
        self.pub.publish(twist)
        rospy.loginfo("Move done: moved %.2f meters.", moved)

    def mark_log_start(self):
        self.start_log_pos = (self.x, self.y)
        rospy.loginfo("LOGGING STARTED: (%.3f, %.3f)", self.x, self.y)

    def mark_log_end(self):
        self.end_log_pos = (self.x, self.y)
        rospy.loginfo("LOGGING ENDED: (%.3f, %.3f)", self.x, self.y)

    def print_log_result(self, label="Movement"):
        if self.start_log_pos and self.end_log_pos:
            start_x, start_y = self.start_log_pos
            end_x, end_y = self.end_log_pos
            dx = end_x - start_x
            dy = end_y - start_y
            total_dist = math.sqrt(dx**2 + dy**2)
            print("========== {} LOG ==========".format(label))
            print("Start Pos: (%.3f, %.3f)" % (start_x, start_y))
            print("End Pos:   (%.3f, %.3f)" % (end_x, end_y))
            print("Axis Displacement: dx=%.3f m, dy=%.3f m" % (dx, dy))
            print("Total Displacement: %.3f m" % total_dist)
            print("=================================")
        else:
            print("No log data available.")

    def run(self, turn_angle_deg, move_angle_deg, move_distance):
        rospy.loginfo("turn_angle_deg=%.2f°, move_angle_deg=%.2f°, move_distance=%.2f°", turn_angle_deg, move_angle_deg, move_distance)
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
        number_of_segments = 2
        mover = SmartMover(speed=0.1)
        raw_input("Press Enter to start square movement,")
        mover.mark_log_start()  # Start logging position
        # Run Twice
        for _ in range(number_of_segments):
            # Step 1 | Move Forward 1 Meter
            mover.run(turn_angle_deg=0, move_angle_deg=0, move_distance=1)
            time.sleep(1)
            # Step 2 | Without Turning from Current Position Move Left 1 Meter
            mover.run(turn_angle_deg=0, move_angle_deg=90, move_distance=1)
            time.sleep(1)
            # Step 3 | Turn 90 Degrees Clockwise and Don't Move
            mover.run(turn_angle_deg=-90, move_angle_deg=0, move_distance=0)
            time.sleep(1)
            # Step 4 | Without Turning from Current Position Move Right 1 Meter
            mover.run(turn_angle_deg=0, move_angle_deg=-90, move_distance=1)
            time.sleep(1)
            # Step5 | Move Forward 1 Meter while Turning Anticlockwise During the Journey
            desired_distance = 0.2
            overshoot = 0.15 # Adjusted Displacement to Account for the Size of the Robot
            move_distance = desired_distance - overshoot
            mover.run(turn_angle_deg=0, move_angle_deg=0, move_distance=move_distance) # Move Forward {move_distance} Meters without Turning, NOTE: This was Supposed to be 0.2 Meters but was Changed to {move_distance} Meters to Account for the Displacement of the Robot probably caused because of the Size.
            # mover.run(turn_angle_deg=0, move_angle_deg=0, move_distance=overshoot) # TODO - Uncomment this line if you want to move forward the Extra {overshoot} Meters which we Deducted from the Previous Step.
            mover.run(turn_angle_deg=45, move_angle_deg=-45, move_distance=0.4) # After Turing 45 Degrees (first), Move Forward 0.4 Meters from the Position at a 45 Degree Angle.
            mover.run(turn_angle_deg=45, move_angle_deg=-90, move_distance=0.4) # After Turing 45 Degrees (again), Move Right 0.4 Meters from the Position.
            time.sleep(1)
        mover.mark_log_end()
        rospy.loginfo("All segments complete.\n")
        mover.print_log_result(label="Square Movement * %d" % number_of_segments)  # Print the log result
    except rospy.ROSInterruptException:
        pass