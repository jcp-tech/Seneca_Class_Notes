#!/usr/bin/env python
# encoding: utf-8
# @Author: Aiden
# @Date: 2022/11/18
import math
import rospy
from hiwonder_servo_msgs.msg import CommandDuration, JointState

class InitPose:
    def __init__(self):
        rospy.init_node('init_pose')

        joint1_angle = rospy.get_param('~joint1', 0)
        joint2_angle = rospy.get_param('~joint2', -40)
        joint3_angle = rospy.get_param('~joint3', 110)
        joint4_angle = rospy.get_param('~joint4', 70)
        jointr_angle = rospy.get_param('~joint5', 0)
        horizontal = rospy.get_param('~horizontal', False)
        namespace = rospy.get_namespace()

        joint1 = rospy.Publisher('joint1_controller/command_duration', CommandDuration, queue_size=1)
        joint2 = rospy.Publisher('joint2_controller/command_duration', CommandDuration, queue_size=1)
        joint3 = rospy.Publisher('joint3_controller/command_duration', CommandDuration, queue_size=1)
        joint4 = rospy.Publisher('joint4_controller/command_duration', CommandDuration, queue_size=1)
        jointr = rospy.Publisher('r_joint_controller/command_duration', CommandDuration, queue_size=1)
        while not rospy.is_shutdown():
            try:
                if rospy.get_param(namespace + 'hiwonder_servo_manager/running') and rospy.get_param(namespace + 'joint_states_publisher/running'):
                    break
            except:
                rospy.sleep(0.1)
        if horizontal:
            joint1.publish(CommandDuration(data=0, duration=2000))
            joint2.publish(CommandDuration(data=math.radians(-59), duration=2000))
            joint3.publish(CommandDuration(data=math.radians(117), duration=2000))
            joint4.publish(CommandDuration(data=math.radians(31), duration=2000))
            jointr.publish(CommandDuration(data=0, duration=2000))
        else:
            joint1.publish(CommandDuration(data=joint1_angle, duration=2000))
            joint2.publish(CommandDuration(data=math.radians(joint2_angle), duration=2000))
            joint3.publish(CommandDuration(data=math.radians(joint3_angle), duration=2000))
            joint4.publish(CommandDuration(data=math.radians(joint4_angle), duration=2000))
            jointr.publish(CommandDuration(data=jointr_angle, duration=2000))

if __name__ == '__main__':
    InitPose()
