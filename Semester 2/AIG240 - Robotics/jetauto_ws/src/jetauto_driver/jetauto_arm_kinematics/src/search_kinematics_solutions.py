#!/usr/bin/python3
# coding=utf8
# Date:2021/08/05
# Author:Aiden
# 机械臂根据逆运动学算出的角度进行移动(the robotic arm will move according to the angle calculated by inverse kinematics)
import math

if __name__ == '__main__':
    import kinematics
else:
    from . import kinematics

class SearchKinematicsSolutions:
    joint4_servo_range = (0, 1000, 0, math.radians(240.0), 500)  # 脉宽, 角度, 0点位置(pulse width, angle, 0 point position)
    joint3_servo_range = (0, 1000, 0, math.radians(240.0), 500)
    joint2_servo_range = (0, 1000, 0, math.radians(240.0), 500)
    joint1_servo_range = (0, 1000, 0, math.radians(240.0), 500)

    def __init__(self):
        self.set_servo_range()
    
    def set_servo_range(self, joint1_range=joint1_servo_range, joint2_range=joint2_servo_range, joint3_range=joint3_servo_range, joint4_range=joint4_servo_range):
        # 适配不同的舵机
        self.joint4_servo_range = joint4_range
        self.joint3_servo_range = joint3_range
        self.joint2_servo_range = joint2_range
        self.joint1_servo_range = joint1_range
        self.joint4_scale = (self.joint4_servo_range[1] - self.joint4_servo_range[0]) / (self.joint4_servo_range[3] - self.joint4_servo_range[2])
        self.joint3_scale = (self.joint3_servo_range[1] - self.joint3_servo_range[0]) / (self.joint3_servo_range[3] - self.joint3_servo_range[2])
        self.joint2_scale = (self.joint2_servo_range[1] - self.joint2_servo_range[0]) / (self.joint2_servo_range[3] - self.joint2_servo_range[2])
        self.joint1_scale = (self.joint1_servo_range[1] - self.joint1_servo_range[0]) / (self.joint1_servo_range[3] - self.joint1_servo_range[2])

    def angle2pulse(self, joint1_angle, joint2_angle, joint3_angle, joint4_angle):
        # 将逆运动学算出的角度转换为舵机对应的脉宽值(convert the angle calculated by inverse kinematics into the corresponding pulse width)
        joint4_servo = joint4_angle * self.joint4_scale + self.joint4_servo_range[4]
        if joint4_servo > 1000 or joint4_servo < 0:
            return False

        joint3_servo = -joint3_angle * self.joint3_scale + self.joint3_servo_range[4]
        if joint3_servo > 1000 or joint3_servo < 0:
            return False

        joint2_servo = joint2_angle * self.joint2_scale + self.joint2_servo_range[4]
        if joint2_servo > 1000 or joint2_servo < 0:
            return False

        joint1_servo = -joint1_angle * self.joint1_scale + self.joint1_servo_range[4]
        if joint1_servo > 1000 or joint1_servo < 0:
            return False

        return {"joint1": joint1_servo, "joint2": joint2_servo, "joint3": joint3_servo, "joint4": joint4_servo}

    def pulse2angle(self, joint1_servo, joint2_servo, joint3_servo, joint4_servo):
        joint4_angle = (joint4_servo - self.joint4_servo_range[4]) / self.joint4_scale

        joint3_angle = -(joint3_servo - self.joint3_servo_range[4]) / self.joint3_scale

        joint2_angle = (joint2_servo - self.joint2_servo_range[4]) / self.joint2_scale
        
        joint1_angle = -(joint1_servo - self.joint1_servo_range[4]) / self.joint1_scale

        return {"joint1": joint1_angle, "joint2": joint2_angle, "joint3": joint3_angle, "joint4": joint4_angle}

    def solveFK(self, joint1_angle, joint2_angle, joint3_angle, joint4_angle):
        result = self.pulse2angle(joint1_angle, joint2_angle, joint3_angle, joint4_angle)
        pose = kinematics.solveFK(result['joint1'], result['joint2'], result['joint3'], result['joint4'])
        
        return pose 

    def solveIK(self, coordinate_data, alpha, alpha1, alpha2, d=math.radians(0.2)):
        # 给定坐标coordinate_data和俯仰角alpha,以及俯仰角范围的范围alpha1, alpha2，自动寻找最接近给定俯仰角的解(the given coordinate coordinate_data and pitch angle alpha,
        # and the pitch angle range alpha1, alpha2. Automatically find the solution closest to the given pitch angle)
        # 如果无解返回False,否则返回舵机角度、俯仰角(if there is no solution, return False, otherwise return servo angle and pitch angle)
        # 坐标单位m， 以元组形式传入，例如(0, 0.5, 0.1)(the unit of the coordinate is m. The coordinate is passed in the form of tuple, for example (0, 0.5, 0.1))
        # alpha为给定俯仰角, 单位度(alpha is the given pitch angle in degree)
        # alpha1和alpha2为俯仰角的取值范围(alpha1 and alpha2 are the pitch angle range)

        x, y, z = coordinate_data
        
        d1 = abs(alpha - alpha1)
        d2 = abs(alpha - alpha2)
        a_range1 = 2*int(d2/d) + 1
        a_range2 = int((d1 - d2)/d)
        symmetry_point = alpha - d2
        f = -1
        if d1 < d2:
            f = 1
            symmetry_point = alpha + d1 
            a_range1 = 2*int(d1/d) + 1
            a_range2 = int((d2 - d1)/d)
        for i in range(a_range1):
            if i % 2:
                alpha_ = alpha + ((i + 1)/2)*d
            else:                
                alpha_ = alpha - (i/2)*d
                if alpha_ < alpha1:
                    alpha_ = alpha2 - (i/2)*d
            result = kinematics.solveIK(x, y, z, alpha_)
            if result:
                servos = self.angle2pulse(result[0], result[1], result[2], result[3])
                if servos:
                    return result, servos, alpha_
        for i in range(a_range2):
            alpha_ = symmetry_point + f*(i + 1)*d
            result = kinematics.solveIK(x, y, z, alpha_)
            if result:
                servos = self.angle2pulse(result[0], result[1], result[2], result[3])
                if servos:
                    return result, servos, alpha_
        
        return False

if __name__ == "__main__":
    import rospy
    import logging
    from imp import reload
    from hiwonder_servo_msgs.msg import MultiRawIdPosDur
    from hiwonder_servo_controllers.bus_servo_control import set_servos
    
    # 初始化节点(initialize the node)
    rospy.init_node('kinematics_test')
    
    # 在rospy.init_node后重新加载logging，不然会失效(reload logging after rospy.init_node)
    reload(logging)
    logging.basicConfig(level=logging.ERROR)
    # 舵机发布(servo publishes)
    joints_pub = rospy.Publisher('/servo_controllers/port_id_1/multi_id_pos_dur', MultiRawIdPosDur, queue_size=1)
    rospy.sleep(0.2)
    
    arm_kinematics = SearchKinematicsSolutions()
    t1 = rospy.get_time()
    target = arm_kinematics.solveIK((0, 0.149, 0.4), math.radians(0), math.radians(-90), math.radians(90))
    print('cal time: ', rospy.get_time() - t1)
    if target:
        joint_data = target[1]
        print(joint_data, target[-1])
        t2 = rospy.get_time()
        print(arm_kinematics.solveFK(joint_data['joint1'], joint_data['joint2'], joint_data['joint3'], joint_data['joint4']))
        print('cal time: ', rospy.get_time() - t2)
        set_servos(joints_pub, 2000, ((1, 500), (2, joint_data['joint4']), (3, joint_data['joint3']), (4, joint_data['joint2']), (5, joint_data['joint1'])))
