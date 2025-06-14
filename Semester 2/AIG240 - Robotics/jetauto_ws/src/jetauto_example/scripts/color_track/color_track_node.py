#!/usr/bin/env python
# encoding: utf-8
# @data:2022/11/18
# @author:aiden
# 颜色跟踪
import rospy
import signal
import jetauto_sdk.pid as pid
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger, TriggerResponse
from hiwonder_servo_msgs.msg import MultiRawIdPosDur
from jetauto_interfaces.msg import ColorsInfo, ColorDetect
from jetauto_interfaces.srv import SetColorDetectParam, SetString
from hiwonder_servo_controllers.bus_servo_control import set_servos
from kinematics.search_kinematics_solutions import SearchKinematicsSolutions

class ColorTrackNode:
    def __init__(self, name):
        rospy.init_node(name, anonymous=True)
        self.z_dis = 0.38
        self.x_dis = 500
        self.y_init = 0.149
        self.center = None
        self.running = True
        self.start = False
        self.name = name
        signal.signal(signal.SIGINT, self.shutdown)
        self.pid_z = pid.PID(0.00005, 0.0, 0.0)
        self.pid_x = pid.PID(0.04, 0.0, 0.0)
        self.joints_pub = rospy.Publisher('servo_controllers/port_id_1/multi_id_pos_dur', MultiRawIdPosDur, queue_size=1)  # 舵机控制
        rospy.Subscriber('/color_detect/color_info', ColorsInfo, self.get_color_callback)
        rospy.Service(self.name + '/start', Trigger, self.start_srv_callback)  # 进入玩法
        rospy.Service(self.name + '/stop', Trigger, self.stop_srv_callback)  # 退出玩法
        rospy.Service(self.name + '/set_color', SetString, self.set_color_srv_callback)  # 设置颜色
        self.mecanum_pub = rospy.Publisher('/jetauto_controller/cmd_vel', Twist, queue_size=1)  # 底盘控制
        self.search_kinemactis_solutions = SearchKinematicsSolutions()
        while not rospy.is_shutdown():
            try:
                if rospy.get_param('/hiwonder_servo_manager/running') and rospy.get_param('/joint_states_publisher/running'):
                    break
            except:
                rospy.sleep(0.1)
        self.init_action()

        if rospy.get_param('~start', True):
            self.start_srv_callback(None)
            self.set_color_srv_callback(String('red'))

        self.mecanum_pub.publish(Twist())
        self.color_track()

    def set_color_srv_callback(self, msg):
        rospy.loginfo("start color track")

        msg_red = ColorDetect()
        msg_red.color_name = msg.data
        msg_red.detect_type = 'circle'
        res = rospy.ServiceProxy('/color_detect/set_param', SetColorDetectParam)([msg_red])
        if res.success and msg.data != '':
            print('start_track_' + msg_red.color_name)
        else:
            print('track_fail')

        return [True, 'set_color']

    def start_srv_callback(self, msg):
        rospy.loginfo("start color track")

        self.start = True

        return TriggerResponse(success=True)

    def stop_srv_callback(self, msg):
        rospy.loginfo('stop color track')

        self.start = False
        res = rospy.ServiceProxy('/color_detect/set_param', SetColorDetectParam)()
        if res.success:
            print('set color success')
        else:
            print('set color fail')

        return TriggerResponse(success=True)

    def init_action(self):
        res = self.search_kinemactis_solutions.solveIK((0, self.y_init, self.z_dis), 0, -90, 90)
        if res:
            joint_data = res[1]
            set_servos(self.joints_pub, 1500, ((1, 500), (2, joint_data['joint4']), (3, joint_data['joint3']), (4, joint_data['joint2']), (5, joint_data['joint1'])))
            rospy.sleep(1.8)

    def shutdown(self, signum, frame):
        self.running = False
        rospy.loginfo('shutdown')

    def get_color_callback(self, msg):
        if msg.data != []:
            if msg.data[0].radius > 10:
                self.center = msg.data[0]
            else:
                self.center = None 
        else:
            self.center = None

    def color_track(self):
        while self.running:
            if self.center is not None and self.start:
                self.pid_x.SetPoint = self.center.width/2 
                self.pid_x.update(self.center.x)
                self.x_dis += self.pid_x.output
                if self.x_dis < 200:
                    self.x_dis = 200
                if self.x_dis > 800:
                    self.x_dis = 800

                self.pid_z.SetPoint = self.center.height/2 
                self.pid_z.update(self.center.y)
                self.z_dis += self.pid_z.output
                if self.z_dis > 0.43:
                    self.z_dis = 0.43
                if self.z_dis < 0.33:
                    self.z_dis = 0.33
                res = self.search_kinemactis_solutions.solveIK((0, self.y_init, self.z_dis), 0, -5, 5)
                if res:
                    joint_data = res[1]
                    set_servos(self.joints_pub, 20, ((1, 500), (2, joint_data['joint4']), (3, joint_data['joint3']), (4, joint_data['joint2']), (5, self.x_dis)))
                    rospy.sleep(0.02)
                else:
                    set_servos(self.joints_pub, 20, ((5, self.x_dis), ))
                    rospy.sleep(0.02)
            else:
                rospy.sleep(0.01)
        
        rospy.signal_shutdown('shutdown')

if __name__ == "__main__":
    ColorTrackNode('color_track')
