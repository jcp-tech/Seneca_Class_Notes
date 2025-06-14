#!/usr/bin/env python
# encoding: utf-8
# @data:2022/11/07
# @author:aiden
# 颜色分拣
import sys
import cv2
import rospy
import signal
import threading
import numpy as np
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger, TriggerResponse
from jetauto_interfaces.msg import ColorsInfo, ColorDetect, ROI
from jetauto_interfaces.srv import SetColorDetectParam, SetCircleROI
sys.path.append('/home/jetauto/jetauto_software/jetauto_arm_pc')
import action_group_controller as controller

class ColorSortingNode:
    def __init__(self, name):
        rospy.init_node(name, anonymous=True)
        self.name = name
        self.running = True
        self.center = None
        self.image = None
        #self.pick_roi = [290, 310, 310, 330] #[y_min, y_max, x_min, x_max]
        self.pick_roi = [349, 369, 344, 364]  #[y_min, y_max, x_min, x_max]
        self.start_pick = False
        self.target_color = ''
        self.count = 0
        signal.signal(signal.SIGINT, self.shutdown)
        
        rospy.Subscriber('/color_detect/color_info', ColorsInfo, self.get_color_callback)
        rospy.Subscriber('/color_detect/image_result/', Image, self.image_callback)
        rospy.Service(self.name + '/start', Trigger, self.start_srv_callback)  # 进入玩法
        rospy.Service(self.name + '/stop', Trigger, self.stop_srv_callback)  # 退出玩法
        controller.runAction('pick_init')
        
        self.debug = rospy.get_param('~debug', False)
        if self.debug:
            self.pick_roi = [20, 460, 20, 620]
            controller.runAction('pick_debug')
            rospy.sleep(5)
            controller.runAction('pick_init')
        if rospy.get_param('~start', True):
            self.start_srv_callback(None)
        
        threading.Thread(target=self.pick, daemon=True).start()
        self.color_track() 

    def start_srv_callback(self, msg):
        rospy.loginfo("start color sorting")

        roi = ROI()
        roi.x_min = self.pick_roi[2] - 20
        roi.x_max = self.pick_roi[3] + 20
        roi.y_min = self.pick_roi[0] - 20
        roi.y_max = self.pick_roi[1] + 20

        res = rospy.ServiceProxy('/color_detect/set_circle_roi', SetCircleROI)(roi)
        if res.success:
            print('set roi success')
        else:
            print('set roi fail')

        msg_red = ColorDetect()
        msg_red.color_name = 'red'
        msg_red.detect_type = 'circle'
        msg_green = ColorDetect()
        msg_green.color_name = 'green'
        msg_green.detect_type = 'circle'
        msg_blue = ColorDetect()
        msg_blue.color_name = 'blue'
        msg_blue.detect_type = 'circle'
        res = rospy.ServiceProxy('/color_detect/set_param', SetColorDetectParam)([msg_red, msg_green, msg_blue])
        if res.success:
            print('set color success')
        else:
            print('set color fail')
        self.start = True
         
        return TriggerResponse(success=True)
    
    def stop_srv_callback(self, msg):
        rospy.loginfo('stop color sorting')

        self.start = False
        res = rospy.ServiceProxy('/color_detect/set_param', SetColorDetectParam)()
        if res.success:
            print('set color success')
        else:
            print('set color fail')
        return TriggerResponse(success=True)   

    def shutdown(self, signum, frame):
        self.running = False
        rospy.loginfo('shutdown')

    def get_color_callback(self, msg):
        data = msg.data
        if data != []:
            if data[0].radius > 10:
                self.center = data[0]
                self.target_color = data[0].color
            else:
                self.target_color = ''
        else:
            self.target_color = ''

    def pick(self):
        while self.running:
            if self.start_pick:
                self.stop_srv_callback(Trigger())
                if self.target_color == 'red':
                    controller.runAction('pick')
                    controller.runAction('place_center')
                elif self.target_color == 'green':
                    controller.runAction('pick')
                    controller.runAction('place_left')
                elif self.target_color == 'blue':
                    controller.runAction('pick')
                    controller.runAction('place_right')
                self.start_pick = False
                self.start_srv_callback(Trigger())
            else:
                rospy.sleep(0.01)

    def color_track(self):
        while self.running:
            if self.target_color != '' and self.start:
                if self.pick_roi[2] < self.center.x < self.pick_roi[3] and self.pick_roi[0] < self.center.y < self.pick_roi[1] and not self.start_pick and not self.debug:
                    self.count += 1
                    if self.count > 10:
                        self.count = 0
                        self.start_pick = True
                elif self.debug:
                    print(self.center.y - 10, self.center.y + 10, self.center.x - 10, self.center.x + 10)
                    cv2.rectangle(self.image, (self.center.x - 25, self.center.y - 25,), (self.center.x + 25, self.center.y + 25), (0, 0, 255), 2)
            if self.image is not None:
                if not self.start_pick and not self.debug:
                    cv2.rectangle(self.image, (self.pick_roi[2] - 25, self.pick_roi[0] - 25), (self.pick_roi[3] + 25, self.pick_roi[1] + 25), (0, 255, 255), 2)
                cv2.imshow('image', self.image)
                key = cv2.waitKey(1)
                if key != -1:
                    self.running = False
        controller.runAction('init')
        rospy.signal_shutdown('shutdown')

    def image_callback(self, ros_image):
        self.image = np.ndarray(shape=(ros_image.height, ros_image.width, 3), dtype=np.uint8, buffer=ros_image.data) # 原始 RGB 画面

if __name__ == "__main__":
    ColorSortingNode('color_sorting')
