#!/usr/bin/env python
# encoding: utf-8
# @data:2022/11/07
# @author:aiden
# 无人驾驶
import os
import cv2
import math
import yaml
import rospy
import threading
import numpy as np
import jetauto_sdk.pid as pid
import jetauto_sdk.misc as misc
from std_srvs.srv import Trigger
import jetauto_sdk.common as common
from sensor_msgs.msg import Image
import geometry_msgs.msg as geo_msg
from jetauto_interfaces.msg import ObjectsInfo
from hiwonder_servo_msgs.msg import MultiRawIdPosDur
from hiwonder_servo_controllers.bus_servo_control import set_servos

def get_yaml_data(yaml_file):
    yaml_file = open(yaml_file, 'r', encoding='utf-8')
    file_data = yaml_file.read()
    yaml_file.close()
    
    data = yaml.load(file_data, Loader=yaml.FullLoader)
    
    return data

lab_data = get_yaml_data("/home/jetauto/jetauto_software/lab_tool/lab_config.yaml")

class LineFollower:
    def __init__(self, color):
        self.target_color = color
        self.rois = ((450, 480, 0, 320, 0.7), (390, 420, 0, 320, 0.2), (330, 360, 0, 320, 0.1))
        self.weight_sum = 1.0

    @staticmethod
    def get_area_max_contour(contours, threshold=100):
        '''
        获取最大面积对应的轮廓
        :param contours:
        :param threshold:
        :return:
        '''
        contour_area = zip(contours, tuple(map(lambda c: math.fabs(cv2.contourArea(c)), contours)))
        contour_area = tuple(filter(lambda c_a: c_a[1] > threshold, contour_area))
        if len(contour_area) > 0:
            max_c_a = max(contour_area, key=lambda c_a: c_a[1])
            return max_c_a
        return None
    
    def get_max_y(self, image):
        h, w = image.shape[:2]
        roi = image[:, int(w/2):w]
        flip_binary = cv2.flip(roi, 0)
        max_y = cv2.minMaxLoc(flip_binary)[-1][1]
        return h - max_y

    def get_max_x(self, image):
        h, w = image.shape[:2]
        roi = image[int(h/2):h, 0:int(w/2)]
        #cv2.imshow('roi', roi)
        #rotate_binary = cv2.rotate(roi, cv2.ROTATE_90_COUNTERCLOCKWISE)
        flip_binary = cv2.flip(roi, 1)
        #cv2.imshow('1', flip_binary)
        (x, y) = cv2.minMaxLoc(flip_binary)[-1]
        return 320 - x, y + 240

    def get_binary(self, image):
        img_lab = cv2.cvtColor(image, cv2.COLOR_RGB2LAB)  # rgb转lab
        img_blur = cv2.GaussianBlur(img_lab, (3, 3), 3)  # 高斯模糊去噪
        mask = cv2.inRange(img_blur, tuple(lab_data['lab']['Stereo'][self.target_color]['min']), tuple(lab_data['lab']['Stereo'][self.target_color]['max']))  # 二值化
        eroded = cv2.erode(mask, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))  # 腐蚀
        dilated = cv2.dilate(eroded, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))  # 膨胀

        return dilated

    def __call__(self, image, result_image):
        centroid_sum = 0
        h, w = image.shape[:2]
        max_center_x = -1
        center_x = []
        for roi in self.rois:
            blob = image[roi[0]:roi[1], roi[2]:roi[3]]  # 截取roi
            contours = cv2.findContours(blob, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_TC89_L1)[-2]  # 找轮廓
            max_contour_area = self.get_area_max_contour(contours, 30)  # 获取最大面积对应轮廓
            if max_contour_area is not None:
                rect = cv2.minAreaRect(max_contour_area[0])  # 最小外接矩形
                box = np.int0(cv2.boxPoints(rect))  # 四个角
                for j in range(4):
                    box[j, 1] = box[j, 1] + roi[0]
                cv2.drawContours(result_image, [box], -1, (0, 255, 255), 2)  # 画出四个点组成的矩形

                # 获取矩形对角点
                pt1_x, pt1_y = box[0, 0], box[0, 1]
                pt3_x, pt3_y = box[2, 0], box[2, 1]
                # 线的中心点
                line_center_x, line_center_y = (pt1_x + pt3_x) / 2, (pt1_y + pt3_y) / 2

                cv2.circle(result_image, (int(line_center_x), int(line_center_y)), 5, (0, 0, 255), -1)  # 画出中心点
                center_x.append(line_center_x)
            else:
                center_x.append(-1)
        for i in range(len(center_x)):
            if center_x[i] != -1:
                if center_x[i] > max_center_x:
                    max_center_x = center_x[i]
                centroid_sum += center_x[i] * self.rois[i][-1]
        if centroid_sum == 0:
            return result_image, None, max_center_x
        center_pos = centroid_sum / self.weight_sum  # 按比重计算中心点
        angle = math.degrees(-math.atan((center_pos - (w / 2.0)) / (h / 2.0)))
        
        return result_image, angle, max_center_x

class LineFollowingNode:
    def __init__(self, name):
        rospy.init_node(name, anonymous=True)
        self.name = name
        self.is_running = True
        self.pid = pid.PID(0.01, 0.0, 0.0)
        self.center = None
        self.stop = False
        self.red = False
        self.park = False
        self.green = False
        self.image = None
        self.min_distance = 0
        self.right_area = -1
        self.park_pos = (0, 0)
        self.count_red = 0
        self.count_green = 0
        self.count_turn = 0
        self.count_go = 0
        self.objects_info = []
        self.colors = common.Colors()
        self.start_turn = False
        self.start_park = False
        self.turn_right = False
        self.start_slow_down = False
        self.count_cross_walk = 0
        self.cross_walk_distance = 0.4
        self.follower = LineFollower("yellow") 
        self.classes = rospy.get_param('/yolov5_node/classes')
        self.mecanum_pub = rospy.Publisher('/jetauto_controller/cmd_vel', geo_msg.Twist, queue_size=1)  # 底盘控制
        self.result_publisher = rospy.Publisher(self.name + '/image_result', Image, queue_size=1)  # 图像处理结果发布
        self.joints_pub = rospy.Publisher('servo_controllers/port_id_1/multi_id_pos_dur', MultiRawIdPosDur, queue_size=1)  # 舵机控制
        depth_camera = rospy.get_param('/depth_camera/camera_name', 'camera')  # 获取参数
        rospy.Subscriber('/%s/rgb/image_raw'%depth_camera, Image, self.image_callback)  # 摄像头订阅
        rospy.Subscriber('/yolov5/object_detect', ObjectsInfo, self.get_object_callback)
        if not rospy.get_param('~only_line_follow', False):
            while not rospy.is_shutdown():
                try:
                    if rospy.get_param('/yolov5_node/start'):
                        break
                except:
                    rospy.sleep(0.1)
            rospy.ServiceProxy('/yolov5/start', Trigger)()
        while not rospy.is_shutdown():
            try:
                if rospy.get_param('/hiwonder_servo_manager/running') and rospy.get_param('/joint_states_publisher/running'):
                    break
            except:
                rospy.sleep(0.1)
        set_servos(self.joints_pub, 1000, ((5, 500), ))  # 舵机中位
        self.mecanum_pub.publish(geo_msg.Twist())
        self.image_proc()

    def image_callback(self, ros_image):
        rgb_image = np.ndarray(shape=(ros_image.height, ros_image.width, 3), dtype=np.uint8, buffer=ros_image.data) # 原始 RGB 画面
        self.image = rgb_image

    def park_action(self):
        twist = geo_msg.Twist()
        twist.linear.x = 0.15
        self.mecanum_pub.publish(twist)
        rospy.sleep(0.25/0.15)
        self.mecanum_pub.publish(geo_msg.Twist())
        twist = geo_msg.Twist()
        twist.linear.y = -0.15
        self.mecanum_pub.publish(twist)
        rospy.sleep(0.25/0.15)
        self.mecanum_pub.publish(geo_msg.Twist())

    def image_proc(self):
        if self.image is not None:
            while self.is_running:
                twist = geo_msg.Twist()
                binary_image = self.follower.get_binary(self.image)
                if 600 < self.park_pos[0] and 150 < self.park_pos[1] and not self.start_park:
                    self.mecanum_pub.publish(geo_msg.Twist())
                    self.start_park = True
                    threading.Thread(target=self.park_action).start()
                    self.stop = True
                if 0 < self.park_pos[0]:
                    self.park = True
                if 1500 < self.right_area:
                    self.turn_right = True
                if 440 < self.min_distance < 460:
                    self.count_cross_walk += 1
                    if self.count_cross_walk == 5:
                        self.count_cross_walk = 0
                        self.start_slow_down = True
                        self.count_slow_down = rospy.get_time()
                else:
                    self.count_cross_walk = 0
                if self.start_slow_down:
                    if self.red:
                        self.mecanum_pub.publish(geo_msg.Twist())
                        self.stop = True
                    elif self.green:
                        twist.linear.x = 0.1 
                        self.stop = False
                    else:
                        twist.linear.x = 0.1
                        if rospy.get_time() - self.count_slow_down > self.cross_walk_distance/twist.linear.x:
                            self.start_slow_down = False
                else:
                    twist.linear.x = 0.15
                try:
                    if self.turn_right:
                        y = self.follower.get_max_y(binary_image)
                        roi = [(0, y), (640, y), (640, 0), (0, 0)]
                        cv2.fillPoly(binary_image, [np.array(roi)], [0, 0, 0])
                        min_x = cv2.minMaxLoc(binary_image)[-1][0]
                        cv2.line(binary_image, (min_x, y), (640, y), (255, 255, 255), 10)
                    elif self.park:
                        x, y = self.follower.get_max_x(binary_image)
                        #print(x, y)
                        #line_x = self.follower(binary_image, self.image.copy())[-1]
                        cv2.line(binary_image, (0, 450), (x, y), (255, 255, 255), 10)
                    result_image, angle, x = self.follower(binary_image, self.image.copy())
                    if x != -1 and not self.stop:
                        if x > 200:
                            if self.turn_right:
                                self.start_turn = True
                            twist.angular.z = -0.45
                        else:
                            if self.start_turn:
                                self.count_turn += 1
                            else:
                                self.count_turn = 0
                            if self.count_turn > 6:
                                self.count_turn = 0
                                self.start_turn = False
                                self.turn_right = False
                            self.pid.SetPoint = 80
                            self.pid.update(x)
                            twist.angular.z = misc.set_range(self.pid.output, -0.8, 0.8)
                        self.mecanum_pub.publish(twist)
                    else:
                        self.pid.clear()
                except Exception as e:
                    result_image = self.image
                    rospy.logerr(str(e))
                bgr_image = cv2.cvtColor(result_image, cv2.COLOR_RGB2BGR)
                '''
                for i in self.objects_info:
                    color = self.colors(self.classes.index(i.class_name), True)
                    common.plot_one_box(
                    i.box,
                    bgr_image,
                    color=color,
                    label="{}:{:.2f}".format(
                        i.class_name, i.score
                    ),
                )
                '''
                cv2.imshow('result', bgr_image)
                key = cv2.waitKey(1)
                if key != -1:
                    self.is_running = False
            #ros_image.data = result_image.tostring()
            #self.result_publisher.publish(ros_image)
        else:
            rospy.sleep(0.01)
        self.mecanum_pub.publish(geo_msg.Twist())

    def get_object_callback(self, msg):
        self.objects_info = msg.objects
        if self.objects_info == []:
            self.red = False
            self.green = False
            self.right_area = -1
            self.park_x = -1
            self.center = None
            self.min_distance = 0
        else:
            min_distance = 0
            for i in self.objects_info:
                class_name = i.class_name
                self.center = (int((i.box[0] + i.box[2])/2), int((i.box[1] + i.box[3])/2))
                if class_name == 'right':
                    self.right_area = abs((i.box[0] - i.box[2])*(i.box[1] - i.box[3]))
                if class_name == 'crosswalk':
                    if self.center[1] > min_distance:
                        min_distance = self.center[1]
                if class_name == 'park':
                    self.park_pos = self.center
                if class_name == 'red':
                    self.red = True
                    self.count_red = 0
                else:
                    self.count_red += 1
                if self.count_red == 2:
                    self.count_red = 0
                    self.red = False
                
                if class_name == 'green':
                    self.green = True
                    self.count_green = 0
                else:
                    self.count_green += 1
                if self.count_green == 2:
                    #self.count_green = 0
                    self.green = False
        
            self.min_distance = min_distance

if __name__ == "__main__":
    LineFollowingNode('line_following')
