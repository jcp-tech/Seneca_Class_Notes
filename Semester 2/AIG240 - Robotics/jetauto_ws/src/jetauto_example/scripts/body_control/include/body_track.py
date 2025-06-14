#!/usr/bin/env python
# encoding: utf-8
# @data:2022/11/07
# @author:aiden
# 人体跟踪
import cv2
import rospy
import numpy as np
import faulthandler
import mediapipe as mp
import jetauto_sdk.pid as pid
import jetauto_sdk.fps as fps
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

faulthandler.enable()

mp_pose = mp.solutions.pose
LEFT_SHOULDER = mp_pose.PoseLandmark.LEFT_SHOULDER
LEFT_HIP = mp_pose.PoseLandmark.LEFT_HIP

RIGHT_SHOULDER = mp_pose.PoseLandmark.RIGHT_SHOULDER
RIGHT_HIP = mp_pose.PoseLandmark.RIGHT_HIP

def get_body_center(h, w, landmarks):
    landmarks = np.array([(lm.x * w, lm.y * h) for lm in landmarks])
    center = ((landmarks[LEFT_HIP] + landmarks[LEFT_SHOULDER] + landmarks[RIGHT_HIP] + landmarks[RIGHT_SHOULDER])/4).astype(int)
    return center.tolist()

class BodyControlNode:
    def __init__(self, name):
        rospy.init_node(name, anonymous=True)
        self.name = name
        self.drawing = mp.solutions.drawing_utils
        self.body_detector = mp_pose.Pose(
            static_image_mode=False,
            model_complexity=0,
            min_tracking_confidence=0.5,
            min_detection_confidence=0.5)
       
        self.pid_d = pid.PID(0.03, 0, 0)
        #self.pid_d = pid.PID(0, 0, 0)
        
        self.pid_angular = pid.PID(0.0025, 0, 0.0005)
        #self.pid_angular = pid.PID(0, 0, 0)
        
        self.go_speed, self.turn_speed = 0.007, 0.04
        self.linear_x, self.angular = 0, 0

        self.fps = fps.FPS()  # fps计算器

        self.turn_left = False
        self.turn_right = False
        self.go_forward = False
        self.back = False
        self.next_frame = True
        self.depth_frame = None

        camera = rospy.get_param('/depth_camera/camera_name', 'astra_cam')
        self.image_sub = rospy.Subscriber('/%s/rgb/image_raw'%camera, Image, self.image_callback, queue_size=1) 
        self.depth_image_sub = rospy.Subscriber('/%s/depth/image_raw'%camera, Image, self.depth_image_callback, queue_size=1)
        self.mecanum_pub = rospy.Publisher('/jetauto_controller/cmd_vel', Twist, queue_size=1)
        rospy.sleep(0.2)
        self.mecanum_pub.publish(Twist())

    def depth_image_callback(self, depth_image):
        self.depth_frame = np.ndarray(shape=(depth_image.height, depth_image.width), dtype=np.uint16, buffer=depth_image.data)

    def image_callback(self, ros_image):
        rgb_image = np.ndarray(shape=(ros_image.height, ros_image.width, 3), dtype=np.uint8, buffer=ros_image.data)  # 将自定义图像消息转化为图像(convert the custom image message into image)  into)
        
        try:
            result_image = self.image_proc(np.copy(rgb_image))
        except BaseException as e:
            print(e)
            result_image = cv2.flip(cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR), 1)
        self.fps.update()
        result_image = self.fps.show_fps(result_image)
        cv2.imshow(self.name, result_image)
        key = cv2.waitKey(1)
        if key != -1:
            self.mecanum_pub.publish(Twist())
            rospy.signal_shutdown('shutdown')

    def image_proc(self, image):
        image_flip = cv2.flip(cv2.cvtColor(image, cv2.COLOR_RGB2BGR), 1)
        if self.next_frame:
            self.next_frame = False
            return image_flip
        else:
            self.next_frame = True
            results = self.body_detector.process(image)
            #print(results, results.pose_landmarks)
            
            twist = Twist()
            if results is not None and results.pose_landmarks is not None:
                h, w = image.shape[:-1]
                center = get_body_center(h, w, results.pose_landmarks.landmark)
                cv2.circle(image, tuple(center), 10, (255, 255, 0), -1) 
                #################
                roi_h, roi_w = 5, 5
                w_1 = center[0] - roi_w
                w_2 = center[0] + roi_w
                if w_1 < 0:
                    w_1 = 0
                if w_2 > w:
                    w_2 = w
                h_1 = center[1] - roi_h
                h_2 = center[1] + roi_h
                if h_1 < 0:
                    h_1 = 0
                if h_2 > h:
                    h_2 = h
                
                cv2.rectangle(image, (w_1, h_1), (w_2, h_2), (255, 255, 0), 2)
                roi = self.depth_frame[h_1:h_2, w_1:w_2]
                distances = roi[np.logical_and(roi > 0, roi < 40000)]
                if distances != []:
                    distance = int(np.mean(distances)/10)
                else:
                    distance = 0
                    #print(distance)
                ################
                if distance > 600 or distance < 100:
                    distance = 300
                self.pid_d.SetPoint = 300
                if abs(distance - 300) < 30:
                    distance = 300
                self.pid_d.update(distance)  # 更新pid(update pid)
                tmp = self.go_speed - self.pid_d.output
                self.linear_x = tmp
                if tmp > 0.2:
                    self.linear_x = 0.2
                if tmp < -0.2:
                    self.linear_x = -0.2
                if abs(tmp) < 0.008:
                    self.linear_x = 0
                #twist.linear.x = self.linear_x
                
                self.pid_angular.SetPoint = w/2
                if abs(center[0] - w/2) < 20:
                    center[0] = w/2
                self.pid_angular.update(center[0])  # 更新pid(update pid)
                tmp = self.turn_speed + self.pid_angular.output
                self.angular = tmp
                if tmp > 2:
                    self.angular = 2
                if tmp < -2:
                    self.angular = -2
                if abs(tmp) < 0.05:
                    self.angular = 0
                #if self.angular == 0:
                twist.linear.x = self.linear_x
                #else:
                twist.angular.z = self.angular
                result_image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
                '''
                self.drawing.draw_landmarks(
                    result_image,
                    results.pose_landmarks,
                    mp_pose.POSE_CONNECTIONS)
                '''
                self.mecanum_pub.publish(twist)
                return cv2.flip(result_image, 1)
            else:
                self.mecanum_pub.publish(twist)
                return image_flip

if __name__ == "__main__":
    print('\n******Press any key to exit!******')
    node = BodyControlNode('body_control')
    try:
        rospy.spin()
    except Exception as e:
        rospy.logerr(str(e))
