#!/usr/bin/env python
# encoding: utf-8
# @data:2022/11/07
# @author:aiden
# 跌倒检测
import cv2
import rospy
import threading
import numpy as np
import faulthandler
import mediapipe as mp
import jetauto_sdk.fps as fps
from jetauto_sdk import buzzer
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

faulthandler.enable()

mp_pose = mp.solutions.pose

LEFT_SHOULDER = mp_pose.PoseLandmark.LEFT_SHOULDER
LEFT_ELBOW = mp_pose.PoseLandmark.LEFT_ELBOW
LEFT_WRIST = mp_pose.PoseLandmark.LEFT_WRIST
LEFT_HIP = mp_pose.PoseLandmark.LEFT_HIP

RIGHT_SHOULDER = mp_pose.PoseLandmark.RIGHT_SHOULDER
RIGHT_ELBOW = mp_pose.PoseLandmark.RIGHT_ELBOW
RIGHT_WRIST = mp_pose.PoseLandmark.RIGHT_WRIST
RIGHT_HIP = mp_pose.PoseLandmark.RIGHT_HIP

LEFT_KNEE = mp_pose.PoseLandmark.LEFT_KNEE
LEFT_ANKLE = mp_pose.PoseLandmark.LEFT_ANKLE

RIGHT_KNEE = mp_pose.PoseLandmark.RIGHT_KNEE
RIGHT_ANKLE = mp_pose.PoseLandmark.RIGHT_ANKLE

def get_joint_landmarks(img, landmarks):
    """
    将landmarks从medipipe的归一化输出转为像素坐标(Convert landmarks from medipipe's normalized output to pixel coordinates)
    :param img: 像素坐标对应的图片(picture corresponding to pixel coordinate)
    :param landmarks: 归一化的关键点(normalized keypoint)
    :return:
    """
    h, w, _ = img.shape
    landmarks = [(lm.x * w, lm.y * h) for lm in landmarks]
    return np.array(landmarks)

def height_cal(landmarks):
    y = []
    for i in landmarks:
        y.append(i[1])
    height = sum(y)/len(y)

    return height

class FallDownDetectNode:
    def __init__(self, name):
        rospy.init_node(name, anonymous=True)
        self.name = name
        self.drawing = mp.solutions.drawing_utils
        self.body_detector = mp_pose.Pose(
            static_image_mode=False,
            model_complexity=0,
            min_tracking_confidence=0.5,
            min_detection_confidence=0.5)
        
        self.fps = fps.FPS()  # fps计算器
        
        self.fall_down_count = []
        self.move_finish = True
        self.stop_flag = False

        camera = rospy.get_param('/depth_camera/camera_name', 'camera')
        self.image_sub = rospy.Subscriber('/%s/rgb/image_raw'%camera, Image, self.image_callback, queue_size=1) 
        self.mecanum_pub = rospy.Publisher('/jetauto_controller/cmd_vel', Twist, queue_size=1)
        rospy.sleep(0.2)
        buzzer.off()
        self.mecanum_pub.publish(Twist())

    def image_callback(self, ros_image):
        rgb_image = np.ndarray(shape=(ros_image.height, ros_image.width, 3), dtype=np.uint8, buffer=ros_image.data)  # 将自定义图像消息转化为图像(convert the custom image message into image)age))
        
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

    def move(self):
        for i in range(5):
            twist = Twist()
            twist.angular.z = 2
            self.mecanum_pub.publish(twist)
            rospy.sleep(0.2)
            twist = Twist()
            twist.angular.z = -2
            self.mecanum_pub.publish(twist)
            rospy.sleep(0.2)
        self.mecanum_pub.publish(Twist())
        self.stop_flag =True
        self.move_finish = True

    def buzzer_warn(self):
        if not self.stop_flag:
            while not self.stop_flag:
                buzzer.on()
                rospy.sleep(0.05)
                buzzer.off()
                rospy.sleep(0.1)
        else:
            buzzer.on()
            rospy.sleep(0.1)
            buzzer.off()

    def image_proc(self, image):
        image_flip = cv2.flip(cv2.cvtColor(image, cv2.COLOR_RGB2BGR), 1)
        results = self.body_detector.process(image)
        if results is not None and results.pose_landmarks:
            if self.move_finish:
                landmarks = get_joint_landmarks(image, results.pose_landmarks.landmark)
                h = height_cal(landmarks)
                if h > 240:
                    self.fall_down_count.append(1)
                else:
                    self.fall_down_count.append(0)
                if len(self.fall_down_count) == 3:
                    count = sum(self.fall_down_count)
                    
                    self.fall_down_count = []
                    if self.stop_flag:
                        if count <= 1:
                            threading.Thread(target=self.buzzer_warn).start()
                            self.stop_flag = False
                    else:
                        if count > 1:
                            self.move_finish = False
                            threading.Thread(target=self.buzzer_warn).start()
                            threading.Thread(target=self.move).start()

            result_image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
            self.drawing.draw_landmarks(
                result_image,
                results.pose_landmarks,
                mp_pose.POSE_CONNECTIONS)
            return cv2.flip(result_image, 1)
        else:
            return image_flip

if __name__ == "__main__":
    print('\n******Press any key to exit!******')
    node = FallDownDetectNode('fall_down_detect')
    try:
        rospy.spin()
    except Exception as e:
        rospy.logerr(str(e))
