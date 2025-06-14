#!/usr/bin/env python
# encoding: utf-8
# @data:2022/11/18
# @author:aiden
# 追踪拾取
import os
import sys
import cv2
import math
import rospy
import threading
import numpy as np
from jetauto_sdk.pid import PID
import jetauto_sdk.misc as misc
from sensor_msgs.msg import Image
import jetauto_sdk.common as common
from geometry_msgs.msg import Twist
from xf_mic_asr_offline import voice_play
from jetauto_interfaces.msg import Pose2D
from jetauto_interfaces.srv import SetPose2D
from std_srvs.srv import Trigger, TriggerResponse
from hiwonder_servo_msgs.msg import MultiRawIdPosDur
from hiwonder_servo_controllers import bus_servo_control
sys.path.append('/home/jetauto/jetauto_software/jetauto_arm_pc')
import action_group_controller as controller

if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)

debug = False
start_pick = False
start_place = False
target_color = ""
linear_base_speed = 0.007
angular_base_speed = 0.03

yaw_pid = PID(P=0.007, I=0, D=0.000)
#linear_pid = PID(P=0, I=0, D=0)
linear_pid = PID(P=0.001, I=0, D=0)
#angular_pid = PID(P=0, I=0, D=0)
angular_pid = PID(P=0.003, I=0, D=0)

linear_speed = 0
angular_speed = 0
yaw_angle = 90

stop_x = 287
stop_y = 388
stop = True

d_y = 10
d_x = 10

pick = False
place = False

broadcast_status = ''
status = "approach"
count_stop = 0
count_turn = 0

lab_data = common.get_yaml_data("/home/jetauto/jetauto_software/lab_tool/lab_config.yaml")

def cancel_callback(msg):
    global start_pick, start_place

    start_pick = False
    start_place = False

    return TriggerResponse(success=True)

def start_pick_callback(msg):
    global start_pick, yaw_angle, stop_y, stop, status, target_color, broadcast_status
    global linear_speed, angular_speed
    global d_x, d_y
    global pick, place
    global count_turn, count_stop

    rospy.loginfo("start pick")
    rospy.set_param('~status', 'start_pick')

    linear_speed = 0
    angular_speed = 0
    yaw_angle = 90

    stop_y = 388
    stop = True

    d_y = 10
    d_x = 10

    pick = False
    place = False

    status = "approach"
    target_color = 'blue'
    broadcast_status = 'find_target'
    count_stop = 0
    count_turn = 0

    linear_pid.clear()
    angular_pid.clear()
    start_pick = True

    return TriggerResponse(success=True)

def start_place_callback(msg):
    global start_place, stop_y, stop, target_color, broadcast_status
    global linear_speed, angular_speed
    global d_x, d_y
    global pick, place

    rospy.loginfo("start place")
    rospy.set_param('~status', 'start_place')

    linear_speed = 0
    angular_speed = 0
    d_y = 30
    d_x = 30
    stop_y = 330
    stop = True
    pick = False
    place = False
    target_color = 'red'
    broadcast_status = 'mission_completed'

    linear_pid.clear()
    angular_pid.clear()
    start_place = True

    return TriggerResponse(success=True)

# 颜色识别
size = (320, 240)
def colorDetect(img):
    img_h, img_w = img.shape[:2]
    frame_resize = cv2.resize(img, size, interpolation=cv2.INTER_NEAREST)
    frame_gb = cv2.GaussianBlur(frame_resize, (3, 3), 3)
    frame_lab = cv2.cvtColor(frame_gb, cv2.COLOR_BGR2LAB)  # 将图像转换到LAB空间
    frame_mask = cv2.inRange(frame_lab, tuple(lab_data['lab']['Mono'][target_color]['min']), tuple(lab_data['lab']['Mono'][target_color]['max']))  # 对原图像和掩模进行位运算

    eroded = cv2.erode(frame_mask, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))  # 腐蚀
    dilated = cv2.dilate(eroded, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))  # 膨胀
    
    contours = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]  # 找出轮廓

    center_x, center_y, angle = -1, -1, -1
    if len(contours) != 0:
        areaMaxContour, area_max = common.get_area_max_contour(contours, 10)  # 找出最大轮廓
        if areaMaxContour is not None:
            if 100 < area_max:  # 有找到最大面积
                rect = cv2.minAreaRect(areaMaxContour)#最小外接矩形
                angle = rect[2]
                box = np.int0(cv2.boxPoints(rect))#最小外接矩形的四个顶点
                for j in range(4):
                    box[j, 0] = int(misc.val_map(box[j, 0], 0, size[0], 0, img_w))
                    box[j, 1] = int(misc.val_map(box[j, 1], 0, size[1], 0, img_h))

                cv2.drawContours(img, [box], -1, (0,255,255), 2)#画出四个点组成的矩形
                #获取矩形的对角点
                ptime_start_x, ptime_start_y = box[0, 0], box[0, 1]
                pt3_x, pt3_y = box[2, 0], box[2, 1]
                radius = abs(ptime_start_x - pt3_x)
                center_x, center_y = int((ptime_start_x + pt3_x) / 2), int((ptime_start_y + pt3_y) / 2)#中心点       
                cv2.circle(img, (center_x, center_y), 5, (0, 255, 255), -1)#画出中心点

    return center_x, center_y, angle

def action_thread():
    global pick, place, start_pick, start_place, broadcast_status
    while True:
        if pick:
            mecnum_pub.publish(Twist())
            rospy.sleep(0.5)
            controller.runAction('navigation_pick')
            if broadcast and broadcast_status == 'crawl_succeeded':
                broadcast_status = 'mission_completed'
                voice_play.play('crawl_succeeded', language=language)
            rospy.set_param('~status', 'pick_finish')
            start_pick = False
            pick = False
            if broadcast:
                pose = Pose2D()
                pose.x = place_position[0] 
                pose.y = place_position[1]
                pose.roll = place_position[2]
                pose.pitch = place_position[3]
                pose.yaw = place_position[4]
                rospy.ServiceProxy('/navigation_transport/place', SetPose2D)(pose)
            print('pick finish')
        elif place:
            mecnum_pub.publish(Twist())
            rospy.sleep(1)
            controller.runAction('navigation_place')
            if broadcast and broadcast_status == 'mission_completed':
                broadcast_status = ''
                voice_play.play('mission_completed', language=language)
            rospy.set_param('~status', 'place_finish')
            start_place = False
            place = False
            print('place finish')
        else:
            rospy.sleep(0.01)

def pick_handle(usb_cam_img):
    global pick, count_turn, count_stop, angular_speed, linear_speed, status, d_x, d_y, broadcast_status

    twist = Twist()
    if not pick or debug:
        object_center_x, object_center_y, object_angle = colorDetect(usb_cam_img)  # 获取物体颜色的中心和角度
        if debug:
            print(object_center_x, object_center_y)  # 打印当前物体离中心的像素距离
        elif object_center_x > 0:
            if broadcast and broadcast_status == 'find_target':
                broadcast_status = 'crawl_succeeded'
                voice_play.play('find_target', language=language)
            ########电机pid处理#########
            # 以图像的中心点的x，y坐标作为设定的值，以当前x，y坐标作为输入#
            linear_pid.SetPoint = stop_y
            if abs(object_center_y - stop_y) <= d_y:
                object_center_y = stop_y
            if status != "align":
                linear_pid.update(object_center_y)  # 更新pid
                tmp = linear_base_speed + linear_pid.output

                linear_speed = tmp
                if tmp > 0.15:
                    linear_speed = 0.15
                if tmp < -0.15:
                    linear_speed = -0.15
                if abs(tmp) <= 0.0075:
                    linear_speed = 0

            angular_pid.SetPoint = stop_x
            if abs(object_center_x - stop_x) <= d_x:
                object_center_x = stop_x
            if status != "align":
                angular_pid.update(object_center_x)  # 更新pid
                tmp = angular_base_speed + angular_pid.output

                angular_speed = tmp
                if tmp > 1.2:
                    angular_speed = 1.2
                if tmp < -1.2:
                    angular_speed = -1.2
                if abs(tmp) <= 0.035:
                    angular_speed = 0

            if abs(linear_speed) == 0 and abs(angular_speed) == 0:
                count_turn += 1
                if count_turn > 5:
                    count_turn = 5
                    status = "align"
                    if object_angle > 25:
                        yaw_pid.SetPoint = 90
                        if abs(object_angle - 90) <= 1:
                            object_angle = 90
                    else:
                        yaw_pid.SetPoint = 0
                        if abs(object_angle - 0) <= 1:
                            object_angle = 0
                    yaw_pid.update(object_angle)  # 更新pid
                    tmp = yaw_pid.output
                    yaw_angle = tmp
                    if tmp > 1:
                        yaw_angle = 1
                    if tmp < -1:
                        yaw_angle = -1
                    if 90 > object_angle > 25:
                        count_stop = 0
                        twist.linear.y = 2 * 0.3 * math.sin(yaw_angle / 2)
                        twist.angular.z = -yaw_angle
                    elif 25 >= object_angle > 0:
                        count_stop = 0
                        twist.linear.y = -2 * 0.3 * math.sin(yaw_angle / 2)
                        twist.angular.z = yaw_angle
                    else:
                        count_stop += 1
                        if count_stop > 10:
                            d_x = 5
                            d_y = 5
                            status = "adjust"
                        if count_stop > 25:
                            pick = True
            else:
                count_turn = 0
                if status != 'align':
                    twist.linear.x = linear_speed
                    twist.angular.z = angular_speed

    mecnum_pub.publish(twist)

    return usb_cam_img

def place_handle(usb_cam_img):
    global place, angular_speed, linear_speed

    twist = Twist()
    if not place or debug:
        object_center_x, object_center_y, object_angle = colorDetect(usb_cam_img)  # 获取物体颜色的中心和角度
        if debug:
            print(object_center_x, object_center_y)  # 打印当前物体离中心的像素距离
        elif object_center_x > 0:
            ########电机pid处理#########
            # 以图像的中心点的x，y坐标作为设定的值，以当前x，y坐标作为输入#
            linear_pid.SetPoint = stop_y
            if abs(object_center_y - stop_y) <= d_y:
                object_center_y = stop_y
            linear_pid.update(object_center_y)  # 更新pid
            tmp = linear_base_speed + linear_pid.output

            linear_speed = tmp
            if tmp > 0.15:
                linear_speed = 0.15
            if tmp < -0.15:
                linear_speed = -0.15
            if abs(tmp) <= 0.0075:
                linear_speed = 0

            angular_pid.SetPoint = stop_x
            if abs(object_center_x - stop_x) <= d_x:
                object_center_x = stop_x

            angular_pid.update(object_center_x)  # 更新pid
            tmp = angular_base_speed + angular_pid.output

            angular_speed = tmp
            if tmp > 1.2:
                angular_speed = 1.2
            if tmp < -1.2:
                angular_speed = -1.2
            if abs(tmp) <= 0.035:
                angular_speed = 0

            if abs(linear_speed) == 0 and abs(angular_speed) == 0:
                place = True
            else:
                twist.linear.x = linear_speed
                twist.angular.z = angular_speed

    mecnum_pub.publish(twist)

    return usb_cam_img

def image_callback(ros_image):
    global place, stop

    rgb_image = np.ndarray(shape=(ros_image.height, ros_image.width, 3), dtype=np.uint8, buffer=ros_image.data)  # 将自定义图像消息转化为图像
    if start_pick:
        stop = True
        result_image = pick_handle(cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR))
    elif start_place:
        stop = True
        if place_without_color:
            place = True
            result_image = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR)
        else:
            result_image = place_handle(cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR))
    else:
        if stop:
            stop = False
            mecnum_pub.publish(Twist())
        result_image = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR)
    ros_image = common.cv2_image2ros(result_image)
    image_pub.publish(ros_image)

if __name__ == '__main__':
    rospy.init_node('automatic_pick', anonymous=True)

    rospy.set_param('~status', 'start')
    broadcast = rospy.get_param('~broadcast', False)
    language = os.environ['LANGUAGE']

    joints_pub = rospy.Publisher('/servo_controllers/port_id_1/multi_id_pos_dur', MultiRawIdPosDur, queue_size=1)
    mecnum_pub = rospy.Publisher('/jetauto_controller/cmd_vel', Twist, queue_size=1)
    image_pub = rospy.Publisher('/automatic_pick/image_result', Image, queue_size=1)
    image_topic = rospy.get_param('~image_topic', '/usb_cam/image_rect_color')
    rospy.Subscriber(image_topic, Image, image_callback)
    
    rospy.Service('/automatic_pick/pick', Trigger, start_pick_callback)
    rospy.Service('/automatic_pick/place', Trigger, start_place_callback)
    rospy.Service('/automatic_pick/cancel', Trigger, cancel_callback)
    
    place_position = rospy.get_param('~place_position', [1.0, 1.0, 0, 0, 0])
    place_without_color = rospy.get_param('~place_without_color', True)
    debug = rospy.get_param('~debug', False)

    rospy.sleep(0.2)
    while not rospy.is_shutdown():
        try:
            if rospy.get_param('/hiwonder_servo_manager/running') and rospy.get_param(
                    '/joint_states_publisher/running'):
                break
        except:
            rospy.sleep(0.1)
    bus_servo_control.set_servos(joints_pub, 2000, ((1, 200), (2, 215), (3, 15), (4, 700), (5, 500)))
    mecnum_pub.publish(Twist())
    threading.Thread(target=action_thread, daemon=True).start()
    try:
        rospy.spin()
    except Exception as e:
        mecnum_pub.publish(Twist())
        rospy.logerr(str(e))
