#!/usr/bin/env python
# encoding: utf-8
# @data:2022/11/18
# @author:aiden
# yolov5目标检测
import os
import cv2
import rospy
import signal
import numpy as np
import jetauto_sdk.fps as fps
from std_msgs.msg import Header
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger, TriggerResponse
from jetauto_interfaces.msg import ObjectInfo, ObjectsInfo

MODE_PATH = os.path.split(os.path.realpath(__file__))[0]

class Yolov5Node:
    def __init__(self, name):
        rospy.init_node(name, anonymous=True)
        self.name = name
        
        self.bgr_image = None
        self.running = True
        self.start = False

        signal.signal(signal.SIGINT, self.shutdown)
        
        self.fps = fps.FPS()  # fps计算器
        engine = rospy.get_param('~engine')
        
        if engine == 'traffic_signs_640s_7_0.engine':
            from yolov5_trt_7_0 import YoLov5TRT, colors, plot_one_box
        else:
            from yolov5_trt import YoLov5TRT, colors, plot_one_box
        
        self.colors = colors 
        self.plot_one_box = plot_one_box
        lib = rospy.get_param('~lib')
        conf_thresh = rospy.get_param('~conf_thresh', 0.8)
        self.classes = rospy.get_param('~classes')
        
        self.yolov5 = YoLov5TRT(os.path.join(MODE_PATH, engine), os.path.join(MODE_PATH, lib), self.classes, conf_thresh)
        rospy.Service('/yolov5/start', Trigger, self.start_srv_callback)  # 进入玩法
        rospy.Service('/yolov5/stop', Trigger, self.stop_srv_callback)  # 退出玩法
        if rospy.get_param('~use_depth_cam', True):
            camera = rospy.get_param('/depth_camera/camera_name', 'camera')
            self.image_sub = rospy.Subscriber('/%s/rgb/image_raw'%camera, Image, self.image_callback, queue_size=1)
        else:
            camera = rospy.get_param('/usb_cam_name', 'usb_cam')
            self.image_sub = rospy.Subscriber('/%s/image_raw'%camera, Image, self.image_callback, queue_size=1)
        self.object_pub = rospy.Publisher('/yolov5/object_detect', ObjectsInfo, queue_size=1)
        self.result_image_pub = rospy.Publisher('/yolov5/object_image', Image, queue_size=1)
        rospy.set_param('~start', True)
        self.image_proc()

    def start_srv_callback(self, msg):
        rospy.loginfo("start yolov5 detect")

        self.start = True

        return TriggerResponse(success=True)

    def stop_srv_callback(self, msg):
        rospy.loginfo('stop yolov5 detect')

        self.start = False

        return TriggerResponse(success=True)

    def image_callback(self, ros_image):
        rgb_image = np.ndarray(shape=(ros_image.height, ros_image.width, 3), dtype=np.uint8, buffer=ros_image.data)  # 将自定义图像消息转化为图像
        self.bgr_image = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR)
   
    def shutdown(self, signum, frame):
        self.running = False
        rospy.loginfo('shutdown')

    def image_proc(self):
        while self.running:
            if self.bgr_image is not None:
                image = self.bgr_image
                try:
                    if self.start:
                        objects_info = []
                        boxes, scores, classid = self.yolov5.infer(image)
                        for box, cls_conf, cls_id in zip(boxes, scores, classid):
                            color = self.colors(cls_id, True)

                            object_info = ObjectInfo()
                            object_info.class_name = self.classes[cls_id]
                            object_info.box = box.astype(int)
                            object_info.score = cls_conf

                            objects_info.append(object_info)

                            self.plot_one_box(
                            box,
                            image,
                            color=color,
                            label="{}:{:.2f}".format(
                                self.classes[cls_id], cls_conf
                            ),
                        )
                        object_msg = ObjectsInfo()
                        object_msg.objects = objects_info
                        self.object_pub.publish(object_msg)
                except BaseException as e:
                    print(e)

                self.fps.update()
                result_image = self.fps.show_fps(image)
                self.result_image_pub.publish(cv2_image2ros(result_image))
                
                #cv2.imshow(self.name, result_cv2_image2rosimage)
                #key = cv2.waitKey(1)
                #if key != -1:
                #    self.running = False
            else:
                rospy.sleep(0.01)
        self.yolov5.destroy() 
        rospy.signal_shutdown('shutdown')

def cv2_image2ros(image):
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    image_temp = Image()
    header = Header(stamp=rospy.Time.now())
    header.frame_id = 'yolov5'
    image_temp.height = image.shape[:2][0]
    image_temp.width = image.shape[:2][1]
    image_temp.encoding = 'rgb8'
    image_temp.data = image.tostring()
    image_temp.header = header
    image_temp.step = image_temp.width*3
    
    return image_temp

if __name__ == "__main__":
    node = Yolov5Node('yolov5')
