#!/usr/bin/env python
# encoding: utf-8
import tf
import math
import rospy
import tf2_ros
import jetauto_sdk.common as common
from geometry_msgs.msg import TransformStamped

if __name__ == '__main__':
    rospy.init_node('tf_publish')
    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)

    br = tf2_ros.StaticTransformBroadcaster()

    map_frame = rospy.get_param('~map_frame', '/jetauto_1/map')
    base_frame = rospy.get_param('~base_frame', '/jetauto_2/base_footprint')
    surround_frame = rospy.get_param('~surround_frame', 'surround_frame')
    radius = rospy.get_param('~radius', 0.5)
    speed = rospy.get_param('~speed', 10)
    
    d_angle = math.radians(speed/30.0)
    angle = 0
    rate = rospy.Rate(30.0)
    while not rospy.is_shutdown():
        try:
            # 查看相对的tf, 返回平移和旋转(check relative tf, and return translation and rotation)
            trans = tf_buffer.lookup_transform(map_frame, base_frame, rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue
        
        x = trans.transform.translation.x
        y = trans.transform.translation.y
        if angle > math.pi*2:
            angle = 0
        target_x = x + math.sin(angle)*radius
        target_y = y + math.cos(angle)*radius
        qua = common.rpy2qua(0, 0, math.radians(270 - math.degrees(angle) + 90))

        transform = TransformStamped()
        transform.header.stamp = rospy.Time.now()
        transform.header.frame_id = map_frame
        transform.child_frame_id = surround_frame
        
        transform.transform.translation.x = target_x
        transform.transform.translation.y = target_y
        transform.transform.translation.z = 0

        transform.transform.rotation = qua
        br.sendTransform(transform)

        angle += d_angle 
        rate.sleep()  # 以固定频率执行((execute at the fixed frequency))
