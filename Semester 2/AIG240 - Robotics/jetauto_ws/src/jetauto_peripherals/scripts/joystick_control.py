#!/usr/bin/env python
# encoding: utf-8
import sys
import time
import math
import rospy
from enum import Enum
from std_srvs.srv import Trigger
import jetauto_sdk.misc as misc
import jetauto_sdk.buzzer as buzzer
import geometry_msgs.msg as geo_msg
import sensor_msgs.msg as sensor_msg
from hiwonder_servo_msgs.msg import CommandDuration, JointState

AXES_MAP = 'lx', 'ly', 'rx', 'ry', 'r2', 'l2', 'hat_x', 'hat_y'
BUTTON_MAP = 'cross', 'circle', '', 'square', 'triangle', '', 'l1', 'r1', 'l2', 'r2', 'select', 'start', '', 'l3', 'r3', '', 'hat_xl', 'hat_xr', 'hat_yu', 'hat_yd', ''

class ButtonState(Enum):
    Normal = 0
    Pressed = 1
    Holding = 2
    Released = 3

class JoystickController:
    def __init__(self):
        rospy.init_node('joystick_control', anonymous=True)
        self.max_linear = rospy.get_param('~max_linear', 0.4)
        self.max_angular = rospy.get_param('~max_angular', 2.0)
        self.machine = rospy.get_param('~machine', 'JetAuto')
        self.disable_servo_control = rospy.get_param('~disable_servo_control', 'true')
        cmd_vel = rospy.get_param('~cmd_vel', 'jetauto_controller/cmd_vel')
        
        self.joy_sub = rospy.Subscriber('joy', sensor_msg.Joy, self.joy_callback) 
        self.mecanum_pub = rospy.Publisher(cmd_vel, geo_msg.Twist, queue_size=1)

        self.last_axes =  dict(zip(AXES_MAP, [0.0,] * len(AXES_MAP)))
        self.last_buttons = dict(zip(BUTTON_MAP, [0.0,] * len(BUTTON_MAP)))
        self.mode = 0

        self.joint1 = rospy.Publisher('joint1_controller/command_duration', CommandDuration, queue_size=1)
        self.joint2 = rospy.Publisher('joint2_controller/command_duration', CommandDuration, queue_size=1)
        self.joint3 = rospy.Publisher('joint3_controller/command_duration', CommandDuration, queue_size=1)
        self.joint4 = rospy.Publisher('joint4_controller/command_duration', CommandDuration, queue_size=1)
        self.jointr = rospy.Publisher('r_joint_controller/command_duration', CommandDuration, queue_size=1)

        self.joint1_state = 0
        self.joint2_state = 0
        self.joint3_state = 0
        self.joint4_state = 0
        self.joint5_state = 0
        self.jointr_state = 0
        rospy.Subscriber('joint1_controller/state', JointState, lambda msg: setattr(self, 'joint1_state', msg.goal_pos))
        rospy.Subscriber('joint2_controller/state', JointState, lambda msg: setattr(self, 'joint2_state', msg.goal_pos))
        rospy.Subscriber('joint3_controller/state', JointState, lambda msg: setattr(self, 'joint3_state', msg.goal_pos))
        rospy.Subscriber('joint4_controller/state', JointState, lambda msg: setattr(self, 'joint4_state', msg.goal_pos))
        rospy.Subscriber('jointr_controller/state', JointState, lambda msg: setattr(self, 'jointr_state', msg.goal_pos))

    def axes_callback(self, axes):
        twist = geo_msg.Twist()
        if self.machine != 'JetMega':
            twist.linear.y = misc.val_map(axes['lx'], -1, 1, -self.max_linear, self.max_linear)
        else:
            twist.linear.y = 0 
        twist.linear.x = misc.val_map(axes['ly'], -1, 1, -self.max_linear, self.max_linear)
        twist.angular.z = misc.val_map(axes['rx'], -1, 1, -self.max_angular, self.max_angular)
        self.mecanum_pub.publish(twist)

    def select_callback(self, new_state):
        if new_state == ButtonState.Pressed:
            self.mode = 1 if self.mode == 0 else 0
            rospy.loginfo("SELECT="+ str(self.mode))
            if self.mode == 0:
                buzzer.on()
                time.sleep(0.1)
                buzzer.off()
            elif self.mode == 1:
                for i in range(2):
                    buzzer.on()
                    time.sleep(0.1)
                    buzzer.off()
                    time.sleep(0.1)
            else:
                pass

    def l1_callback(self, new_state):
        if self.mode == 0 and not self.disable_servo_control:
            p = self.joint4_state - math.radians(2)
            self.joint4.publish(CommandDuration(data=p, duration=50))
            self.joint4_state = p
        else:
            pass

    def l2_callback(self, new_state):
        pass

    def r1_callback(self, new_state):
        if self.mode == 0 and not self.disable_servo_control:
            p = self.joint4_state + math.radians(2)
            self.joint4.publish(CommandDuration(data=p, duration=50))
            self.joint4_state = p
        else:
            pass

    def r2_callback(self, new_state):
        pass

    def square_callback(self, new_state):
        if self.mode == 0 and not self.disable_servo_control:
            p = self.jointr_state - math.radians(2)
            self.jointr.publish(CommandDuration(data=p, duration=50))
            self.jointr_state = p
        else:
            res = rospy.ServiceProxy('/set_row', Trigger)()
            if res.success:
                print('set row success')
            else:
                print('set row failed')

    def cross_callback(self, new_state):
        if self.mode == 0 and not self.disable_servo_control:
            p = self.joint3_state - math.radians(2)
            self.joint3.publish(CommandDuration(data=p, duration=50))
            self.joint3_state = p
        else:
            pass

    def circle_callback(self, new_state):
        if self.mode == 0 and not self.disable_servo_control:
            p = self.jointr_state + math.radians(2)
            self.jointr.publish(CommandDuration(data=p, duration=50))
            self.jointr_state = p
        else:
            res = rospy.ServiceProxy('/set_column', Trigger)()
            if res.success:
                print('set column success')
            else:
                print('set column failed')

    def triangle_callback(self, new_state):
        if self.mode == 0 and not self.disable_servo_control:
            p = self.joint3_state + math.radians(2)
            self.joint3.publish(CommandDuration(data=p, duration=50))
            self.joint3_state = p
        else:
            res = rospy.ServiceProxy('/set_triangle', Trigger)()
            if res.success:
                print('set triangle success')
            else:
                print('set triangle failed')

    def start_callback(self, new_state):
        if new_state == ButtonState.Pressed:
            buzzer.on()
            time.sleep(0.1)
            buzzer.off()
            if self.mode == 0:
                self.joint1.publish(CommandDuration(data=0, duration=2000))
                self.joint2.publish(CommandDuration(data=math.radians(-40), duration=2000))
                self.joint3.publish(CommandDuration(data=math.radians(110), duration=2000))
                self.joint4.publish(CommandDuration(data=math.radians(70), duration=2000))
                self.jointr.publish(CommandDuration(data=0, duration=2000))

    def hat_xl_callback(self, new_state):
        if self.mode == 0 and not self.disable_servo_control:
            p = self.joint1_state - math.radians(2)
            self.joint1.publish(CommandDuration(data=p, duration=50))
            self.joint1_state = p
        else:
            pass

    def hat_xr_callback(self, new_state):
        if self.mode == 0 and not self.disable_servo_control:
            p = self.joint1_state + math.radians(2)
            self.joint1.publish(CommandDuration(data=p, duration=50))
            self.joint1_state = p
        else:
            pass

    def hat_yd_callback(self, new_state):
        if self.mode == 0 and not self.disable_servo_control:
            p = self.joint2_state - math.radians(2)
            self.joint2.publish(CommandDuration(data=p, duration=50))
            self.joint2_state = p
        else:
            pass

    def hat_yu_callback(self, new_state):
        if self.mode == 0 and not self.disable_servo_control:
            p = self.joint2_state + math.radians(2)
            self.joint2.publish(CommandDuration(data=p, duration=50))
            self.joint2_state = p
        else:
            pass

    def joy_callback(self, joy_msg: sensor_msg.Joy):
        axes = dict(zip(AXES_MAP, joy_msg.axes))
        axes_changed = False
        hat_x, hat_y = axes['hat_x'], axes['hat_y']
        hat_xl, hat_xr = 1 if hat_x > 0.5 else 0, 1 if hat_x < -0.5 else 0
        hat_yu, hat_yd = 1 if hat_y > 0.5 else 0, 1 if hat_y < -0.5 else 0
        buttons = list(joy_msg.buttons)
        buttons.extend([hat_xl, hat_xr, hat_yu, hat_yd, 0])
        buttons = dict(zip(BUTTON_MAP, buttons))
        for key, value in axes.items(): # 轴的值被改变
            if self.last_axes[key] != value:
                axes_changed = True
        if axes_changed:
            try:
                self.axes_callback(axes)
            except Exception as e:
                rospy.logerr(str(e))
        for key, value in buttons.items():
            new_state = ButtonState.Normal
            if value != self.last_buttons[key]:
                new_state = ButtonState.Pressed if value > 0 else ButtonState.Released
            else:
                new_state = ButtonState.Holding if value > 0 else ButtonState.Normal
            callback = "".join([key, '_callback'])
            if new_state != ButtonState.Normal:
                rospy.loginfo(key + ': ' + str(new_state))
                if  hasattr(self, callback):
                    try:
                        getattr(self, callback)(new_state)
                    except Exception as e:
                        rospy.logerr(str(e))
        self.last_buttons = buttons
        self.last_axes = axes

if __name__ == "__main__":
    node = JoystickController()
    try:
        rospy.spin()
    except Exception as e:
        rospy.logerr(str(e))

