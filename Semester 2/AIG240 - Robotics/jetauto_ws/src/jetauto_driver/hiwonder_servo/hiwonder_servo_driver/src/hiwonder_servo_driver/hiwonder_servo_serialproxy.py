#!/usr/bin/env python
# encoding: utf-8
import sys
import rospy
from threading import Thread
from collections import deque

from hiwonder_servo_driver import hiwonder_servo_io
from hiwonder_servo_driver.hiwonder_servo_const import *

from hiwonder_servo_msgs.msg import MultiRawIdPosDur, SetServoState, ServoState, ServoStateList, RawIdPosDur
from hiwonder_servo_msgs.srv import GetServoState, SetReadTimeout

class SerialProxy:
    def __init__(self,
                 port_name='/dev/ttyTHS1',
                 port_id= 1,
                 baud_rate='115200',
                 min_motor_id=1,
                 max_motor_id=25,
                 connected_ids=[],
                 update_rate=5,
                 fake_read=False):

        self.port_name = port_name
        self.port_id = str(port_id)
        self.baud_rate = baud_rate
        self.min_servo_id = min_motor_id
        self.max_servo_id = max_motor_id
        self.servos = connected_ids
        self.update_rate = update_rate
        self.fake_read = fake_read
        self.running = False
        self.servo_io = None
        self.servos_static_info = {}

        self.actual_rate = update_rate
        self.error_counts = {'non_fatal': 0, 'checksum': 0, 'dropped': 0}
        self.current_state = ServoStateList()
        self.servo_states_pub = rospy.Publisher('servo_controllers/port_id_{}/servo_states'.format(self.port_id),
                                                ServoStateList, queue_size=1)

        self.servo_command_sub = rospy.Subscriber('servo_controllers/port_id_{}/id_pos_dur'.format(self.port_id),
                                                  RawIdPosDur,
                                                  self.id_pos_dur_cb)
        self.servo_command_sub = rospy.Subscriber('servo_controllers/port_id_{}/multi_id_pos_dur'.format(self.port_id),
                                                  MultiRawIdPosDur,
                                                  self.multi_id_pos_dur_cb)
        rospy.Subscriber('servo_control/set_servo_state', SetServoState, self.set_servo_state)
        rospy.Service('servo_control/get_servo_state', GetServoState, self.get_servo_state)
        rospy.Service('servo_control/set_read_timeout', SetReadTimeout, self.set_read_timeout)

    def set_read_timeout(self, msg):
        self.servo_io.set_timeout(msg.data)

        return [True, 'set_read_timeout']

    def set_servo_state(self, msg):
        cmd = msg.cmd
        param = msg.param
        if cmd == 'id':
            self.servo_io.set_servo_id(param[0], param[1])
        elif cmd == 'deviation':
            self.servo_io.set_servo_deviation(param[0], param[1])
        elif cmd == 'save_deviation':
            self.servo_io.save_servo_deviation(param[0])
        elif cmd == 'pulse_range':
            self.servo_io.set_servo_range(param[0], param[1], param[2])
        elif cmd == 'voltage_range':
            self.servo_io.set_servo_vin_range(param[0], param[1], param[2])
        elif cmd == 'temperature_range':
            self.servo_io.set_servo_temp_range(param[0], param[1])
        elif cmd == 'unload':
            self.servo_io.unload_servo(param[0])
        
    def get_servo_state(self, msg):
        cmd = msg.cmd
        servo_id = msg.id
        result = None
        if cmd == 'id':
            if servo_id >= 0:
                result = self.servo_io.get_servo_id(servo_id)
            else:
                result = self.servo_io.get_servo_id()
        elif cmd == 'deviation':
            result = self.servo_io.get_servo_deviation(servo_id)
        elif cmd == 'pulse_range':
            result = self.servo_io.get_servo_range(servo_id)
        elif cmd == 'voltage_range':
            result = self.servo_io.get_servo_vin_range(servo_id)
        elif cmd == 'temperature_range':
            result = self.servo_io.get_servo_temp_range(servo_id)
        elif cmd == 'temperature':
            result = self.servo_io.get_servo_temp(servo_id)
        elif cmd == 'voltage':
            result = self.servo_io.get_servo_vin(servo_id)
        elif cmd == 'load_state':       
            result = self.servo_io.get_servo_load_state(servo_id)
        if result is None:
           result = []
        elif type(result) != tuple:
            result = [result]
        return [True, result]

    def id_pos_dur_cb(self, msg):
        self.servo_io.set_position(msg.id, msg.position, msg.duration)

    def multi_id_pos_dur_cb(self, msg):
        for id_pos_dur in msg.id_pos_dur_list:
            self.servo_io.set_position(id_pos_dur.id, id_pos_dur.position, id_pos_dur.duration)

    def connect(self):
        try:
            self.servo_io = hiwonder_servo_io.HiwonderServoIO(self.port_name, self.baud_rate)
            self.__find_motors()
        except hiwonder_servo_io.SerialOpenError as e:
            rospy.logfatal(e.message)
            sys.exit(1)
        self.running = True
        if self.update_rate > 0:
            Thread(target=self.__update_servo_states).start()

    def disconnect(self):
        self.running = False

    def __find_motors(self):
        rospy.loginfo(
            '%s: Pinging motor IDs %d through %d...' % (self.port_id, self.min_servo_id, self.max_servo_id))
        if not self.servos:
            for servo_id in range(self.min_servo_id, self.max_servo_id + 1):
                result = self.servo_io.ping(servo_id)
                if result:
                    self.servos.append(servo_id)
        if not self.servos:
            rospy.logfatal('port_id_%s: No motors found.' % self.port_id)
            sys.exit(1)
        status_str = 'port_id_%s: Found %d motors - ' % (self.port_id, len(self.servos))
        # rospy.set_param('/{}/serial_ports/{}/connected_ids'.format(self.root_namespace, self.port_namespace),
        #                 self.servos)
        rospy.loginfo('%s, initialization complete.' % status_str[:-2])

    def __update_servo_states(self):
        num_events = 50
        rates = deque([float(self.update_rate)] * num_events, maxlen=num_events)
        last_time = rospy.Time.now()

        rate = rospy.Rate(self.update_rate)
        while not rospy.is_shutdown() and self.running:
            # get current state of all motors and publish to motor_states topic
            servo_states = []
            for servo_id in self.servos:
                try:
                    state = self.servo_io.get_feedback(servo_id, self.fake_read)
                    #print(servo_id, state['position'])
                    if state:
                        servo_states.append(ServoState(**state))
                        if hiwonder_servo_io.exception:
                            raise hiwonder_servo_io.exception
                except Exception as e:
                    rospy.logerr(e)

            if servo_states:
                msl = ServoStateList()
                msl.servo_states = servo_states
                self.servo_states_pub.publish(msl)
                self.current_state = msl
                # calculate actual update rate
                current_time = rospy.Time.now()
                rates.append(1.0 / (current_time - last_time).to_sec())
                self.actual_rate = round(sum(rates) / num_events, 2)
                last_time = current_time

            rate.sleep()
