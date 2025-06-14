#!/usr/bin/env python
# encoding: utf-8
# @data:2022/11/18
# @author:aiden
# 语音开启垃圾分类
import os
import json
import rospy
from jetauto_sdk import buzzer
from std_msgs.msg import String
from std_srvs.srv import Trigger
from xf_mic_asr_offline import voice_play

class VoiceControlGarbageClassificationNode:
    def __init__(self, name):
        rospy.init_node(name, anonymous=True)
        self.running = True
        
        self.language = os.environ['LANGUAGE']
        rospy.Subscriber('/asr_node/voice_words', String, self.words_callback)
        while not rospy.is_shutdown():
            try:
                if rospy.get_param('/yolov5_node/start'):
                    break
            except:
                rospy.sleep(0.1)
        rospy.wait_for_service('/voice_control/get_offline_result')                  
        self.play('running')
       
        rospy.loginfo('唤醒口令: 小幻小幻(Wake up word: hello hiwonder)')
        rospy.loginfo('唤醒后15秒内可以不用再唤醒(No need to wake up within 15 seconds after waking up)')
        rospy.loginfo('控制指令: 开启垃圾分类 关闭垃圾分类(Voice command: sort waste/stop sort waste)')
        try:
            rospy.spin()
        except Exception as e:
            rospy.logerr(str(e))
            rospy.loginfo("Shutting down")

    def play(self, name):
        voice_play.play(name, language=self.language)

    def words_callback(self, msg):
        words = json.dumps(msg.data, ensure_ascii=False)[1:-1]
        if self.language == 'Chinese':
            words = words.replace(' ', '')
        print('words:', words)
        if words is not None and words not in ['唤醒成功(wake-up-success)', '休眠(Sleep)', '失败5次(Fail-5-times)', '失败10次(Fail-10-times']:
            if words == '开启垃圾分类' or words == 'sort waste':
                res = rospy.ServiceProxy('/garbage_classification/start', Trigger)()
                if res.success:
                    self.play('open_success')
                else:
                    self.play('open_fail')
            elif words == '关闭垃圾分类' or words == 'stop sort waste':
                res = rospy.ServiceProxy('/garbage_classification/stop', Trigger)()
                if res.success:
                    self.play('close_success')
                else:
                    self.play('close_fail')
        elif words == '唤醒成功(wake-up-success)':
            self.play('awake')
        elif words == '休眠(Sleep)':
            buzzer.on()
            rospy.sleep(0.05)
            buzzer.off()

if __name__ == "__main__":
    VoiceControlGarbageClassificationNode('voice_control_garbage_classification')
