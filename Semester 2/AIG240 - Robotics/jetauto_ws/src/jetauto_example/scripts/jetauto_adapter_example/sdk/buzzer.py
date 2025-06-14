#!/usr/bin/python3
# coding=utf8
import Jetson.GPIO as GPIO

BUZZER_PIN = 18  # buzzer对应引脚号

mode = GPIO.getmode()
if mode == 1 or mode is None:  # 是否已经设置引脚编码
    GPIO.setmode(GPIO.BCM)  # 设为BCM编码

GPIO.setwarnings(False)  # 关闭警告打印

GPIO.setup(BUZZER_PIN, GPIO.OUT)  # 设置引脚为输出模式

def on():
    GPIO.output(BUZZER_PIN, 1)

def off():
    GPIO.output(BUZZER_PIN, 0)

def set(new_state):
    GPIO.output(BUZZER_PIN, new_state)
