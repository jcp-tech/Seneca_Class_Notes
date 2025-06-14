#!/usr/bin/python3
# coding=utf8
import platform
if platform.machine() == 'aarch64':
    import Jetson.GPIO as GPIO
    
    BUZZER_PIN = 18  # buzzer对应引脚号(the pin number corresponding to buzzer)

    mode = GPIO.getmode()
    if mode == 1 or mode is None:  # 是否已经设置引脚编码(whether the pin number is set)
        GPIO.setmode(GPIO.BCM)  # 设为BCM编码(set as BCM code)

    GPIO.setwarnings(False)  # 关闭警告打印(close alarm print)

    GPIO.setup(BUZZER_PIN, GPIO.OUT)  # 设置引脚为输出模式(set the pin as output mode)

    def on():
        GPIO.output(BUZZER_PIN, 1)

    def off():
        GPIO.output(BUZZER_PIN, 0)

    def set(new_state):
        GPIO.output(BUZZER_PIN, new_state)
else:
    def on():
        print('buzzer on')

    def off():
        print('buzzer off')

    def set(new_state):
        pass
