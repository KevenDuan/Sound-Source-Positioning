#!/usr/bin/python
# coding:utf-8
# 作者：段浩
try:
    import RPi.GPIO as GPIO
except RuntimeError:
    print("Error importing RPi.GPIO!  This is probably because you need superuser privileges. "
          " You can achieve this by using 'sudo' to run your script")
import time


def servo_map(before_value, before_range_min, before_range_max, after_range_min, after_range_max):
    """
    功能:将某个范围的值映射为另一个范围的值
    参数：原范围某值，原范围最小值，原范围最大值，变换后范围最小值，变换后范围最大值
    返回：变换后范围对应某值
    """
    percent = (before_value - before_range_min) / (before_range_max - before_range_min)
    after_value = after_range_min + percent * (after_range_max - after_range_min)
    return after_value


GPIO.setmode(GPIO.BOARD)  # 初始化GPIO引脚编码方式
# 下部舵机
servo_SIG1 = 11
servo_GND1 = 6
servo_VCC1 = 4
# 上部舵机
servo_SIG2 = 12
servo_VCC2 = 2
servo_GND2 = 9

servo_freq = 50
servo_time = 0.01
servo_width_min = 2.5
servo_width_max = 12.5
# servo_degree_div =servo_width_max - servo_width_min)/180
GPIO.setup(servo_SIG1, GPIO.OUT)
GPIO.setup(servo_SIG2, GPIO.OUT)
# 如果你需要忽视引脚复用警告，请调用GPIO.setwarnings(False)
# GPIO.setwarnings(False)
servo1 = GPIO.PWM(servo_SIG1, servo_freq)  # 信号引脚=servo_SIG 频率=servo_freq in HZ
servo2 = GPIO.PWM(servo_SIG2, servo_freq)

servo1.start(0)
servo2.start(0)
servo1.ChangeDutyCycle(servo_map(75, 0, 180,servo_width_min,servo_width_max))  # 回归舵机中位
servo2.ChangeDutyCycle(servo_map(40, 0, 180,servo_width_min,servo_width_max))  # 回归舵机中位
time.sleep(1)
servo1.ChangeDutyCycle(0)
servo2.ChangeDutyCycle(0)
print('预设置完成，1秒后开始等待输入')
time.sleep(1)
# 为舵机指定位置
try:    # try和except为固定搭配，用于捕捉执行过程中，用户是否按下ctrl+C终止程序
    position1, position2 = '100', '0'
    while 1:
        if position1.isdigit() == 1:
            dc1 = int(position1)
            if (dc1>=0) and (dc1<=180):
                dc_trans1=servo_map(dc1, 0, 180,servo_width_min,servo_width_max)
                servo1.ChangeDutyCycle(dc_trans1)
                print("已转动到%d°处"%dc1)
                time.sleep(0.04)
                servo1.ChangeDutyCycle(0)
            else:
                print("Error Input:Exceed Range")
        else:
            print("Error Input:Not Int Input")
        if position2.isdigit() == 1:
            dc2 = int(position2)
            if (dc2 >= 0) and (dc2 <= 80):
                dc_trans2 = servo_map(dc2, 0, 180, servo_width_min, servo_width_max)
                servo2.ChangeDutyCycle(dc_trans2)
                print("已转动到%d°处"%dc2)
                time.sleep(0.04)
                servo2.ChangeDutyCycle(0)
            else:
                print("Error Input:Exceed Range")
        else:
            print("Error Input:Not Int Input")
        time.sleep(3)
        if position1 == '100':
            position1, position2 = '0', '80'
        else:
            position1, position2 = '100', '0'
except KeyboardInterrupt:
    pass

servo1.stop()  # 停止pwm
servo2.stop()
GPIO.cleanup()  # 清理GPIO引脚
