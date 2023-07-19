#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#!/usr/bin/env python coding=utf-8

# SPI_AD7606
# 用树莓派接口驱动AD转换芯片AD7606采样8路信号
# AD7606模拟输入：8路正负5V输入，编号V1~V8
# 树莓派物理引脚21(GPIO13,MISO)接AD7606的DB7/DoutA，
# 树莓派物理引脚19(GPIO12,MOSI)需空
# 树莓派物理引脚24(GPIO10,CE0)或25脚（CE1）接AD7606的	CS
# 树莓派物理引脚23(GPIO14,SCLK)接AD7606的RD/SCLK
# 树莓派物理引脚7(GPIO7)接AD7606的RST引脚，用来复位AD7606,H有效
# 树莓派物理引脚11(GPIO0)接AD7606的CA//CB引脚(两者并联)，上升沿启动转换
# AD7606空引脚：FD、BUSY
# 树莓派5V引脚接AD7606的5V供电。
# 接地引脚D15-D9, D6-D0，RANGE，OS2-OS0

import wiringpi as pi
import numpy as np
import matplotlib.pyplot as plt
import time
import math
import struct
try:
    import RPi.GPIO as GPIO
except RuntimeError:
    print("Error importing RPi.GPIO!  This is probably because you need superuser privileges. "
          " You can achieve this by using 'sudo' to run your script")

def servo_map(before_value, before_range_min, before_range_max, after_range_min, after_range_max):
    """
    功能:将某个范围的值映射为另一个范围的值
    参数：原范围某值，原范围最小值，原范围最大值，变换后范围最小值，变换后范围最大值
    返回：变换后范围对应某值
    """
    percent = (before_value - before_range_min) / (before_range_max - before_range_min)
    after_value = after_range_min + percent * (after_range_max - after_range_min)
    return after_value

def servo_move(angle):
    try:    # try和except为固定搭配，用于捕捉执行过程中，用户是否按下ctrl+C终止程序
        position1 = str(angle)
        position2 = '40'
        if position1.isdigit() == 1:
            dc1 = int(position1)
            if (dc1>=0) and (dc1<=180):
                dc_trans1=servo_map(dc1, 0, 180,servo_width_min,servo_width_max)
                servo1.ChangeDutyCycle(dc_trans1)
                print("已转动到%d°处"%dc1)
                time.sleep(0.04)
                servo1.ChangeDutyCycle(0)
                time.sleep(2)
            else:
                print("Error Input:Exceed Range")
        else:
            print("Error Input:Not Int Input")
            # 上部舵机
            # if position2.isdigit() == 1:
            #     dc2 = int(position2)
            #     if (dc2 >= 0) and (dc2 <= 80):
            #         dc_trans2 = servo_map(dc2, 0, 180, servo_width_min, servo_width_max)
            #         servo2.ChangeDutyCycle(dc_trans2)
            #         print("已转动到%d°处"%dc2)
            #         time.sleep(0.04)
            #         servo2.ChangeDutyCycle(0)
            #     else:
            #         print("Error Input:Exceed Range")
            # else:
            #     print("Error Input:Not Int Input")
    except KeyboardInterrupt:
        pass

def AD7606Reset():
    """
    function:复位
    """
    pi.digitalWrite(AD7606RST,HIGH) 
    pi.delay(1)
    pi.digitalWrite(AD7606RST,LOW) 
    
def AD7606StartConv():
    """
    function:开始转换
    """
    pi.digitalWrite(AD7606CONVST,HIGH)

def AD7606StopConv():
    """
    function:停止转换
    """
    pi.digitalWrite(AD7606CONVST,LOW) 

def getMax(li):
    """
    function:get index of max value
    return:index;max of all list
    """
    max = li[0]
    idx = 0
    for i in range(1, len(li)):
        if li[i] > max:
            max = li[i]
            idx = i
    return idx, max
    
def autocorrelation(li1, li2):
    """
    value:using autocorrelation of double list 
    return:result(list);N(int)
    function: get autocorrelation
    """
    result = []
    # 平移单元数
    N = 200
    for i in range(0, N):
        temp = 0
        for j in range(i, i + N):
            temp += (li1[j - i] * li2[j])
        result.append(temp)
    return N, result

def getFeak(num):
    """
    function: 把采样点的峰值存到峰值数组里
    value: num -> 采样点峰值
    """
    if len(feak) < 100 and num > 0:
        feak.append(num)
    elif len(feak) == 100:
        return True

def getAngle(n, l2):
    """
    function: 求声源角度
    value: n -> 峰值横坐标;l2 -> 斜边角
    """
    # 23khz 采样率
    l1 = 340 * (1/23000) * n
    temp = math.acos(l1/l2)
    # 对角度取整 -> 方便舵机移动
    angle = int((temp * 180)/math.pi)
    return angle

# 舵机角度控制
GPIO.setmode(GPIO.BOARD)  # 初始化GPIO引脚编码方式
# 下部舵机
servo_SIG1 = 32
servo_GND1 = 25
servo_VCC1 = 4
# 上部舵机
servo_SIG2 = 11
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
print('预设置完成,1秒后开始等待输入')
time.sleep(1)


# 定义逻辑信号i
LOW    = 0
HIGH   = 1
INPUT  = 0
OUTPUT = 1

#定义引脚
AD7606RST = 7     # pin7
AD7606CONVST = 0  # pin11

SPIchannel = 0    # SPI Channel (CE0)
# 单路采样频率约23kHz
SPIspeed = 62500000    # Clock Speed in 62500000Hz(max)


# N = 30000+2
N = 500+2
X = range(N)
# 16字节发送数据
sendData = bytes([0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0])
# 每行存8个短符号数
rdata = np.zeros((N,8),np.float32)

# init wiringpi 初始化
pi.wiringPiSetup()
# digtalWrite 数据写入之前必须将管脚设置为输出模式             
pi.pinMode(AD7606RST,OUTPUT)   
pi.pinMode(AD7606CONVST,OUTPUT)
# 初始化SPI
pi.wiringPiSPISetupMode(SPIchannel, SPIspeed, 0)  #Mode = 0~3
# mode = 0 -> 表示SPI设备以0时钟极性（CPOL）和0时钟相位（CPHA）工作

# 重置
AD7606Reset()
# 开始转换
AD7606StartConv()

# 存放100个自相关峰值点
feak = []

while True:
    for i in range(N):
        AD7606StopConv()
        # wiringpiSPIDataRW(int channel, unsigned char *data)
        recvData0 = pi.wiringPiSPIDataRW(SPIchannel,sendData)
        AD7606StartConv()
        nums = struct.unpack('>8h', recvData0[1])
        # >8h:">"表示高位在前，8h表示拆成8个短符号数
        rdata[i,:] = nums

    ain1 = rdata[:,0].tolist()
    ain2 = rdata[:,1].tolist()
    l, rst = autocorrelation(ain1, ain2)
    # a -> 峰值点横坐标 amax -> 峰值
    a, amax = getMax(rst)
    # 将峰值横坐标存到数组里
    flag = getFeak(a)
    if flag:
        print(feak)
        last_n = int(sum(feak)/500)
        feak = []
        print(last_n)
        angle = getAngle(30, 0.59)
        print(angle)
        servo_move(angle)