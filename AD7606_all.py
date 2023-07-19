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
import struct

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

def autocorrelation(li1, li2):
    """
    double list autocorrelation
    """
    result = []
    # 平移单元数
    N = 40
    for i in range(0, N):
        temp = 0
        for j in range(i, i + N):
            temp += (li1[j - i] * li2[j])/1000
        result.append(temp)
    return N, result

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

# 创建绘制实时动态窗口
plt.ion()

while True:
    stime = time.time()
    for i in range(N):
        AD7606StopConv()
        # wiringpiSPIDataRW(int channel, unsigned char *data)
        recvData0 = pi.wiringPiSPIDataRW(SPIchannel,sendData)
        AD7606StartConv()
        nums = struct.unpack('>8h', recvData0[1])
        # >8h:">"表示高位在前，8h表示拆成8个短符号数
        rdata[i,:] = nums
    plt.clf()  # 清除之前画的图
    ain1 = rdata[:,0].tolist()
    ain2 = rdata[:,1].tolist()
    l, rst = autocorrelation(ain1, ain2)
    plt.plot(range(l), rst)
    plt.pause(0.001)  # 暂停一段时间，不然画的太快会卡住显示不出来
    plt.ioff()  # 关闭画图窗口
    
    etime = time.time()
    dtime = etime - stime
    print(dtime)
