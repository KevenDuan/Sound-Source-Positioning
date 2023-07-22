#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#!/usr/bin/env python coding=utf-8

import wiringpi as pi
import numpy as np
import matplotlib.pyplot as plt
import time
import struct
import pandas as pd

import pandas as pd

def save_to_file(file_name, contents):
    fh = open(file_name, 'w')
    fh.write(contents)
    fh.close()

def deal(name, li):
    
    # list转dataframe
    df = pd.DataFrame(li, columns=['company_name'])
    
    # 保存到本地excel
    df.to_excel(f"{name}.xlsx", index=False)


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
    N = 200
    for i in range(0, N):
        temp = 0
        for j in range(i, i + N):
            temp += li1[j - i] * li2[j]
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
N = 60000+2
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
#plt.ion()

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

    arr1 = rdata[:,0].tolist()
    arr2 = rdata[:,1].tolist()
    save_to_file('a.txt', str(arr1))
    save_to_file('b.txt', str(arr2))
    
    

    
    time.sleep(100)
    # plt.plot(X,rdata[:,1],color="red")  # 画出当前x列表和y列表中的值的图形
    # plt.pause(0.001)  # 暂停一段时间，不然画的太快会卡住显示不出来
    # plt.ioff()  # 关闭画图窗口
    
    etime = time.time()
    dtime = etime - stime
    print(dtime)
