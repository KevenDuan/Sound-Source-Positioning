# Sound-Source-Positioning
2021 Electronic Design Competition - E

**My test code, as well as the contest code.**
> 需要的材料：AD7606 * 1、树莓派 * 1、杜邦线若干、舵机 * 2、MAX9814麦克风 * 2

## 文件说明
- AD7606_all.py 运行即可显示自相关函数图像
- AD7606_double.py 运行即可显示双麦克风的声波信号
- test1.py（初始版）运行即可根据声音定位位置
- **test2.py (改良版) 精准的定位声源位置**
- Gimbal.py 为舵机云台的运动代码
- plotExcel.py 运行即可把Ain1和Ain2的模数转化信号给保存下来
> test2.py 是这个赛题的完整代码，包含AD转化，可以测出声源位置并控制舵机运动。
## 接线方式
**树莓派与AD7606的接线方式：**    
>用树莓派接口驱动AD转换芯片AD7606采样8路信号  
AD7606模拟输入：8路正负5V输入，编号V1~V8  
树莓派物理引脚21(GPIO13,MISO)接AD7606的DB7/DoutA  
树莓派物理引脚19(GPIO12,MOSI)需空  
树莓派物理引脚24(GPIO10,CE0)或25脚（CE1）接AD7606的	CS  
树莓派物理引脚23(GPIO14,SCLK)接AD7606的RD/SCLK  
树莓派物理引脚7(GPIO7)接AD7606的RST引脚，用来复位AD7606,H有效  
树莓派物理引脚11(GPIO0)接AD7606的CA//CB引脚(两者并联)，上升沿启动转换  
AD7606空引脚：FD、BUSY 
树莓派5V引脚接AD7606的5V供电。 
接地引脚D15-D9, D6-D0，RANGE，OS2-OS0  

**MAX9814麦克风模块的相关接线：**
> 两个MAX9814模块的 OUT分别接AD706的 AIN1和AIN2  
> 两个MAX9814模块的 Vcc和GND分别接在树莓派上的3.3V和GND引脚

**舵机云台的接线方式：**
> 舵机的SIG信号引脚接树莓派的pin32  
> Vcc和GND 分别接到树莓派的5v和GND  
> **主要是用到下部电机，上部电机可以不接线**
