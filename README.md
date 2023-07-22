# Sound-Source-Positioning
2021 Electronic Design Competition - E

**My test code, as well as the contest code.**
> 需要的材料：AD7606、树莓派、杜邦线、舵机、MAX9814麦克风

## 文件说明
- AD7606_all.py 运行即可显示自相关函数图像
- AD7606_double.py 运行即可显示双麦克风的声波信号
- test1.py（初始版）运行即可根据声音定位位置
- **test2.py (改良版) 精准的定位声源位置**
- Gimbal.py 为舵机云台的运动代码
- plotExcel.py 运行即可把Ain1和Ain2的模数转化信号给保存下来
> test2.py 是这个赛题的完整代码，包含AD转化，可以测出声源位置并控制舵机运动。
