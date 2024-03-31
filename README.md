# ros-control

把视控一体的控制部分压缩为一个节点，消除进程间通信延迟

## Dependencies

```
-ros2=galactic(can not be foxy!)
-libusb=1.0.26(can be other version)
-rt kernel(eg. 5.15.133-rt69)
```

```shell
# setup libusb
wget https://github.com/libusb/libusb/releases/download/v1.0.26/libusb-1.0.26.tar.bz2 
tar -jxvf libusb-1.0.26.tar.bz2 
sudo apt update 
sudo apt-get install libudev-dev
cd libusb-1.0.26
sudo ./configure 
sudo make install

# check libusb
cd example
sudo make listdevs
sudo ./listdevs

# 出现类似以下内容即为成功
1d6b:0003 (bus 4, device 1)
8087:0033 (bus 3, device 3) path: 10
0483:5740 (bus 3, device 2) path: 1
1d6b:0002 (bus 3, device 1)
1d6b:0003 (bus 2, device 1)
1d6b:0002 (bus 1, device 1)
```

[related links]( https://blog.csdn.net/jiacong_wang/article/details/106720863?spm=1001.2014.3001.5502)

to setup the realtime kernel, please check this [article]( https://zhuanlan.zhihu.com/p/675155576)

## 检查硬件

### usb

usb通信接受从C板传来的imu及遥控器数据

在launch之前需要赋权usb。

```shell
lsusb   #查看usb设备
```

找到stm32 usb虚拟串口，假设为

```
Bus 003 Device 009: ID 0483:5740 STMicroelectronics Virtual COM Port
```

一般Bus id不会改变，而Device id在每次重新插入后都会发生变化 ,所以一般这样赋权此usb端口

```shell
sudo chmod 777 /dev/bus/usb/003/*
# or 
./scripts/usb.sh
```

### can communication

两条can总线分别挂载了底盘电机和云台电机。

> NOTE: 测试用的车的CAN1的HL线颜色反了，红色是L黑色是H

``` shell
./scripts/can.sh
# 检查can0 can1顺序
candump can0    #底盘
candump can1    #云台
```

### remote

遥控器上的两个拨杆布局如下：

```
remote:
buttons[5]---     ---buttons[2]
buttons[4]---     ---buttons[1]
buttons[3]---     ---buttons[0]
```

左右拨杆分别拨到上中下某一位置时对应数组元素置1，其余为0

两个摇杆布局如下：

```
     axis[3]                axis[1]
     ^                      ^
     |                      |
<--------->axis[2]     <--------->axis[0]
     |                      |
     v                      v
```

```axis[4]```是遥控器左上角的轮子，向左转是正向右转是负

### chassis wheels

底盘轮子电机布局如下：

```
    //2-----battary-----1
    //|                 |
    //|                 |
    //|                 |
    //|                 |
    //|                 |
    //|                 |
    //|                 |
    //3-----------------4
```
can 反馈报文 id = 200 + index

### gimbal

各电机的RX标准帧ID：

```c
#define AMMOR 0x201
#define AMMOL 0x202
#define ROTOR 0x203
#define YAW   0x205 
#define PITCH 0x206
```

## Build and Launch

```shell
# [可选]清理构建残留
cd sh
./clean.sh
###########

colcon build
source install/setup.bash

ros2 launch control_node control_node
```

## Useful commands

```shell
candump can1 | grep 1FF

candump can0 | grep 2004
```

出现其他问题可优先检查：

- sh文件内控制路径、自瞄路径及nuc密码
- 参数文件是否正确读取
- can0 can1有没有弄反
