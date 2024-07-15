---
layout: post
title:  "XI35 装机全过程"
date:   2024-07-15 11:40:00 +0800
tags: quadcopter drone
categories:
    - fast-drone
---

本篇记录 XI35 机体的组装过程，以及飞控、ORIN的配置，最终实现 fast-drone-250 的过程。

所有的配件型号可在 [配件型号采购表](https://docs.qq.com/sheet/DQlVFVXBTYkpobktF?u=4ce3cf3cb1fc4960917789399c899a1a&tab=bliok2) 中找到。

总重 540g -> 775g（安装了相机和orin）


## 机体安装步骤

- 将电机安装到机架上，使用四个M2*6，注意机架的展望反面·上下面，孔位有凹槽的为上面

- 穿电池绑带

- 将M3转M2橡胶减震柱穿过电驱和飞控的四角安装孔（技巧：橡皮筋技巧）

- 使用M2*20安装电驱，8pin接口朝机头，接口贴着机架			

- 电机线修修剪（用剥线钳），焊接(加松香)；电源线和滤波电容焊接

- 连接飞控（见下面QCG版本问题），连接电池，更改电机序号映射和转向

- 连接接收机（遥传一体模块中靠近LED的是RC接口）到飞控的 RC 端口，数传（遥传一体模块中的另一个接口）到 TELEM1 端口，GPS模块连接 到 GPS 端口

- 线穿过镂空部分把这几个模块都放到机架顶部

- （略）M2*12固定摄像头座，M2*16固定图传天空端	

- M3*6安装前六个铝柱的上部，第七个在尾部的立柱先不装（b站视频说错了，应该看三维动图）

- （略）M2*12安装配重打印件

- 3d打印天线座安装，大的套在机架尾部，小的套在涵道圈，打印件有弹性，孔偏小，需要M3螺丝旋转	着才能穿过

- 把各个数据线的飞控端都装上（tele2（连接到orin的），gps，接收机）

- 机架，七个铝柱，涵道圈，菱形底架，从上至下叠起来后从下往上安装M3*14

- 调试完成后安装桨叶，使用M2*10安装

- 打印相机和orin的安装连接件，都是M2，立柱也是M2，配合螺母安装


## 飞控配置

### 基本配置

- 测试电机转向

- 遥控器和接收机对频，并在地面站校准

- 配置通道功能，用于飞行模式切换、紧急停止等（注意：有三档的前两档一致）详见 https://gitee.com/jerry-ironman/px4ctrl
  - 5通道2档：自稳和offboard 
  - 6通道3档：是否接受命令 
  - 7通道3档：急停 
  - 8通道2档：px4ctrl对px4的重启

- 飞控与罗盘方向设置

- 陀螺仪与加速度计校准

- 校准地平线

### 与 ORIN 通信相关的配置

主要参考 [Nxt-FC](https://github.com/HKUST-Aerial-Robotics/Nxt-FC?tab=readme-ov-file)，github 上的 readme 进行设置，不要按照商家给的安装教程里的进行设置。主要是配置 MAV_0 的参数

- MAV_0_CONFIG: TELE2 (Serial Configuration for MAVLink (instance 0))
- MAV_0_FLOW_CTRL: Auto-detected
- MAV_0_FORWARD: Enabled
- MAV_0_MODE: External Vision
- MAV_0_RADIO_CTL: Enabled
- MAV_0_RATE: 92160 B/s


### 高分辨率和高频率的 IMU 数据

create file in your tf-card /etc/extras.txt

```
mavlink stream -d /dev/ttyS3 -s ATTITUDE -r 200

mavlink stream -d /dev/ttyS3 -s HIGHRES_IMU -r 1000
```

then using the following settings:
- IMU_GYRO_RATEMAX: 2000Hz
- IMU_INTEG_RATE: 400Hz
- MAV_0_MODE: External vision
- Set Uart4(SER_TELE2) to 921600
- MAV_0_RATE 92160B/s

after these settings you will have 250Hz /imu/data_raw /imu/data

### ORIN NX 配置

#### 安装 jetpack 5.1.3 linux for jetson orin nx modules: 

- 在 PC 上的 ubuntu 里安装 sdkmanager

- 按住 REC 键并给 orin 上电，上电后可松开，之后将 typeC 插到 Recovery Port（在底板短边上的那个typec口，即 USB1），另一端连接到 PC

- 按照 sdkmanager 的引导操作即可，可选择组件安装
    - 此时 opencv 可以不装，因为此阶段安装的 opencv 不支持 cuda 加速

- 下载镜像

- 安装镜像
    - 注意 username不能是大写字母，否则 sdkmanager 无法自动设置orin的用户名等后续的默认设置参数，进一步导致无法通过usb以太网连接orin
    - 一切安装完成后，忘记拔掉 usb 将导致板子一直无法正常启动，外接屏幕无画面，拔掉后就一切正常

- 如果 PC 上的 ubuntu 硬盘空间不足，可外接移动硬盘，将 Download Folder 和 Target HW image folder 设置在移动硬盘（FAT）里
    - 注意：移动硬盘的文件系统不能是 windows 的 FAT，而应该是 EXT4，否则镜像将会安装错误

#### 安装

