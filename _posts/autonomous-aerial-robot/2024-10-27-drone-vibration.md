---
layout: post
title:  "Drone Vibration"
date:   2024-10-27 20:00:00 +0800
tags: 
    - motion control
categories:
    - autonomous aerial robot
---


四旋翼无人机的飞行控制的反馈主要来自于 IMU，也就是加速度计和角速度计，其中角速度测量值对震动不敏感，而加速度测量值对震动十分敏感，若减震不佳，如机架有其他部件未上紧，或是桨叶严重受损，都会对带来高频的振动噪声，对飞行控制还有里程计的估计有“毁灭性的污染”。此时使用数字滤波，只会带来很大的相位延迟，仍然会导致控制的不稳定。因此只有从源头上解决震动，辅以软件数字滤波，才能得到较好的控制效果。

引用自 px4 的教程：
A few of simple steps that may reduce vibrations are:

- Make sure everything is firmly attached on the vehicle (landing gear, GPS mast, etc.).
- Use balanced propellers.
- Make sure to use high-quality components for the propellers, motors, ESC and airframe. Each of these components can make a big difference.
- Use a vibration-isolation method to mount the autopilot. Many flight controllers come with mounting foam that you can use for this purpose, while others have inbuilt vibration-isolation mechanisms.
- As a last measure, adjust the software filters. It is better to reduce the source of vibrations, rather than filtering them out in software.

## 振动影响因素实验

为了验证不同因素对加速度计测量值震动，进行了一下几个实验：

第一个实验为震动最小的配置，作为一个参考基准，震动最小的原因是其桨叶全新，同时上部未安装相机、NX机载电脑等额外的负载，因此振动的来源最少。第二至四个实验，随着改进措施的实施，震动逐渐减小，结果如下：

|实验编号|1|2|3|4|5|
|-|-|-|-|-|-|
|振动幅值[m s^-2]|0.75|15.0|3.0|2.75|2.5|

结论为桨叶弯折会带来巨大的振动，负载晃动会带来一定的振动，桨叶轻微磨损则问题不大。

具体实验配置和测试结果如下：

1.  08 号：桨叶全新 + 无任何额外负载
![orin08_noload](/assets/2024-10-27-drone-vibration/orin08_noload.png)

2. 01 号：桨叶磨损弯折 + 负载松动
![orin01_bladebroken_loadloose](/assets/2024-10-27-drone-vibration/orin01_bladebroken_loadloose.png)

3. 01 号：桨叶换新 + 负载松动
![orin01_newblade_loadloose](/assets/2024-10-27-drone-vibration/orin01_newblade_loadloose.png)

4. 01 号：桨叶换新 + 上紧用于固定 RS 相机的 3D 打印件
![orin01_newblade_tightRS](/assets/2024-10-27-drone-vibration/orin01_newblade_tightRS.png)

5. 01 号：桨叶换新 + 上紧rs相机打印件 + 垫nx板打印件中间
![orin01_newblade_tightRS_padNX](/assets/2024-10-27-drone-vibration/orin01_newblade_tightRS_padNX.png)


## 油门估计反馈量的选择

下图是滑窗100 + 延迟补偿，imu原始数据 和 imu_propagate 的速度微分后 都有相同的高频噪声，差了个9.8的直流分量，都不适合用于油门估计。

odom的速度微分后，得到加速度，频率较低，但没有高频噪声。
![alt text](/assets/2024-10-27-drone-vibration/image.png)

归根结底，如果没有解决振动问题，并没有合适的加速度反馈量来源，因此首要的目的还是利用上文的结论对 imu 进行减震。

## 数字一阶低通滤波

在完成减震后，可进一步使用软件滤波，进一步提高加速度计测量值的质量，同时不带来太多的相位延迟。

一阶低通滤波，给定截止频率 $f_c$，采样频率 $f_s$，可计算权重：
$$
b = 2 \pi \frac{f_c}{f_s}
$$

$$
a = \frac{b}{1 + b}
$$

$$
y[n] = a * x[n] + (1-a) * y[n-1]
$$

如果选择 a = 0.5 进行滤波，效果如下：

|实验类型|无滤波|有滤波(a = 0.5)|
|-|-|-|
|振动幅值 [m s^-2]|2.5 |2.0|

无滤波：
![nolpf](/assets/2024-10-27-drone-vibration/nolpf.png)

有滤波：
![1st_lpf](/assets/2024-10-27-drone-vibration/1st_lpf.png)

### 参考

[航模基础知识--------螺旋桨的静平衡 ](https://www.douban.com/group/topic/86585459/?_i=9946276ISFgcZ_,1901979ISFgcZ_)

[Vibration Isolation - px4](https://docs.px4.io/main/en/assembly/vibration_isolation.html)
