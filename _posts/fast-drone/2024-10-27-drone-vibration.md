---
layout: post
title:  "Drone Vibration"
date:   2024-10-27 20:00:00 +0800
tags: 
    - motion control
categories:
    - fast-drone
---


四旋翼无人机的飞行控制的反馈主要来自于 IMU，也就是加速度计和角速度计，其中角速度测量值对震动不敏感，而加速度测量值对震动十分敏感，若减震不佳，如机架有其他部件未上紧，或是桨叶严重受损，都会对带来高频的振动噪声，对飞行控制还有里程计的估计有“毁灭性的污染”。此时使用数字滤波，只会带来很大的相位延迟，仍然会导致控制的不稳定。因此只有从源头上解决震动，辅以软件数字滤波，才能得到较好的控制效果。


下图是滑窗100 + 延迟补偿，imu原始数据 和 imu_propagate 的速度微分后 都有相同的高频噪声，差了个9.8的直流分量，都不适合用于油门估计
odom的速度微分后，得到加速度，频率较低，但没有高频噪声
![alt text](/assets/2024-10-27-drone-vibration/image.png)


为了验证不同因素对加速度计测量值震动，进行了一下几个实验：

1.  08 号：桨叶全新 + 无任何额外负载
![orin08_noload](/assets/2024-10-27-drone-vibration/orin08_noload.png)

2. 01 号：桨叶磨损 + 负载松动
![orin01_bladebroken_loadloose](/assets/2024-10-27-drone-vibration/orin01_bladebroken_loadloose.png)

3. 01 号：桨叶换新 + 负载松动
![orin01_newblade_loadloose](/assets/2024-10-27-drone-vibration/orin01_newblade_loadloose.png)

4. 01 号：桨叶换新 + 上紧用于固定 RS 相机的 3D 打印件
![orin01_newblade_tightRS](/assets/2024-10-27-drone-vibration/orin01_newblade_tightRS.png)

5. 01 号：桨叶换新 + 上紧rs相机打印件 + 垫nx板打印件中间
![orin01_newblade_tightRS_padNX](/assets/2024-10-27-drone-vibration/orin01_newblade_tightRS_padNX.png)


第一个实验为震动最小的配置，作为一个参考基准，震动最小的原因是其桨叶全新，同时上部未安装相机、NX机载电脑等额外的负载，因此振动的来源最少。

第二至四个实验，随着改进措施的实施，震动逐渐减小。


|实验编号|1|2|3|4|5|
|-|-|-|-|-|-|
|振动幅值[m s^-2]|0.75|15.0|3.0|2.75|2.5|
