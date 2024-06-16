---
layout: post
title:  "[Paper Reading] Online Temporal Calibration for Visual Intertial System [IROS 2018]"
date:   2024-06-15 21:24:00 +0800
tags: 
    - slam
    - calibration
categories:
    - paper reading
---

> [arXiv:1808.00692](https://arxiv.org/abs/1808.00692)

## 针对的问题

相机时间戳与IMU时间戳存在初始的时间偏移，线程收到两个时间戳的两帧KeyFrame，在这两个时刻内进行 IMU 预积分，此时需要在 imu buffer 里检索这段时间内的 IMU 数据，若以 IMU 的时间为基准，这两个时刻实际的关键帧并不是一开始收到的那两帧，而是在空间上有所偏移-

## 使用的方法

两个时刻上很接近的关键帧，其空间变化也很小，图像上的特征点认为是匀速运动的，速度用两帧关键帧的对应特征点坐标相减求出，然后根据还没求出但假设已知的时间偏移 $t_d$ 对特征点的坐标进行修正。

将带有 $t_d$ 的修正后的特征点坐标，替换掉重投影误差中原来的特征点坐标，从而将时间偏差参数 $t_d$ 引入了优化方程相机残差项

每次优化完成后，对相机时间戳进行修正，由粗到细，使得匀速运动假设更加合理。

## 实现的效果

- 与 Kalibr 离线标定效果接近
- 手动添加时间偏移，比没有该措施的 vinsmono 和 okvis 好
- 适用于廉价传感器

## 存在的问题/未来的工作

没有对 $t_d$ 的可观性进行分析


### 参考

[知乎：VIO系统中IMU与相机时间偏差标定(PaperReading)](https://zhuanlan.zhihu.com/p/53555106)
