---
layout: post
title:  "Realization of Online Temporal Calibration on AirSLAM" 
date:   2024-06-15 21:24:00 +0800
tags: 
    - slam
    - calibration
categories:
    - coding 
---

AirSLAM 在公开数据集上的效果很不错，但在笔者自己录制的数据集上效果很差，而相同的参数在 vins-fusion 上的效果却还不错。可能的原因是，公开数据集的 imu 与 图像帧有硬件同步，而自己录制的数据集未作同步，而 vins-fusion 在运行过程中在线校准 imu 和图像帧的时间偏移。这是二者存在的一个较为明显的区别，为了验证是否是未进行时间偏移估计导致的 AirSLAM 效果较差，笔者在 airslam 上增加了时间偏移估计，思路参考 vins-fusion。



最值得思索的是:

- 用已有的特征点去估计 imu batch 对应的起始或结束时刻的特征点位置

- 算法的迭代形式，这个迭代使得算法能够处理较大时间偏移。


给同步的数据集人为增加时间偏移，作为真值。