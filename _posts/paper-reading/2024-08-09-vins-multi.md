---
layout: post
title:  "[Paper Reading] VINS-Multi: A Robust Asynchronous Multi-camera-IMU State Estimator"
date:   2024-08-09 15:39:00 +0800
tags: 
    - slam
    - multi-camera
categories:
    - paper reading
---

> [VINS-Multi: A Robust Asynchronous Multi-camera-IMU State Estimator [WXS24] Arxiv 2024](https://arxiv.org/pdf/2405.14539) <br>
> Author: Luqi Wang, Yang Xu and Shaojie Shen

针对的问题：

- 单相机鲁棒性不够（某些场景下稳定的特征点过少）
- 多相机同步，带来设备昂贵的问题，错误的触发信号也会降低系统的鲁棒性
- 仅使用多相机，利用 B 样条来插值以对运动轨迹进行建模，会丢失自由度，引入IMU


提出的方法：

- 使用多个相机，获取更多的环境信息

- 总最大特征点数目恒定，动态分配各个相机的最大特征点数目，使得最后所有相机的特征点跟踪率一致，也就是特征点少的相机就不应该跟踪太多点，特征点多的相机就多跟踪些点

- 该文的边缘化策略，能够应对不均匀的来自多个相机的帧，以及某些相机失效的场景
    - 边缘化的是最旧的关键帧，无论是哪个相机的；
    - 但扔掉的则是新采集的帧所属相机对应的上一帧（疑惑：不这样做就不能应对上述的场景了吗？）

- 把 `后端需要特征点多的关键帧` 和 `后端需要非故障的稳定帧率的关键帧` 等价转换为 `优先级` 问题，然后通过选择优先级高的准则来实现前面的两种需求


达到的效果：

- 部分相机故障和恢复过程下，状态估计保持可接受的误差
- 对特征点稀少的墙面检测场景下，多相机比单一相机的漂移更小
- 该框架适用于不同类型的相机
- 实际应用场景之一：无人机通风管检测
