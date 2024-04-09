---
layout: post
title:  "a CSLAM and Mutual Localization Review"
date:   2024-04-09 21:00:00 +0800
tags: slam
categories:
---

CSLAM 全称为 collaborative SLAM，用于估计机器人间的 `相对位姿` 和 `全局一致的轨迹`。本篇将介绍 CSLAM 的整体框架，侧重于介绍 CSLAM 的初始化阶段，以及机器人间的相互定位。

## FACT: Fast and Active Coordinate Initialization for Vision-based Drone Swarms

> IROS 投稿
<br>
原文：[arXiv:2403.13455](https://arxiv.org/abs/2403.13455)
<br>
视频：[FACT: Fast and Active Coordinate Initialization for Vision-based Drone Swarms](https://www.bilibili.com/video/BV1sA4m1A7DD/?vd_source=e371652571b1539bbd501fb7adb6cfc4)



- 针对什么问题？
    - 针对有着 SWaP(size, weight and power) 约束的无人机集群，仅使用`基于视觉的匿名的相互观测`，而不使用其他传感器辅助（如 UWB，动捕和特制信标），如何实现`坐标系的初始化`？
    - `基于视觉的匿名的相互观测` 带来了新的挑战：
        - 有限的 FOV：视角和传感器距离有限，某些无人机之间不存在相互观测
        - 匿名：无法直接知晓通过视觉检测到的无人机的 id 是多少
        - 安全性：在初始化阶段，无人机需要避免和其他无人机以及环境发生碰撞
- 采用什么方法？
    - 问题建模：相互观测向量求和为零向量 -> 最小二乘优化问题
    - 整体思路：原地旋转，判断相互测量是否完整（$rank(\mathrm{Z}^*) \leq N+1$），若不满足，则随机移动，重复以上步骤。当 $rank(\mathrm{Z}^*) \leq N+1$，意味着所有测量均已获得，均为相互的。此时可求解出各机坐标系的相对旋转 $R_{t_0}$，不受匿名条件的影响。求解出 $R_{t_0}$ 后，视觉测量和 VIO 的关联问题可以转化为求解关联矩阵 $A_i$，等价于求解二分图匹配问题，可使用匈牙利算法求解。
    - 针对有限的 FOV：原地旋转 ＋ 多轮随机移动
    - 针对匿名的测量：使用匈牙利算法求解 $A_i$
    - 针对安全性：设置固定的机间最小距离；设置最大移动范围；随机选择探索或向最远无人机移动；确认目标点不在障碍物内；使用 ego-planner 生成轨迹用于下次观测的目标点
    - 针对解的全局最优：将非凸原问题松弛为 SDP（semi-definite programming）问题，松弛后为凸问题
    - 视觉测量：检测使用 YOLOv8，训练集来自现实，使用 NOKOV 自动标注后手动微调；跟踪使用 BoT-SORT，并分配 ID
- 达到什么效果？
    - 原地旋转和随机移动，重复多轮，能够弥补 FOV 有限的缺陷
    - SDP相比与局部优化方法，计算出的旋转的 MAE 均值和波动均更小，计算消耗时间更短
    - 随即移动能够打破具有对称性的初始队形
- 存在什么不足？
    - 无人机数目 N 需要事先给定
    - 相对观测的误差对系统的影响未被探索和分析，作者的未来工作是提高观测噪声的阈值，以更鲁棒地处理视觉识别中的不确定性
- 个人疑惑
    - 建模部分公式似乎有些错误
    - $rank(Z^*) \leq N+1$ 是否意味着所有观测均为相互？不是的话意味着什么？

## A Bearing-Angle Approach for Unknown Target Motion Analysis Based on Visual Measurements

> IJRR <br>
原文：[arXiv:2401.17117](https://arxiv.org/pdf/2401.17117.pdf)
<br>
视频：[【IJRR最新成果】利用被忽视的视觉信息大幅提升目标定位可观性](https://www.bilibili.com/video/BV1EC411z7Lz/?spm_id_from=333.337.search-card.all.click&vd_source=e371652571b1539bbd501fb7adb6cfc4)

- 针对什么问题？
    - 估计目标的运动状态


