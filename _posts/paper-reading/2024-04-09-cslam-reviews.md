---
layout: post
title:  "a CSLAM and Mutual Localization Review"
date:   2024-04-09 21:00:00 +0800
tags: slam
categories:
---

CSLAM 全称为 collaborative SLAM，用于估计机器人间的 `相对位姿` 和 `全局一致的轨迹`。本篇将介绍 CSLAM 的整体框架，侧重于介绍 CSLAM 的初始化阶段，以及机器人间的相互定位。


- [*匿名条件* 下的相互定位](#certifiably-optimal-mutual-localization-with-anonymous-bearing-measurements)
- [*部分观测* 下的相对定位](#a-bearing-angle-approach-for-unknown-target-motion-analysis-based-on-visual-measurements)
- [同时相对定位与*时间同步*](#simultaneous-time-synchronization-and-mutual-localization-for-multi-robot-system)


### FACT: Fast and Active Coordinate Initialization for Vision-based Drone Swarms

> IROS 投稿
<br>
原文：[arXiv:2403.13455](https://arxiv.org/abs/2403.13455)
<br>
视频：[FACT: Fast and Active Coordinate Initialization for Vision-based Drone Swarms](https://www.bilibili.com/video/BV1sA4m1A7DD/?vd_source=e371652571b1539bbd501fb7adb6cfc4)



针对什么问题？

- 匿名+部分相互观测：针对有着 SWaP(size, weight and power) 约束的无人机集群，仅使用`基于视觉的匿名的部分相互观测`，而*不使用其他传感器*辅助（如 UWB，动捕和特制信标），如何实现`坐标系的初始化`？

- `基于视觉的匿名的相互观测` 带来了新的挑战：
    - 有限的 FOV：视角和传感器距离有限，某些无人机之间不存在相互观测
    - 匿名：无法直接知晓通过视觉检测到的无人机的 id 是多少
    - 安全性：在初始化阶段，无人机需要避免和其他无人机以及环境发生碰撞

采用什么方法？

- 问题建模：相互观测向量求和为零向量 -> 最小二乘优化问题

- 整体思路：原地旋转，判断相互测量是否完整，若不满足，则随机移动，重复以上步骤。当 $rank(Z^*) \leq N+1$，意味着所有测量均已获得，均为相互的。此时可求解出各机坐标系的相对旋转 $R_{t_0}$，不受匿名条件的影响。求解出 $R_{t_0}$ 后，视觉测量和 VIO 的关联问题可以转化为求解关联矩阵 $A_i$，等价于求解二分图匹配问题，可使用匈牙利算法求解。

- 针对有限的 FOV：原地旋转 ＋ 多轮随机移动

- 针对匿名的测量：使用匈牙利算法求解 $A_i$

- 针对安全性：设置固定的机间最小距离；设置最大移动范围；随机选择探索或向最远无人机移动；确认目标点不在障碍物内；使用 ego-planner 生成轨迹用于下次观测的目标点

- 针对解的全局最优：将非凸原问题松弛为 SDP（semi-definite programming）问题，松弛后为凸问题

- 视觉测量：检测使用 YOLOv8，训练集来自现实，使用 NOKOV 自动标注后手动微调；跟踪使用 BoT-SORT，并分配 ID

达到什么效果？
- 原地旋转和随机移动，重复多轮，能够弥补 FOV 有限的缺陷
- SDP相比与局部优化方法，计算出的旋转的 MAE 均值和波动均更小，计算消耗时间更短
- 随即移动能够打破具有对称性的初始队形

存在什么不足？
- 无人机数目 N 需要事先给定
- 相对观测的误差对系统的影响未被探索和分析，作者的未来工作是提高观测噪声的阈值，以更鲁棒地处理视觉识别中的不确定性

个人疑惑
- 建模部分公式似乎有些错误
- $rank(Z^*) \leq N+1$ 是否意味着所有观测均为相互？不是的话意味着什么？


### Bearing-based Relative Localization for Robotic Swarm with Partially Mutual Observations

> RA-L
<br>
原文：[arXiv:2210.08265](https://arxiv.org/abs/2210.08265)
<br>

针对什么问题？

- 先前的工作已经开发了可证明的鲁棒的求解器，用于 *每对* 机器人之间的相对变换估计，未考虑部分观测的情况

- 基于地图的相对位姿估计所需带宽大，受环境影响

- 仅使用 2D 视觉检测的基于测角的相互定位，由于常用的相机 FOV 有限，机器人通常只能观测到部分机器人，形成*部分观测图（partial observation graph）*

- 由于问题建模得到的公式极端非凸，传统局部优化方法可能陷入局部最小，这在相对定位问题中十分常见

采用什么方法？

- 针对部分相互观测的情况，本文基于不同的变量边缘化方法，推导了两种可解的问题建模公式，得到一个统一的在 Stiefel 流形上的最小值问题，*联合优化所有机器人的位姿*。作者对这两种公式化进行了细致分析和对比。

- 针对问题非凸的情况，本文将原始非凸问题松弛为 SDP（semi-definite programming） 问题，SDP 问题是凸的，能够求得全局最小值。此外，本文还提供了一个充分条件，在该条件下，能够严格保证无噪声情况下松弛的紧致性（严密性）

- 为了避免现实中噪声的影响导致解不准确，使用了 *秩约束优化*

达到什么效果？

- 相比于局部优化算法（黎曼流形优化、LM优化算法），本文的方法能够实现解的最优
- 使用了变脸消除策略，变量数目固定，因此 *计算时间* 只和机器人数目有关。1Hz的坐标系调整
- 鲁棒性高，体现在抗噪声能力比*纯SDP*、*黎曼流形优化*强
- 实际实验使用 鱼眼相机 和 *带标签的LED* 来获取相互观测

存在什么不足？

- 使用了 *带标签的LED* 来产生 *非匿名* 的相互观测
- 在每个机器人内都进行了问题建模和优化求解，计算冗余了

个人疑惑

- 如何松弛？什么是 SDP 问题？
- 解的好坏可以从 $Z^*$ 的秩和某个 cost 反映出来，有待进一步了解




### Certifiably Optimal Mutual Localization with Anonymous Bearing Measurements

> RA-L with IROS 2022
<br>
原文：[arXiv:2203.09312](https://arxiv.org/abs/2203.09312)


针对什么问题？


采用什么方法？


达到什么效果？


存在什么不足？



### Meeting-Merging-Mission: A Multi-robot Coordinate Framework for Large-Scale Communication-Limited Exploration

> IROS 2022
<br>
原文：[arXiv:2203.09312](https://arxiv.org/abs/2203.09312)


针对什么问题？


采用什么方法？


达到什么效果？


存在什么不足？



### Simultaneous Time Synchronization and Mutual Localization for Multi-robot System

> ICRA 2024
<br>
原文：[arXiv:2311.02948](https://arxiv.org/pdf/2311.02948.pdf)
<br>
视频：[Simultaneous Time Synchronization and Mutual Localization for Multi-robot System](https://www.bilibili.com/video/BV1ew411r7z8/?vd_source=e371652571b1539bbd501fb7adb6cfc4)

针对什么问题？


采用什么方法？


达到什么效果？


存在什么不足？



### CREPES: Cooperative RElative Pose Estimation System

> IROS 2023
<br>
原文：[arXiv:2302.01036](https://arxiv.org/abs/2302.01036)
<br>
视频：[CREPES: Cooperative RElative Pose EStimation towards Real-World Multi-Robot Systems](https://www.bilibili.com/video/BV1CW4y1Y79q/?spm_id_from=333.999.0.0&vd_source=e371652571b1539bbd501fb7adb6cfc4)

针对什么问题？


采用什么方法？


达到什么效果？


存在什么不足？




### A Bearing-Angle Approach for Unknown Target Motion Analysis Based on Visual Measurements

> IJRR <br>
原文：[arXiv:2401.17117](https://arxiv.org/pdf/2401.17117.pdf)
<br>
视频：[【IJRR最新成果】利用被忽视的视觉信息大幅提升目标定位可观性](https://www.bilibili.com/video/BV1EC411z7Lz/?spm_id_from=333.337.search-card.all.click&vd_source=e371652571b1539bbd501fb7adb6cfc4)

针对什么问题？

- 估计目标的运动状态

采用什么方法？



达到什么效果？


存在什么不足？



### DIDO: Deep Inertial Quadrotor Dynamical Odometry

> RA-L with IROS 2022
<br> 
原文：[arXiv:2203.03149](https://arxiv.org/abs/2203.03149)
<br>
视频：[DIDO: Deep Inertial Quadrotor Dynamical Odometry](https://www.bilibili.com/video/BV1dU4y1Z773/?spm_id_from=333.999.0.0&vd_source=e371652571b1539bbd501fb7adb6cfc4)
