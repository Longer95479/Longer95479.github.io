---
layout: post
title:  "a CSLAM and Mutual Localization Review"
date:   2024-04-09 21:00:00 +0800
tags: slam
categories:
---

CSLAM 全称为 collaborative SLAM，用于估计机器人间的 `相对位姿` 和 `全局一致的轨迹`。本篇将介绍 CSLAM 的整体框架，侧重于介绍 CSLAM 的初始化阶段，以及机器人间的相互定位。

## FACT: Fast and Active Coordinate Initialization for Vision-based Drone Swarms

原文链接：[arXiv:2403.13455](https://arxiv.org/abs/2403.13455)

- 针对什么问题？
    - 针对有着 SWaP(size, weight and power) 约束的无人机集群，仅使用`基于视觉的匿名的相互观测`，而不使用其他传感器辅助（如 UWB，动捕和特制信标），如何实现`坐标系的初始化`？
    - `基于视觉的匿名的相互观测` 带来了新的挑战：
        - 有限的 FOV：视角和传感器距离有限，某些无人机之间不存在相互观测
        - 匿名：无法直接知晓通过视觉检测到的无人机的 id 是多少
        - 安全性：在初始化阶段，无人机需要避免和其他无人机以及环境发生碰撞
- 采用什么方法？
    - 针对有限的 FOV：原地旋转，判断相互测量是否完整，进行多轮测量，直至 $rank(\mathrm{Z}^*) \leq N+1$，意味着所有测量均已获得，均为相互的。此时可求解出各机坐标系的相对旋转 $R_{t_0}$，不受匿名条件的影响
    - 针对匿名的测量：
    - 针对安全性
    - 针对解的全局最优：
- 达到什么效果？
    - a
- 存在什么不足？
    - a
- 个人疑惑

