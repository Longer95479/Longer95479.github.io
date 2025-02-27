---
layout: post
title:  "What is Motion Planning"
date:   2025-02-27 15:56:00 +0800
tags: 
  - motion planning
categories:
  - motion planning
---


## 什么是运动规划（motion planning）

运动规划：使用可计算的方法为机器人生成一套运动行为，以完成给定的任务。

> Motion planning: the use of computational methods to generate a robot's motion to achieve a specified task.


运动规划处于自主机器人系统组件中的中心位置。

它的输入可以分成三类：

- 系统状态（system state）
    - 机器人自身状态
    - 周围的地图
    - 其他机器人的状态和意图

- 系统模型（system model）：假设给定机器人的某项决策后，预测系统将会如何变化演进

- 任务要求（task specification）：一般包含一个或多个损失函数，衡量了候选运动的质量、候选运动满足约束的质量


如果不考虑机器人的动力学，那么运动规划问题就变成了路径规划问题：

> Kinematic motion planning (aka Path planning): the use of computational methods generate a continuous path between two robot configurations while respecting joint limits and avoiding obstacles.

## 运动规划的挑战

为什么运动规划会这么难？可以把运动规划想象成下棋，如果考虑后续的 n 步，每步都有 m 种决策，那么将会有 $m^n$ 种可能性，可能性将随 m、n 增加而迅速增长，更何况实际运动轨迹是连续的场景，因此暴力搜索不是解决办法，我们需要考虑更加精巧的方法来降低复杂度。

## 运动规划的分类

总体可以分成两类：

- 启发式方法（Heuristic methods）：`启式发` 意味着大部分情况下，这些方法能够满足要求，但没有成功或最优性的保证
    - 行为脚本（behavior script）
    - 生成-评分（generate-and-score）

- 更具原则性的方法（More principled methods）
    - 虫子算法（bug algorithms）
    - roadmap planners
        - grid search
        - visibility graph
    - cell decomposition
        - trapezoidal decomposition
        - approximate cell decomposition


## 参考

[Section III. Motion  - Kris Hauser - University of Illinois at Urbana-Champaign](https://motion.cs.illinois.edu/RoboticSystems/)