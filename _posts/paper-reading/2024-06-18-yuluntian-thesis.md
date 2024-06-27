---
layout: post
title:  "[Thesis Reading] Algorithms and Systems for Scalable Multi-Agent Geometric Estimation"
date:   2024-06-16 21:08:00 +0800
tags: 
    - slam
categories:
    - paper reading
---

目的是从存在噪声的局部测量中，构建全局一致的环境几何模型（geometry model of enviroment）。环境几何模型包括机器人轨迹、目标姿态和三维地图等。

存在的挑战：
- 由于各个智能体的状态估计问题之间的耦合
- 由实际计算能力和通信能力引起的较差的数值条件

因此考虑 `合作式优化`。

中心式优化的优点：
- 数据管理（data management）较为方便
- 可以使用现成的求解器

中心式优化的缺点：
- 有扩展性问题，当越来越多的智能体进入系统，中心服务器将成为计算瓶颈

分布式优化的优点：
- 重复利用各智能体上的计算资源

中心式的网络架构（服务器客户端架构），不意味着计算也是中心式的，此时计算也可以是分布到不同智能体上，如联邦学习。适用于已经有一些服务器客户端通信架构的设施的场景。相比于全分布式的通信架构，有两个好处：

- it enables collaborative estimation at a higher level of granularity
-  it leads to faster optimization (in the sense of significantly smaller number of iterations) by leveraging global information accessible by the server

 **技术挑战** One group of challenges arises from the underlying optimization problems：
 - First, most problems of practical interest are non-convex due
  to geometric constraints on the search space (e.g., estimating
   robot orientations on the group of rotations).To handle this
    issue, certifiable algorithms [13–16] that are capable of 
    verifying global optimality of candidate solutions (or 
    otherwise declaring failure of verification) are desirable,
     but are unexplored in the multi-agent setting prior to this 
     thesis
 - many real-world problem instances are ill-conditioned, which 
 could slow down numerical optimization or even cause erratic 
 behaviors.
    - In our applications, poor conditioning is usually a result 
    of the poor connectivity of the underlying measurement 
    graph, which characterizes the couplings among the 
    individual geometric states to be estimated. 
    - In some cases, poor conditioning also arises as a result 
    of the strong nonlinearity of the measurement model, e.g., 
    when working with projective measurements from monocular 
    cameras. 
    - Furthermore, larger problems tend to have worse 
    conditioning, which makes scaling to more agents or longer 
    operation time challenging.

    The second group of challenges arises from operational 
    constraints imposed by real-world communication networks. 
    For instance, in wireless ad hoc networks, connections 
    among robots are usually opportunistic and subject to high 
    latency. To address this issue, algorithms that are designed 
    to cope with communication delays are highly desirable. In 
    addition, communication constraints may also be manifested 
    in the form of limited bandwidth. This issue is especially 
    relevant when one wishes to transmit large-scale geometric 
    models such as large 3D maps. In general, transmitting 
    large models under limited bandwidth may result in long 
    delays, especially if such communication needs to be 
    repeated, e.g., during iterative optimization.


### rotation averageing

$$
\underset{R = (R_1,\cdots,R_n) \in SO(d)^n}{\mathrm{minimize}} \mathcal{k}_{ij} d(R_i\tilde{R}_{ij},R_j)
$$

separator and interior vertices

### PGO

### BA

在标准 BA 中，路标作为分割节点，此时所有位姿节点都是孤立的，不再按照是否连接划分子图，而是按位姿节点属于哪个机器人的位姿进行划分。


## Literature Review

### CSLAM Front-End

主要是机间的回环检测。

A complementary
line of work develops efficient methods for distributed geometric verification.

### CSLAM Back-End

> 疑问：黎曼梯度和欧几里得梯度有什么区别

Choudhary et al. [38] develop DGS, a two-stage approach for finding approximate
solutions to multi-robot PGO in the distributed setting.
- relaxing non-convex SO(d) constraints and priject result to SO(d)
- the rotation estimaite results are then used to initialize a single Gauss-Newton interation on full PGO problem.

multi-stage and besed on distributed Riemannian gradient descent



