---
layout: post
title:  "Extrinsic Online Calibration on AirSLAM" 
date:   2025-01-15 14:00:00 +0800
tags: 
    - slam
    - calibration
categories:
    - AirSLAM Improvement
---


AirSLAM 在公开数据集上的效果很不错，但在笔者自己录制的数据集（螺旋快速运动）上效果很差，具体如下：

- AirSLAM 使用 imu 和相机，定位漂移很大
- 而相同的外参在 vins-fusion 上的效果却还不错。
- AirSLAM 仅使用双目，漂移小，较为正常

根据第一条和第三条信息，可以推断出是 imu 和相机之间的协调出了问题，可能的原因是: 

- 时间基准不同：公开数据集的 imu 与 图像帧有硬件同步且参考同一个时间基准，而自己录制的数据集未作同步且不是同一个时间基准，而 vins-fusion 在运行过程中在线校准 imu 和图像帧的时间偏移。这是二者存在的一个较为明显的区别，
- 相机与 imu 之间的外参不够准确：虽然 vins-fusion 和 AirSLAM 使用的外参一致，但 vins 有对外参做在线估计校准，而 AirSLAM 没有。


在 [为 AirSLAM 增加在线时间校准](https://longer95479.github.io/temporal-online-calibration-exp) 中，已经对比了增加在时间校准前后的精度，得到的结论是 `时间基准不同不是引起该问题的原因`。因此本文将验证第二种可能性：相机与 imu 之间的外参不够准确。

验证的方法是，为 AirSLAM 增加外参在线校准，对比开启该功能与关闭该功能的定位精度，如果提升明显，则说明确实是相机与 imu 之间的外参不准确导致的较大漂移。

## 原理

### 状态的定义

假设机器人配置多个相机和一个 IMU。

$$
\begin{align}
\mathcal{X} = [& \mathrm{x}_0, \mathrm{x}_1, \cdots, \mathrm{x}_{n-1}, \\
& L_0, L_1, \cdots, L_{l-1}, \\
& \mathrm{x}_{c_0}, \mathrm{x}_{c_1}, \cdots,\mathrm{x}_{c_{m-1}}]
\end{align}
$$

其中 $\mathrm{x}_i,\ i = [0, n)$ 是大小为 $n$ 的滑动窗口内的核心状态，包括机体相对于世界系的位置、速度、姿态，以及加速度计和陀螺仪的零偏

$$
\mathrm{x}_i = [p_i^w, v_i^w, R_i^w, b_a, b_g]
$$


$L_j, j= [0, l)$ 是滑窗内被历史观测二次及以上的路标，如果使用 3D 表示，则 $L_j = [x_j, y_j, z_j]^T$，如果使用逆深度表示，则 $L_j = \lambda_j$。

$x_{c_k}$ 是在线校准的状态，包括第 k 个相机相对于 imu 的位置、姿态、时间偏移。

$$
\mathrm{x}_{c_k} = [p_{c_k}^w, R_{c_k}^w, t_{d_k}]
$$

### 构建残差函数

定义这些状态变量后，结合测量值，可构建联合的残差函数：

$$
\mathbf{r}(\mathrm{z}, \mathcal{X}) = \left\| \mathbf{e}_p - \mathbf{H}_p \mathcal{X} \right\|^2 + 

\sum_{i \in \mathcal{B}} \left\| \mathbf{e}_{\mathcal{B}}(\mathrm{z}_i^{i+1}, \mathcal{X}) \right\|_{\mathbf{P}_i^{i+1}}^2 + 

\sum_{(l,j) \in \mathcal{O}} \left\| \mathbf{e}_{\mathcal{C}}(\mathrm{z}_l^j, \mathcal{X}) \right\|^2_{\mathbf{P}_l^j} +

\sum_{k \in \mathcal{C}} \left\| \mathbf{e}_{\mathrm{ext}}(z_{c_k}, \mathrm{x}_{c_k}) \right\|^2_{\mathbf{P}_{c_k}}
$$

其中：
- $\left\| \mathbf{e}_p - \mathbf{H}_p \mathcal{X} \right\|^2$ 是先验因子

- $\sum_{i \in \mathcal{B}} \left\| \mathbf{e}_{\mathcal{B}}(\mathrm{z}_i^{i+1}, \mathcal{X}) \right\|_{\mathbf{P}_i^{i+1}}^2$ 是 IMU propagation 因子

- $\sum_{(l,j) \in \mathcal{O}} \left\| \mathbf{e}_{\mathcal{C}}(\mathrm{z}_l^j, \mathcal{X}) \right\|^2_{\mathbf{P}_l^j}$ 是视觉重投影因子

- 本篇中最重要的因子则是 `外参先验因子`：$\sum_{k \in \mathcal{C}} \left\| \mathbf{e}_{\mathrm{ext}}(z_{c_k}, \mathrm{x}_{c_k}) \right\|^2_{\mathbf{P}_{c_k}}$

涉及到的集合：

- $\mathcal{B}$ 是滑动窗口内的关键帧的 index 集合

- $\mathcal{O}$ 是滑动窗口内路标 index 与 观测到该路标的关键帧 index 对 $(l,j)$ 的集合

- $\mathcal{C}$ 是所有相机的编号的集合

我们的目标是：

$$
\mathcal{X}^* = \mathrm{argmin}_{\mathcal{X}}\  \mathbf{r}(\mathrm{z}, \mathcal{X})
$$

VINS-Fusion 的残差构造满足上述描述，而 AirSLAM 则需对视觉重投影因子稍作修改。因为在 VINS 中，路标点被滑动窗口内的至少两帧观测到，才会被加入滑窗中；而在 AirSLAM 中，一个路标点同样需要被两帧不同关键帧观测，但只要求至少有一帧在滑窗中即可，其余的观测帧可不位于滑窗内，此时我们可以把视觉因子修改为：

$$
\sum_{(l,j) \in \mathcal{C}} \left\| \mathbf{e}_{\mathcal{C}}(\mathrm{z}_l^j, \mathcal{X}_{old}, \mathcal{X}) \right\|^2_{\mathbf{P}_l^j}
$$

其中，$\mathcal{X}_{old}$ 是滑窗外的关键帧，作为参数而不是被优化变量，出现在因子当中。此时 下标 $j$ 不再局限于滑窗内的关键帧下标了，也可能是滑窗外不被优化的关键帧下标。


### 一些因子的具体形式

在本文中，我们关注相机相对于 imu 的位置和姿态，需要把 $\sum_{(l,j) \in \mathcal{C}} \left\| \mathbf{e}_{\mathcal{C}}(\mathrm{z}_l^j, \mathcal{X}_{old}, \mathcal{X}) \right\|^2_{\mathbf{P}_l^j} $ 具体化才能作进一步的分析。该残差具体可以写为：

$$
\mathbf{e}_{\mathcal{C}}(\mathrm{z}_l^j, \mathcal{X}_{old}, \mathcal{X}) = 
\mathrm{z}_l^j - \pi \left(R_j^{wT} (L_l - p_j^w) \right)
$$

其中，$\pi (\cdot)$ 是相机的投影函数。

## 实施

对 AirSLAM 增加了外参在线校准，提交在
[Commit 0dc8f6d: add online calibration of extrinsic between camera and imu.](https://github.com/sair-lab/AirSLAM/commit/0dc8f6d6d62eae5804fa85b85d2c8233e97cbce9)。

AirSLAM 与 ORB-SLAM 类似，使用 `G2O` 库作为后端优化的框架。因此，为了增加所述功能，我们需要增加顶点和边，并修改一些现有的边：

- 增加的顶点是 `外参` 状态变量
- 增加的边是 `外参先验`
- 修改的边是 `视觉重投影`，从二元边变成了多元边



### 增加的核心文件及内容

增加的文件为：
- `src/g2o_optimization/edge_extrinsic_prior.cc`
- `include/g2o_optimization/edge_extrinsic_prior.h` 
- `src/g2o_optimization/vertex_extrinsic.cc`
- `include/g2o_optimization/vertex_extrinsic.h`

提交在上个 commit，但在本次 commit 中使用到的（用到了新定义的 g2o 多元边的功能）：
- `src/g2o_optimization/edge_project_point_td.cc`
- `include/g2o_optimization/edge_project_point_td.h`

### 修改的核心文件及内容


## 效果

使用的数据集为

对比结果：


