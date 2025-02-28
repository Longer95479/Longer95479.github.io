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

### 原理

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

定义这些状态变量后，结合测量值，可构建联合的残差函数：

$$
r(\mathcal{X}, z) = \left\| e_p - H_p \mathcal{X} \right\| + 
\left\| e_{\mathcal{B}} \right\|
$$

在本文中，我们关注相机相对于 imu 的位置和姿态。

### 实施

### 效果
