---
layout: post
title:  "IMU Preintegration"
date:   2024-11-23 15:15:00 +0800
tags: 
  - Quaternion
categories:
  - math
---


从因子图的角度来看，IMU 预积分解决的问题是：约束的表达式与状态变量的值有关，也就是当状态变量更新后，
约束的表达式变化了，此时需要重新计算约束表达式里的一些参数，而不是能够重复利用固定的约束表达式。
而预计分则可以让原本会变的约束表达式变成固定的表达式，与状态变量无关。


### 预积分整体的推导思路

从两个最基本的方程组出发，一个是动力学方程组，另一个是最原始的测量方程组，之后利用匀速旋转和加速度的假设，进行普通的积分，然后将状态的增量都变换到相邻两帧中的第一帧机体坐标系下，再经过一系列的移项构造，使得方程右侧的参数可从上到下依次计算得出，且与状态无关，从而保证了状态在优化后发生值的变动，不会导致预计分方程右侧需要重新计算。

首先从最基本的动力学方程出发

$$
\dot{R} = R [\omega] \\ 
\dot{v} = a \\
\dot{p} = v
$$

其中，$R$ 是将一个向量从 局部（local） 坐标系下的表示变换到 全局（global） 坐标系下的表示。$\omega$ 是 local 坐标系下的机体运动角速度。


转换成积分形式

$$
R_{i+1} = R_i \oplus \int_{t_i}^{t+1} \omega \mathbf{d}t \\
v_{i+1} = v_i + \int_{t_i}^{t+1} a \mathbf{d}t \\ 
p_{i+1} = p_i + \int_{t_i}^{t+1} v \mathrm{d}t
$$

然后在这段时间内假设 $\omega$ 和 $a$ 是恒定值，也就是进行零阶近似

$$
R_{i+1} = R_i \oplus \omega \Delta t \\
v_{i+1} = v_i + a \Delta t \\
p_{i+1} = p_i + v_i \Delta t + \frac12 a \Delta t^2
$$

考虑 IMU 的最原始的测量模型：

$$
a_m = R^T(a-g) + b_a + n_a \\
\omega_m = \omega + b_g + n_g
$$

其中， $a_m$ 是局部坐标系下的加速度计测量值，$\omega_m$ 是。$a$ 是全局坐标系下的加速度真值，$g$ 是全局坐标系下的重力加速度，$b_a$ 是加速度计测量时的偏置，$n_a$ 是加速度计测量时的高斯白噪声。$\omega$ 是局部坐标系下的角速度真实值，$b_g$ 是陀螺仪的偏置，$n_g$ 是陀螺仪测量时的高斯白噪声。

把 原始测量模型 代入到 状态演进方程 上：

$$
R_{i+1} = R_i \oplus (\omega_m - b_g - n_g) \Delta t \\
v_{i+1} = v_i + R_i (a_m - b_a - n_a) \Delta t + g \Delta t \\
p_{i+1} = p_i +v_i \Delta t + \frac12 R_i(a_m - b_a - n_a) \Delta t^2 + \frac12 g \Delta t^2
$$

考虑 $i$ 时刻 和 $j$ 时刻之间的演进，并将这段时间分割成多个小时间块 $\Delta t_k, k = i, i+1, \cdots, j$，可以得到：

$$
R_j = R_i \prod_{k = i}^{j-1} \mathrm{Exp}(\omega_{m_k} - b_g - n_g) \Delta t_k \\
v_j = v_i + \sum_{k=i}^{j-1} R_k(a_{m_k} - b_a - n_a) \Delta t_k + g \sum_{k=i}^{j-1} \Delta t_k \\
p_j = p_i + \sum_{k=i}^{j-1} v_k \Delta t_k + \frac12 \sum_{k=i}^{j-1} R_k(a_m - b_a -n_a) \Delta t 
$$



