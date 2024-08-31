---
layout: post
title:  "State Estimate"
date:   2023-10-06 16:00:00 +0800
tags: slam
categories:
    - math
---

## 状态估计分类
观测方程简化为

$$
    z_t = h(x_t, w_t) = x_t + w_t
$$

估计量是测量值的函数

$$
    \hat{x}(Z^k),\ Z^k = \{z_j\}_{j=1}^k
$$

$x_t$ 有两种建模：
- 非随机：未知的确定值
- 随机：具有先验 pdf 的随机变量的一次实现

对应出两类估计方法：
- 非贝叶斯方法
    - 最大似然估计（ML）
    - 最小二乘估计（LS）
- 贝叶斯方法（用到了 $x$ 的统计性质，如期望、方差）
    - 最大后验估计(MAP)
    - 最小均方误差估计（MMSE）

当测量数据个数趋于无穷时，最大似然和最大后验的估计结果相同，即

$$
    \hat{x}^{MAP}(Z^k) = \hat{x}^{ML}(Z^k) = \bar{z},\ k \rightarrow \infty
$$

当噪声是加性零均值高斯噪声时

$$
    \hat{x}^{LS}(Z^k) = \hat{x}^{ML}(Z^k) = \bar{z},\ k=1
$$

$$

    \hat{x}^{MMSE}(Z^k) = \hat{x}^{ML}(Z^k),\ k=1
$$

当 $p(x)$ 是均匀分布时

$$
    \hat{x}^{MAP}(Z^k) = \hat{x}^{ML}(Z^k)
$$

## 度量

所有方法本质上都蕴含着一个非负的度量函数 $D(\hat{x},Z^k)$，`估计量（estimator）`的作用就是给定多个测量数据 $Z^k$ 后，返回一个`估计值（estimate）` $\hat{x}$，使得 $D(\hat{x},Z^k)$ 最小，即

$$
\hat{x}(Z^k) = \underset{\hat{x}}{\mathrm{arg\ min}}D(\hat{x},Z^k)\\
$$

各类估计方法对应的度量函数如下

$$
\begin{array}{ll}
\hline
method & distance\ D(\hat{x})\\
\hline
MAP& -p(\hat{x}|Z^k) \\
MMSE& E[(x|Z^k-\hat{x})^2]\\
\\
ML& -p(Z^k|\hat{x}) \\
LS& \sum_{j=1}^k(z_j-h(\hat{x}))^2\\
\hline
\end{array}
$$

贝叶斯方法认为 $x$ 是随机变量的实现，体现在公式上便是：度量函数用到了 $p(x \mid Z^k)$。


## SLAM 中的最大后验估计

$$
\begin{align}
\hat{x} &= \underset{\hat{x}}{\mathrm{arg\ max}}\ p(\hat{x}| Z^k)\\
&= \underset{\hat{x}}{\mathrm{arg\ max}}\ p(\hat{x},Z^k)\\
&= \underset{\hat{x}}{\mathrm{arg\ max}}\ p(Z^k|\hat{x})p(\hat{x})\\
&\scriptsize{各个测量独立} \\
&= \underset{\hat{x}}{\mathrm{arg\ max}}\ p(\hat{x})\prod_{z_j\in Z^k} p(z_j|\hat{x})\\
&\scriptsize{测量噪声满足高斯分布} \\
&= \underset{\hat{x}}{\mathrm{arg\ max}}\ p(\hat{x})\sum_{z_j\in Z^k} ||z_j-h_j(\hat{x}))||^2_{\Sigma_j}\\
\end{align}
$$

$h_j$ 是测量预测函数，根据状态的估计值计算期望的测量值。
