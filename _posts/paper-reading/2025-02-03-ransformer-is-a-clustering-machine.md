---
layout: post
title:  "Transformer is a clustering machines"
date:   2025-02-03 21:48:00 +0800
tags: 
    - transformer 
categories:
    - paper reading
---

## ResNets

$x_0$ 可以是图片像素强度向量，经过多层网络后，学习到特征 $x_L$ (Learned representation)，该特征相比于原始图像更利于图像分类。

$$
x_0 \rightarrow x_1 \rightarrow \dots \rightarrow x_L \\
L = \# layers
$$

将学习到的特征放入到传统的机器学习分类器（Classical ML step）中进行分类（例子为 logistic regression）：

$$
x_L \rightarrow y = \sigma (\beta^T x_L)
$$

在 2012 年以前，人们会将 $x_L$ 直接作为原始图片，放入到传统的分类器当中。

如果把神经网络看成一个整体，那么神经网络便是一个映射：

$$
x_L = f_{\theta}(x_0)
$$

$\theta$ 是可训练的参数。$f_{\theta}$ 可以通过多个函数的嵌套组合来得到：

$$
x_{k+1} = x_k + \sigma(W_k x_k + b_k),\ k = 0,\dots, L-1
$$

$W_k x_k + b_k$ 是仿射变换。$\sigma$ 是逐元素操作的非线性函数，例如 $\mathrm{max}(\cdot, 0)$。更详细一些，如果有一个向量 $x = [1,-2,3]$，应用 ReLU 函数（$f(x) = \max(\cdot,0)$）则得到 $f(x) = [\max(0,1), \max(0,-2), \max(0,3)] = [1,0,3]$

这就是传统的前馈神经网络，也称为多层感知器。

Neural ODEs:

$$
x_{k+1} = x_k + \sigma(W_k x_k + b_k) \\
\downarrow \mathrm{Continuous\ time}\\
\dot x(t) = \sigma(W_t x(t) + b_t)
$$

此时，我们得到一个关于 $x$ 的动态系统，因此可以用动态理论和控制理论进行分析。需要与神经网络中常提到的“参数的动态系统”作区分，参数的动态系统关心的是训练的过程中参数的演进（本文并不关注 *参数* 的动态系统）：

$$
\dot \theta (t) = - \nabla_{\theta} \sum_{i=1}^n (Y_i - f_{\theta(t)}(X_i))^2,\ \theta \in \mathbb{R}^n
$$



