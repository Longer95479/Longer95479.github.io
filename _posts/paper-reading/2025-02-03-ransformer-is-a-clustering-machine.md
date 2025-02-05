---
layout: post
title:  "Transformer is a clustering machines"
date:   2025-02-03 21:48:00 +0800
tags: 
    - transformer 
categories:
    - paper reading
---

```
2012    2015    2018 2019 2020  2022        2023
Alex    Res     GPT  GPT2 GPT3  chatGPT     GPT4
                BERT            Chinchilla  Llama2
                                            Claude2
```

## ResNets

$x_0$ 可以是图片像素强度向量，经过多层网络后，学习到特征 $x_L$ (Learned representation)，该特征相比于原始图像更利于图像分类。

$$
x_0 \rightarrow x_1 \rightarrow \dots \rightarrow x_L 
$$

$$
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
\begin{align}
x_{k+1} &= x_k + \sigma(W_k x_k + b_k) \\
& \downarrow \mathrm{Continuous\ time}\\
\dot x(t) &= \sigma(W_t x(t) + b_t)
\end{align}
$$

此时，我们得到一个关于 $x$ 的动态系统，因此可以用动态理论和控制理论进行分析。需要与神经网络中常提到的“训练的动态系统”作区分，训练的动态系统关心的是训练的过程中参数的演进，且只考虑一层（本文并不关注 *参数* 的动态系统）：

$$
\dot \theta (t) = - \nabla_{\theta} \sum_{i=1}^n (Y_i - f_{\theta(t)}(X_i))^2,\ \theta \in \mathbb{R}^N,\ N \rightarrow \infty
$$


NN 可以理解为一种流动映射（flow map, a NN is a compact representation of a funtion that can be represented as a flow map）：

$$
\mathrm{NN = Flow Map}
$$

$$
f_{\theta}: x(0) \mapsto x(T); \mathbb{R}^d \to \mathbb{R}^d
$$


这似乎是在说，NN 不是直接参数化函数本身（有一个表格直接告诉你 f(x) 的值），而是参数化了某条曲线，它将会通过动力学告诉你如何从一个点到达另一个点。


## Transformer

Prompt 可以是文本: 

$$
\begin{align}

& \mathrm{\boxed{The}\  \boxed{quick}\ \boxed{bro}\boxed{wn}\ \boxed{fox}\ \boxed{jumps}\ \boxed{over}\ \boxed{the}\ \boxed{lazy}\ \boxed{dog}} \\

& \downarrow \hspace{6.5em} \mathrm{position + random map} \hspace{6.5em} \downarrow \\

& x_1 \hspace{10.5em} \cdots \hspace{11em} x_n

\end{align}
$$

也可以是图片（整张图片被分割成多个图像块，每个图像块会被编码，附加上以某种顺序读取图片块的位置编码）：

```
 -------             ------- 
|       |           |   |   |
|       |   ---->   |-------|
|       |           |   |   |
 -------             -------
whole picture       pitches
```

因为 tokens 已经作了位置编码，因此可以将输入视为一个集合，而不考虑顺序（e.g. $n=512,\ d=200$），之后将集合视为经验测量得到的概率分布:

$$
\{x_1 \cdots x_n\} \in \mathbb{R}^d: \mathrm{tokens}
$$

$$
\{x_1 \cdots x_n\} \iff \frac1n \sum_{i=1}^n \delta_{x_i}
$$

promp = input = probability measure on tokens （likehood of token being next）

output = probability measure on tokens （empirical dist. of tokens in prompt）

输入给 transformer 的是关于 tokens 的概率测量。而输出则是给定这个句子（经验分布）后下一个 token 的似然概率分布。

虽然输入和输出中的 probability 含义不同，但在数学上看是相同的，或者说是属于同一个集合内的，因此可以套用 ResNet 中 Map flow 的概念。

$$
\mathrm{Transformer = Flow map}
$$

$$
f_{\theta}: \mu(0) \mapsto \mu(T); \mathcal{P}(\mathbb{R}^d) \to \mathcal{P}(\mathbb{R}^d)
$$

其中，$\mathcal{P}(\mathbb{R}^d)$ 是指 $\mathbb{R}^d$ 上的概率分布的空间/集合。


此时，我们可以给出 $\mu(0)$：

$$
\mu(0) = \frac1n \sum_{i=1}^n \delta_{x_i}
$$

那么动力学方程应该是什么呢？我们该如何移动给出的概率分布呢？

$$
\partial_t \mu(t) = ?
$$

我们可以通过移动点的位置来改变概率分布。而对于 Transformer，它实际上实现的是一个在平均场下的相互作用粒子系统（Mean-Field interacting particle system）:

$$
\dot x_i(t) = X_t (\mu(t))(x_i(t)),\ i = 1,\cdots, n
$$

在此，粒子是 token，它的速度由向量场 $X_t$决定，而向量场受粒子的位置 $x_i(t)$ 以及当前时刻所有粒子/tokens 的分布（以一种整体的方式）。

Mean-Field interacting particle siystem => Continuity equation

$$
\dot x_i(t) = X_t (\mu(t))(x_i(t))
$$

$$
\Downarrow
$$

$$
\partial_t \mu(t) + \mathrm{div} (\mu(t)X_t[\mu(t)]) = 0
$$

Self-attention dynamics = Spacial choice of $X_t[\mu(t)](\cdot)$


### 参考

[【MIT Philippe Rigollett】数学视角下的Transformer](https://www.bilibili.com/video/BV16ifaYyE8Z/?spm_id_from=333.1007.tianma.1-2-2.click&vd_source=e371652571b1539bbd501fb7adb6cfc4)


