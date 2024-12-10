---
layout: post
title:  "state estimate back end"
date:   2024-11-23 15:15:00 +0800
tags: 
  - Quaternion
categories:
  - math
---

## 整体思路

状态估计的本质是最大后验估计，联合概率密度函数一步步转化到标量函数的和，最后线性化迭代求解。

$$
\begin{align}
P(X|Z) &\propto P(X|Z)P(Z) \\
&= P(Z|X)P(X) \\
&= P(X) \prod P(z_{i}|x_{\{j\}}) 
\end{align}
$$

利用负对数将乘积转化为加法，我们称之为 `损失函数`，每一个 $h$ 称之为残差函数，而残差函数一般已经在流形的切空间对应的欧式空间上了，可以使用最原始的加减符号：

$$
\begin{align}
- \log [P(X) \prod P(z_{i}|x_{\{j\}})] 
&= \sum_{all\ h} h^T(x_{\{j\}}; z_{i})h(x_{\{j\}}; z_{i}) \\
& \scriptsize{x_{j} 扩充成 x} \\
&\approx \sum_{all\ h} (h_0 + J^h_x \Delta x)^T(h_0 + J^h_x \Delta x) \\
& \scriptsize{把 h_0 + J^h_x \Delta x 堆叠成列向量，然后展开，符号不变} \\
& = 
\begin{bmatrix}
\cdots & (h_0 + J^h_x\Delta x)^T & \cdots
\end{bmatrix} 
\begin{bmatrix}
\vdots \\
h_0 + J^h_x \Delta x\\
\vdots
\end{bmatrix} \\
\end{align}
$$

将上面的矩阵改写为更加紧凑的形式：

$$
\begin{align}
\begin{bmatrix}
\vdots \\
h_0 + J^h_x \Delta x\\
\vdots
\end{bmatrix}
&=  
\begin{bmatrix}
\vdots \\
h_0 \\
\vdots
\end{bmatrix}
+
\begin{bmatrix}
\vdots \\
J^h_x \\
\vdots
\end{bmatrix}
\Delta x \\
&= \mathbf{h} + \mathbf{J} \Delta x
\end{align}
$$

利用紧凑的形式，继续展开损失函数：

$$
\begin{align}
\sum_{all\ h} (h_0 + J^h_x \Delta x)^T(h_0 + J^h_x \Delta x) 
& = (\mathbf{h} + \mathbf{J} \Delta x)^T(\mathbf{h} + \mathbf{J} \Delta x) \\
&= \mathbf{h}^T \mathbf{h} + 2\mathbf{h}^T \mathbf{J} \Delta x + \Delta x^T \mathbf{J}^T \mathbf{J} \Delta x \\
&= \alpha_0 + 2\mathbf{h}^T \mathbf{J} \Delta x + \Delta x^T \mathbf{J}^T \mathbf{J} \Delta x
\end{align}
$$

其中，$\mathbf{h}$ 是很多 $h_0$ 的堆叠，$\Delta x$ 是所有状态变量的增量的堆叠，$\mathbf{J}$ 则是很多 $J^h_x$ 的堆叠，$J^h_x$ 是各个子损失函数对所有状态变量的雅可比矩阵。

展开后，可以看到有一个常数项，这对整个函数值的变化没有贡献，因此：

$$
\min (2\mathbf{h}^T \mathbf{J} \Delta x + \Delta x^T \mathbf{J}^T \mathbf{J} \Delta x)
=
\min (\alpha_0 + 2\mathbf{h}^T \mathbf{J} \Delta x + \Delta x^T \mathbf{J}^T \mathbf{J} \Delta x) 
$$
令 
$
g(\Delta x) = 2\mathbf{h}^T \mathbf{J} \Delta x + \Delta x^T \mathbf{J}^T \mathbf{J} \Delta x 
$

求雅可比矩阵
$$
\mathbf{J}^g_{\Delta x}  = 2 \mathbf{h}^T\mathbf{J} + 2\mathbf{J}^T\mathbf{J}\Delta x
$$

当其雅可比矩阵（其实是梯度的转置）为 零向量 时，函数取极小值：
$$\mathbf{J}^g_{\Delta x}= 0$$
$$
2 \mathbf{h}^T\mathbf{J} + 2\mathbf{J}^T\mathbf{J}\Delta x = 0 \\
$$

从而得到所谓的正规方程，求解该方程，便得到移动的方向和大小 $\Delta x$

$$
\mathbf{J}^T\mathbf{J}\Delta x = -\mathbf{h}^T\mathbf{J}
$$

最直接的求解方法为求逆：
$$
\Delta x = -(\mathbf{J}^T\mathbf{J})^{-1}\mathbf{h}^T\mathbf{J}
$$


## 李群函数的雅可比矩阵

从上一章可以看到，想要求解得到步长

