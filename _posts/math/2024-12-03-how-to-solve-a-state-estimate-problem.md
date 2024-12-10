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
&= \bold{h} + \bold{J} \Delta x
\end{align}
$$

$$
\begin{align}
& \sum_{all\ h} (h_0 + J^h_x \Delta x)^T(h_0 + J^h_x \Delta x) \\
&= \bold{h}^T \bold{h} + 2\bold{h}^T \bold{J} \Delta x + \Delta x^T \bold{J}^T \bold{J} \Delta x \\
&= \alpha_0 + 2\bold{h}^T \bold{J} \Delta x + \Delta x^T \bold{J}^T \bold{J} \Delta x
\end{align}
$$

$$
\min (2\bold{h}^T \bold{J} \Delta x + \Delta x^T \bold{J}^T \bold{J} \Delta x)
=
\min (\alpha_0 + 2\bold{h}^T \bold{J} \Delta x + \Delta x^T \bold{J}^T \bold{J} \Delta x) 
$$
令
$$
g(\Delta x) = 2\bold{h}^T \bold{J} \Delta x + \Delta x^T \bold{J}^T \bold{J} \Delta x 
$$

$$
\bold{J}^g_{\Delta x}  = 2 \bold{h}^T\bold{J} + 2\bold{J}^T\bold{J}\Delta x
$$

令 $$\bold{J}^g_{\Delta x}= 0$$
$$
2 \bold{h}^T\bold{J} + 2\bold{J}^T\bold{J}\Delta x = 0 \\
$$
从而得到所谓的正规方程，求解该方程，便得到移动的方向和大小 $\Delta x$
$$
\bold{J}^T\bold{J}\Delta x = -\bold{h}^T\bold{J}
$$


