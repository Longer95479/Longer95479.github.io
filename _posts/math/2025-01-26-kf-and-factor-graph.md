---
layout: post
title:  "Equivalence between Kalman Filter and Factor graph"
date:   2025-01-26 17:22:00 +0800
tags: 
  - Quaternion
categories:
  - math
---

$$
P(X,Z) = P(Z|X)P(X)
$$

先验（prior）：

$$
P(X_1)
$$

预测（prediction）：

$$
X_2^- = \argmin_{X_2} P(X_2)
$$

$$
\begin{align}
P(X_2) &= \int P(X_2 , X_1) \bf{d} X_1 \\
&= \int P(X_2 | X_1) P(X_1) \bf{d} X_1 
\end{align}
$$

更新（update）：

$$
X_2^* = \argmin_{X_2} P(X_2|Z_2)
$$

$$
P(X_2 | Z_2) \propto P(X_2, Z_2) = P(Z_2 | X_2) P(X_2)
$$

接下来考虑具体的形式。

- 先验

$$
P(X_1) \propto \exp({-\frac12 ||x_1 - \mu_1||^2_{\Sigma_1}})
$$

- 预测

$$
P(X_2 | X_1)P(X_1) \propto \exp ({-\frac12 ||x_2 - (Fx_1 + Bu)||^2_Q}) \cdot
\exp (-\frac12 ||x_1 - \mu_1||^2_{\Sigma_1})
$$

- 更新

$$
P(Z_2 | X_2)P(X_2) \propto \exp (-\frac12 ||Hx_2 - z_2||^2_R) \cdot \int P(X_2, X_1) \bf{d} X_1
$$

### 预测

$$

$$

### 更新

$$
A_1 = \Sigma_1^{-1/2}, A_2 = \Sigma_2^{-1/2}H, b_1 = -\Sigma_1^{-1/2}\mu_1, b_2 = -\Sigma^{-1/2}_2\mu_2
$$

$$
\Sigma = ()
$$

## 有用的定理

$$
\begin{align}
& \ \ \ \ \ (A_1X+b_1)^T(A_2X+b_2) \\
&= X^T(A_1^TA_1 + A_2^TA_2)X + 2 (b_1^TA_1 + b_2 
 ^TA_2)X + (b^T_1b_1 + b_2^Tb_2) \\
&= (AX+b)^T(AX+b) + const
\end{align}
$$

$$
A = (A_1^TA_1 + A_2^TA_2)^{1/2} \\
b = (A_1^TA_1 + A_2^TA_2)^{-T/2} (A_1^Tb_1 + A_2^Tb_2)
$$


