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

为了求极值对应的自变量，我们只需考虑指数部分：

$$
\begin{align}
& ||x_2 - (Fx_1 + Bu)||^2_Q + ||x_1 - \mu_1||^2_{\Sigma_1} \\
&= 
\begin{bmatrix}
Q^{-1/2}(x_1 - Fx_1 - Bu) \\
\Sigma^{-1/2} (x_1 -\mu)
\end{bmatrix} ^T

\begin{bmatrix}
Q^{-1/2}(x_1 - Fx_1 - Bu) \\
\Sigma^{-1/2} (x_1 -\mu)
\end{bmatrix} \\

&= 
\left\|
\begin{bmatrix}
Q^{-1/2} & \bf{0} \\
\bf{0} & \Sigma_1^{-1/2}
\end{bmatrix}
 (
\begin{bmatrix}
I & -F \\
\bf{0} & I
\end{bmatrix}

\begin{bmatrix}
x_2 \\
x_1
\end{bmatrix}
+
\begin{bmatrix}
-Bu \\
- \mu_1
\end{bmatrix}
 )
\right\|^2
\\

&=

\left\|
\begin{bmatrix}
I & -F \\
\bf{0} & I
\end{bmatrix}

\begin{bmatrix}
x_2 \\
x_1 
\end{bmatrix}
+
\begin{bmatrix}
-Bu \\
-\mu_1
\end{bmatrix}
\right\|
^2
_{
\begin{bmatrix}
Q & \bf{0} \\
\bf{0} & \Sigma_1
\end{bmatrix}
} \\

&=

\left\|
Ax + b
\right\|_{\Sigma}
\\
&=

\left\|
x + A^{-1}b
\right\| _ {(A^T \Sigma^{-1} A)^{-1}}

\\
&= 

\left\|
\begin{bmatrix}
x_2 \\
x_1
\end{bmatrix}

+

\begin{bmatrix}
-Bu - F \mu_1 \\
- \mu _1
\end{bmatrix}

\right\|
_{
\begin{bmatrix}
Q+F\Sigma_1 F^T & -F \Sigma \\
-\Sigma_1 F^T & \Sigma_1
\end{bmatrix}

}

\end{align}
$$

此时，可以很方便地把 $x_1$ 边缘化：

$$
X_2 N(Bu+F\mu_1, Q+F\Sigma_1F^T)
$$


$A^{-1}b$，$(A^T \Sigma^{-1} A)^{-1}$ 计算如下：

$$
\begin{align}
A^{-1}b &= 

\begin{bmatrix}
I & -F \\
\bf 0 & I
\end{bmatrix}
^{-1}
\begin{bmatrix}
-Bu \\
- \mu_1
\end{bmatrix}

\\
&= 

\begin{bmatrix}
I & F \\
\bf 0 & I
\end{bmatrix}
\begin{bmatrix}
-Bu \\
- \mu_1
\end{bmatrix}

\\
&= 

\begin{bmatrix}
-Bu - F \mu_1 \\
- \mu _1
\end{bmatrix}

\end{align}
$$

$$
\begin{align}
(A^T \Sigma^{-1} A)^{-1} &= A^{-1} \Sigma A^{-T} \\

&= 

\begin{bmatrix}
I & -F \\
\bf 0 & I 
\end{bmatrix}

\begin{bmatrix}
Q & \bf 0 \\
\bf 0 & \Sigma_1 
\end{bmatrix}

\begin{bmatrix}
I & \bf 0 \\
-F^T & I
\end{bmatrix}
\\

&=

\begin{bmatrix}
Q+F\Sigma_1 F^T & -F \Sigma \\
-\Sigma_1 F^T & \Sigma_1
\end{bmatrix}

\end{align}
$$



### 更新

$$
A_1 = \Sigma_1^{-1/2}, A_2 = \Sigma_2^{-1/2}H, b_1 = -\Sigma_1^{-1/2}\mu_1, b_2 = -\Sigma^{-1/2}_2\mu_2
$$

$$
\Sigma = ()
$$

## 有用的定理

### 1

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

### 2

$$
\begin{bmatrix}
I & F \\
\bf0 & I \\
\end{bmatrix}
^{-1}
= 
\begin{bmatrix}
I & -F \\
\bf0 & I \\
\end{bmatrix}
$$

$$
\begin{bmatrix}
I & \bf0 \\
F & I \\
\end{bmatrix}
^{-1}
= 
\begin{bmatrix}
I & \bf0 \\
-F & I \\
\end{bmatrix}
$$
