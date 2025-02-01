---
layout: post
title:  "Equivalence between Kalman Filter and Factor graph"
date:   2025-01-26 17:22:00 +0800
tags: 
  - Quaternion
categories:
  - math
---

本文将因子图优化和卡尔曼滤波联系起来：卡尔曼滤波等价于因子图优化在线性系统下只优化最新状态。

我们最根本的目标，是在给定测量值 $z$，找出使得联合概率密度函数 $p(x, z)$ 最大的 $x$。而 $p(x, z)$ 可以进行分解，以便我们写出具体的形式：

$$
p(x,z) = p(z|x)p(x)
$$

假如我们考虑两个状态 $x_1, x_2$，以及一个测量 $z$（对 $x_2$ 的测量）,则联合概率密度函数可以写为：

$$
p(x_2, x_1, z) = p(z | x_2) p(x_2 | x_1) p(x_1)
$$

可以看到，联合概率密度函数被分解成了三个概率密度函数的乘积，而这正好对应了卡尔曼滤波里的三个阶段：先验、预测、更新。

- 先验（prior）：$x_1$ 的先验分布，据此可以求出 $x_1$ 的最优值及其协方差。

$$
x_1^* = \underset{x_1}{\mathrm{arg\ min}}\  p(x_1)
$$

- 预测（prediction）：从 $x_1$ 和 $x_2$ 的联合概率密度函数得到 $x_2$ 单独的概率密度函数，并求 $x_2$ 的最优值，绕过对 $x_1$ 的优化的同时又利用了 $x_1$ 的先验信息。

$$
x_2^- = \underset{x_2}{\mathrm{arg\ min}}\  p(x_2)
$$

$$
\begin{align}
p(x_2) &= \int p(x_2 , x_1) \mathrm{d} x_1 \\
&= \int p(x_2 | x_1) p(x_1) \mathrm{d} x_1 
\end{align}
$$

- 更新（update）：在利用了 $x_1$ 的先验信息基础上，利用对 $x_2$ 的直接测量信息。

$$
x_2^* = \underset{x_2}{\mathrm{arg\ min}}\ p(x_2|z_2)
$$

$$
p(x_2 | z_2) \propto p(x_2, z_2) = p(z_2 | x_2) p(x_2)
$$

线性系统的预测和测量模型如下：

$$
x_{k+1} = Fx_k + Bu_k + w
$$

$$
z_k = Hx_k + v
$$

$$
\ w \sim N(0, Q),\ \ v \sim N(0, R)
$$

接下来写出先验、预测、更新的概率密度函数的具体形式：

- 先验

$$
p(x_1) \propto \exp({-\frac12 ||x_1 - \mu_1||^2_{\Sigma_1}})
$$

- 预测

$$
p(x_2 | x_1)p(x_1) \propto \exp ({-\frac12 ||x_2 - (Fx_1 + Bu)||^2_Q}) \cdot
\exp (-\frac12 ||x_1 - \mu_1||^2_{\Sigma_1})
$$

- 更新

$$
p(z_2 | x_2)p(x_2) \propto \exp (-\frac12 ||Hx_2 - z_2||^2_R) \cdot \int p(x_2, x_1) \mathrm{d} x_1
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
Q+F\Sigma_1 F^T & -F \Sigma_1 \\
-\Sigma_1 F^T & \Sigma_1
\end{bmatrix}

}

\end{align}
$$

此时，可以很方便地把 $x_1$ 边缘化，只需要把对应部分的均值和方差抽离出来即可：

$$
X_2 \sim N(F\mu_1 + Bu, F\Sigma_1F^T + Q)
$$

$$
\boxed{ \mu_1 \leftarrow F\mu_1 + Bu }
$$

$$
\boxed{\Sigma_1 \leftarrow F\Sigma_1F^T + Q}
$$

其中，$A^{-1}b$，$(A^T \Sigma^{-1} A)^{-1}$ 计算如下：

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
Q+F\Sigma_1 F^T & -F \Sigma_1 \\
-\Sigma_1 F^T & \Sigma_1
\end{bmatrix}

\end{align}
$$



### 更新

令

$$
\Sigma_2 = R
$$

只考虑指数部分

$$
\begin{align}
& (x_2- \mu_1)^T \Sigma_1^{-1} (x_2 - \mu_1) + (Hx_2 - z_2)^T\Sigma_2^{-1}(Hx_2 - z) \\
&=
\left\| \Sigma^{-1/2}_1 x_2 - \Sigma_1^{-1/2}\mu_1 \right\|^2 + 
\left\| \Sigma_2^{-1/2}Hx_2 - \Sigma_2^{-1/2}z \right\|^2 
\\


&\ \  \downarrow
\scriptsize{
A_1 = \Sigma_1^{-1/2},\  A_2 = \Sigma_2^{-1/2}H,\  b_1 = -\Sigma_1^{-1/2}\mu_1,\  b_2 = -\Sigma^{-1/2}_2 z
} \\

&= (A_1x+b_1)^T(A_1x+b_1) + (A_2x+b_2)^T(A_2x+b_2) \\

&\ \ \downarrow
\scriptsize {
A = (A_1^TA_1 + A_2^TA_2)^{1/2},\ 
b = (A_1^TA_1 + A_2^TA_2)^{-T/2} (A_1^Tb_1 + A_2^Tb_2)
}
\\

&= 
(Ax+b)^T(Ax+b) + \mathrm{const} \\
&=
(x+A^{-1}b)^TA^TA(x+A^{-1}b) + \mathrm{const} \\

&\ \  \downarrow \scriptsize{\Sigma = (A^TA)^{-1},\ \mu = -\Sigma^{-1/2}b} \\ 

&= 
(x - \mu) \Sigma^{-1}(x - \mu) + \mathrm{const}
\end{align}
$$

所以

$$
\Sigma = (\Sigma_1^{-1} + H^T\Sigma_2^{-1}H)^{-1},\  
\mu = (\Sigma_1^{-1} + H^T\Sigma_2^{-1}H)^{-1}(\Sigma_1^{-1} + H^T\Sigma_2^{-1}z_2) 
$$

展开 $\Sigma$：

$$
\begin{align}
\Sigma &= (\Sigma_1^{-1} + H^T\Sigma_2^{-1}H)^{-1} \\

&\ \ \downarrow
\scriptsize{
  (A-BD^{-1}C)^{-1} = A^{-1} + A^{-1}B(D-CA^{-1}B)^{-1}CA^{-1}
}\\

&= 
\Sigma_1 + \Sigma_1 H^T(-\Sigma_2^{-1} - H \Sigma_1 H^T)^{-1}H\Sigma_1 \\

&= [I - \Sigma_1 H^T(\Sigma_2 + H \Sigma_1 H^T)^{-1}H] \Sigma_1 \\

&\ \ \downarrow
\scriptsize{
  G = \Sigma_1 H^T(\Sigma_2 + H \Sigma_1 H^T)^{-1}
} \\

& \boxed{\Sigma = [I - GH]\Sigma_1}

\end{align}
$$

展开 $\mu$ :

$$
\begin{align}
\mu &= (I - GH)\Sigma_1(\Sigma_1^{-1}\mu_1 + H^T\Sigma_2^{-1}z_2) \\
&= (I - GH)(\mu_1 + \Sigma_1 H^T \Sigma_2^{-1} z_2) \\
&= (I-GH)\mu_1 + (I-GH) \Sigma_1 H^T \Sigma_2^{-1} z_2\\

&\ \ \downarrow
\scriptsize{
   (I-GH) \Sigma_1 H^T \Sigma_2^{-1} = G
}\\

&= (I-GH)\mu_1 +  Gz_2\\

& \boxed{\mu = \mu_1 + G(z_2 - H\mu_1)}

\end{align}
$$


## 附录

### 1

$$
\begin{align}
& \ \ \ \ \ (A_1x+b_1)^T(A_2x+b_2) \\
&= x^T(A_1^TA_1 + A_2^TA_2)x + 2 (b_1^TA_1 + b_2 
 ^TA_2)x + (b^T_1b_1 + b_2^Tb_2) \\
 
&\ \ \ \ \scriptsize {
  \downarrow
A = (A_1^TA_1 + A_2^TA_2)^{1/2},\ 
b = (A_1^TA_1 + A_2^TA_2)^{-T/2} (A_1^Tb_1 + A_2^Tb_2)
}
\\

&= x^TA^TAx + 2b^TAx + \mathrm{const}\\
&= (Ax+b)^T(Ax+b) + \mathrm{const} \\

&\ \ \ \ \scriptsize {
  \downarrow
\Sigma = (A^TA)^{-1},\ \mu = -A^{-1}b
} \\

&= (x - \mu)^T \Sigma^{-1} (x-\mu) + \mathrm{const}
\end{align}
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

### 3

$$
(A-BD^{-1}C)^{-1} = A^{-1} + A^{-1}B(D-CA^{-1}B)^{-1}CA^{-1}
$$

可使用舒尔补辅助证明。

### 4

$$
(I-GH) \Sigma_1 H^T \Sigma_2^{-1} = G
$$

可使用反证法证明。