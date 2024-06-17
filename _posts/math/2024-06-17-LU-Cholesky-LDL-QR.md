---
layout: post
title:  "LU, Cholesky, LDL and QR Factorization"
date:   2024-06-17 17:21:00 +0800
tags: 
  - linear algebra
categories:
  - math
---

## LU

LU 分解（LU Factorization）其实是高斯消元法的矩阵形式。

消去 $A_{m\times n}$ 的第一列主元以下的元素：

$$
L_1 = 
\begin{pmatrix}
1 & 0 & 0 & \cdots & 0 \\
-\frac{a_{21}}{a_{11}} & 1 & 0 & \cdots & 0 \\
-\frac{a_{31}}{a_{11}} & 0 & 1 & \cdots & 0 \\
\vdots &&& \ddots & \vdots \\
-\frac{a_{m1}}{a_{11}} & 0 & 0 & \cdots & 1
\end{pmatrix}_{m\times m}
$$

$$
L_1 A=
\begin{pmatrix}
a_{11} & a_{12} & a_{13} & \cdots & a_{1n} \\
0 & a_{22}^{(1)}  & * & \cdots & * \\
0 & * & * & \cdots & * \\
\vdots & & &\ddots & \vdots \\
0 & * & * & \cdots & *
\end{pmatrix}
$$

$$
a_{22}^{(1)} = a_{21} - \frac{a_{21}}{a_{11}}a_{12}
$$

在此基础上消去第二列主元以下的元素：

$$

L_2 = 
\begin{pmatrix}
1 & 0 & 0 & 0 & \cdots & 0 \\
0 & 1 & 0 & 0 & \cdots & 0 \\
0 & -\frac{a_{32}^{(1)}}{a_{22}^{(1)}} & 1 & 0 & \cdots & 0 \\
0 & -\frac{a_{42}^{(1)}}{a_{22}^{(1)}} & 0 & 1 & \cdots & 0 \\
0 & -\frac{a_{52}^{(1)}}{a_{22}^{(1)}} & 0 & 0 & \cdots & 0 \\
\vdots &&&& \ddots & \vdots \\
0 & -\frac{a_{m2}^{(1)}}{a_{22}^{(1)}} & 0 & 0 & \cdots & 1
\end{pmatrix}
$$

$$
L_2 L_1 A = 
\begin{pmatrix}
* & * & * & \cdots & * \\
0 & * & * & \cdots & * \\
0 & 0 & * & \cdots & * \\
\vdots &&& \ddots & \vdots \\
0 & 0 & * & \cdots & *
\end{pmatrix}
$$

以此类推

$$
L_n L_{n-1}\cdots L_1 A = 
\begin{pmatrix}
* & * & * & \cdots & * & * \\
0 & * & * & \cdots & * & * \\
0 & 0 & * & \cdots & * & * \\
0 & 0 & 0 & \cdots & * & * \\
\vdots &&&& \ddots & \vdots \\
0 & 0 & 0 & \cdots & 0 & *
\end{pmatrix}
= U
$$

$L_n L_{n-1}\cdots L_1$ 等于把各个 $L_i$ 的主元下面的非零元素全部填充在一起

$$
A = LU,L=(L_n L_{n-1}\cdots L_1)^{-1}
$$

