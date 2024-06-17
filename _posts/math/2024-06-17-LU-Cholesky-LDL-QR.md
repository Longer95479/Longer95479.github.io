---
layout: post
title:  "LU, Cholesky, LDL and QR Factorization"
date:   2024-06-17 17:21:00 +0800
tags: 
  - linear algebra
categories:
  - math
---

> 除以一个很小的数，是影响数值稳定的一个重要来源

对于满秩的矩阵 $J_{m\times n},m \geq n$,齐次超定方程 $Jx = 0$ 可以用 SVD 分解来求取最小二乘解（[解最小奇异值对应的右奇异向量](/_posts/math/2024-06-13-SVD.md)）。

而对于非齐次的超定方程 $Jx = b$，则可以转化为求解

$$
J^TJx = J^Tb
$$

## LU Factorization

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
\vdots &&& \ddots&& \vdots \\
0 & 0 & 0 & \cdots & 0 & * \\
0 & 0 & 0 & \cdots & 0 & 0 \\
\end{pmatrix}
= U
$$

$L_n L_{n-1}\cdots L_1$ 等于把各个 $L_i$ 的主元下面的非零元素全部填充在一起

最后得到 LU 分解

$$
A_{m \times n} = L_{m\times m}U_{m\times n}\\
L=(L_n L_{n-1}\cdots L_1)^{-1}
$$

有时，我们想让 $U$ 的对角线也都是1

$$
A = LU = LDU'
$$

$D$ 是对角矩阵

## Cholesky Factorization

对于实对称矩正定阵 $\Lambda$（real Symmetric positive definite matrix，SPD）

$$
\begin{align}
\Lambda = LU &= LDU' \\
& \scriptsize{利用对称的条件} \\
&= LDL^T \\
& \scriptsize{D 是对角矩阵} \\
&= L\sqrt{D} (\sqrt{D})^TL^T \\
\Lambda & = R^TR
\end{align}
$$

在实际中，如果矩阵是正定的，使用`Cholesky分解`会比`LU分解`更加高效，更加数值稳定。

## LDL Factorization

实对称矩正定阵 $\Lambda$（real Symmetric positive definite matrix，SPD）

$$
\Lambda = LU = LDU' = LDL^T
$$

相较于 Cholesky 分解，LDL 分解能够避免计算开方。

## QR Factorization



## reference

- [Chapter 3 QR 分解 | 数值分析笔记](https://o-o-sudo.github.io/numerical-methods/qr-.html)
