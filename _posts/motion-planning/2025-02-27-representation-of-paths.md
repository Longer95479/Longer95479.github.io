---
layout: post
title:  "Representation of Paths"
date:   2025-02-27 16:23:00 +0800
tags: 
  - motion planning
categories:
  - motion planning
---

起始配置 和 目标配置之间的连续曲线，称为路径：

$$
y: [0, 1] \rightarrow \mathcal{C}
$$

$q_I = y(0)$ 是起始配置（initial configuration），$q_G = y(1)$ 是目标配置（goal configuration）。configuration 的含义是机器人某一时刻的一种可能的状态，例如机械臂各个关节的角度、速度、角速度等。

有许多类型的函数可以刻画一条曲线，下面介绍在机器人领域中最常用的三种类型。

## 分段线性表示（Piecewise-linear paths）

表示一条曲线的最简单的办法便是 $n$ 条相连的线段。

这需要先给定 $n$ 个路标点（或称为配置）$q_0,q_1,\cdots,q_n$，其中 $q_0 = q_I, q_n = q_G$。可以得到：

$$
y(s) = interp(q_i, q_{i+1}, sn-i),\ i = \lfloor sn \rfloor
$$

相邻两点之间进行线性插值。$i = \lfloor sn \rfloor$ 表示通过 $s$ 来确定线段的序号（线段的序号从 0 开始）。

优点是简单，缺点是存在导数不连续的点，可能导致机器人的急停，难以应用于系统动力学。


## 多项式函数表示（polynomial）

$$
y(s) = poly(\mathbf{c_0}, \mathbf{c_1}, \cdots, \mathbf{c_0};s) = \sum_{i=0}^d \mathbf{c_i}s^i
$$



## 样条函数表示（spline）

将路径表示为 $n$ 条曲线连接而成

$$
y(s) = poly(C_i; sn-i)
$$

$i = \lfloor sn \rfloor$ 是某段曲线的编号，$C_i = [\mathbf{c}_{0,i}, \mathbf{c}_{1,i}, \cdots, \mathbf{c}_{d,i}]$ 是该段多项式曲线的系数，不过我们通常不会存储这些系数，因为手动调整这些参数很难使得曲线连续光滑。一般，我们会存储 *控制点* $\mathbf{p_1}, \mathbf{p_2}, \cdots, \mathbf{p_k}$ ，多项式的系数可以通过控制点推导出来。

### 例子 Cubic Hermite splines

假设某段曲线从 $\mathbf{m_0}$ 跑到了 $\mathbf{m_1}$，并期望满足起始和目标速度分别为 $\mathbf{t_0}$ 和 $\mathbf{t_1}$，即：

$$
\mathbf{m_0} = poly(C_i;0)
$$

$$
\mathbf{m_1} = poly(C_i;1)
$$

$$
\mathbf{t_0} = \frac{\mathrm{d}}{\mathrm{d}u}poly(C_i;s) |_{s = 0}
$$

$$
\mathbf{t_1} = \frac{\mathrm{d}}{\mathrm{d}u}poly(C_i;s) |_{s = 1}
$$

$\mathbf{m_0}, \mathbf{t_0}, \mathbf{m_1}, \mathbf{t_1}$ 就是控制点。

假设分段的多项式曲线的阶数是 3，所以

$$
poly(C_i;s) = c_0 + c_1 s + c_2 s^2 + c_3 s^3 
$$

代入初始点的约束：

$$
\mathbf{m_0} = c_0 + c_1 s + c_2 s^2 + c_3 s^3 |_{s=0} = c_0
$$

$$
\mathbf{t_0} = c_1 + 2 c_2 s + 3 c_3 s^2 |_{s=0} = c_1
$$

带入目标点的约束：

$$
\mathbf{m_1} = c_0 + c_1 s + c_2 s^2 + c_3 s^3 |_{s=1} = c_0 + c_1 + c_2 + c_3
$$

$$
\mathbf{t_1} = c_1 + 2 c_2 s + 3 c_3 s^2 |_{s=1} = c_1 + 2 c_2 + 3 c_3
$$

把以上四个约束等式列在一起，写成矩阵形式：

$$
\begin{bmatrix}
\mathbf{m_0}\\
\mathbf{t_0}\\
\mathbf{m_1}\\
\mathbf{t_1}\\
\end{bmatrix}
= 
\begin{bmatrix}
1 & 0 & 0 & 0\\
0 & 1 & 0 & 0\\
1 & 1 & 1 & 1\\
0 & 1 & 2 & 3\\
\end{bmatrix}

\begin{bmatrix}
\mathbf{c}_0\\
\mathbf{c}_1\\
\mathbf{c}_2\\
\mathbf{c}_3\\
\end{bmatrix}
$$

$$
\begin{bmatrix}
\mathbf{c}_0\\
\mathbf{c}_1\\
\mathbf{c}_2\\
\mathbf{c}_3\\
\end{bmatrix}

= 
\begin{bmatrix}
1 & 0 & 0 & 0\\
0 & 1 & 0 & 0\\
-3 & -2 & 3 & -1\\
2 & 1 & -2 & 1\\
\end{bmatrix}

\begin{bmatrix}
\mathbf{m_0}\\
\mathbf{t_0}\\
\mathbf{m_1}\\
\mathbf{t_1}\\
\end{bmatrix}

$$

所以

$$
\begin{align}

poly(C_i;s) &= 

\begin{bmatrix}
1 & s & s^2 & s^3
\end{bmatrix}

\begin{bmatrix}
\mathbf{c}_0\\
\mathbf{c}_1\\
\mathbf{c}_2\\
\mathbf{c}_3\\
\end{bmatrix}

\\
&=

\begin{bmatrix}
1 & s & s^2 & s^3
\end{bmatrix}

\begin{bmatrix}
1 & 0 & 0 & 0\\
0 & 1 & 0 & 0\\
-3 & -2 & 3 & -1\\
2 & 1 & -2 & 1\\
\end{bmatrix}

\begin{bmatrix}
\mathbf{m_0}\\
\mathbf{t_0}\\
\mathbf{m_1}\\
\mathbf{t_1}\\
\end{bmatrix}

\end{align}
$$

控制点有直观的几何或物理含义，给定这些控制点后，根据最后一个等式，就可以直接得到满足约束的 3 阶段多项式曲线了。

## 参考

[Section III. Motion  - Kris Hauser - University of Illinois at Urbana-Champaign](https://motion.cs.illinois.edu/RoboticSystems/)
