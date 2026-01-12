---
layout: post
title:  "Robust and Efficient Quadrotor Trajectory Generation for Fast Autonomous Flight"
date:   2026-01-05 13:25:00 +0800
tags: 
    - autonomous aerial robot
    - motion planning
categories:
    - paper reading
    - motion planning
---

> [arxiv 1907.01531](https://arxiv.org/pdf/1907.01531)

如果能预先了解 `A*` 算法，那么理解本论文的 `Kinodynamic Path Searching` 会容易许多。`A*` 算法的伪代码如下：

```python
C = dict()
P = PriorityQueue()
P.push(node_start, 0)

while not P.empty():
    node_cur = P.pop()
    C.push(node_cur)
    for node_next in graph.neighbors(node_cur):
        new_cost = graph.cost(node_cur, node_next) + C[node_next]
        if node_next not in C or new_cost < C[node_next]:
            C[node_next] = new_cost
            # fc = gc + hc
            priority = new_cost + heuristic(goal, node_next)
            node_next.parent = node_current
            # pairs that the same node with different priority may exist in P in this way
            P.push(node_next, priority)
```

Primitive Generation

$$
\mathbf{ \dot{x} = Ax + Bu }
$$

$$
\mathbf{
A = 
\begin{bmatrix}
\mathbf{0} & \mathbf{I_3} & \mathbf{0} & \cdots & \mathbf{0} \\
\mathbf{0} & \mathbf{0} & \mathbf{I_3} & \cdots & \mathbf{0} \\
\vdots & \vdots & \vdots & \cdots & \vdots \\
\mathbf{0} & \cdots & \cdots & \mathbf{0} & \mathbf{I_3} \\
\mathbf{0} & \cdots & \cdots & \mathbf{0} & \mathbf{0}
\end{bmatrix}
,
B = 
\begin{bmatrix}
\mathbf{0} \\
\mathbf{0} \\
\vdots \\
\mathbf{0} \\
\mathbf{I_3} \\
\end{bmatrix}
}
$$

$$
\mathbf{ x } (t) = e^{\mathbf{A}t} \mathbf{x}(t) 
+ \int_0^t e^{\mathbf{A}(t-\tau)} \mathbf{Bu}(t-\tau) \mathrm{d}t
$$

## 附录

### 汉密尔顿方程（Hamilton Equation）

> ref: [高级宏观与动态优化：汉密尔顿方程（Hamilton Equation）-知乎](https://zhuanlan.zhihu.com/p/436150206)

$$
\begin{align}
&\max \int_{t_0}^{t_1} f(t, x(t), u(t)) \mathrm{d}t \\
&\mathrm{subject\ to}\ x'(t) = g(t, x(t), u(t)), \\
&t_0 \leq t \leq t_1, x(t_0) = x_0 \ \mathrm{fixed}, x(t_1) \ \mathrm{free} 
\end{align}
$$

求解思路是对最优控制量 $u^*(t)$ 施加扰动，$u(t) = u^*(t) + ah(t)$，再使用上分部积分的技巧。

$$
\int_{t_0}^{t_1} f(t, x(t), u(t)) \mathrm{d}t =
\int_{t_0}^{t_1} f(t, x(t), u(t)) + \lambda(t) g(t, x(t), u(t)) - \lambda(t) x'(t) \mathrm{d}t
$$

利用分部积分可以得到：

$$
\int_{t_0}^{t_1} \lambda(t) x'(t) \mathrm{d}t = x(t_1) \lambda(t_1) - x(t_0) \lambda(t_0) - \int_{t_0}^{t_1} x(t) \lambda'(t) \mathrm{d}t
$$

代入上式，得到

$$
\int_{t_0}^{t_1} (f + \lambda g - \lambda'x) \mathrm{d}t = \int_{t_0}^{t_1} (f + \lambda g + x \lambda') \mathrm{d}t + x(t_0)\lambda(t_0) - x(t_1)\lambda(t_1)
$$

在最优控制中，给定 $u(t)$，通过状态转移方程 $x'(t) = g(t, x(t), u(t))$ 就确定了 $x(t)$。因此问题的核心是确定最优的 $u^*(t)$。

对 $u^*(t)$ 施加扰动，得到 $u(t) = u^*(t) + ah(t)$，相应确定的状态变化记为 $y(t, a)$，并具有以下特性:

$$
y(t, 0) = x^*(t),\ y(t_0, a) = x^*(t_0)
$$

所以 

$$
\begin{align}
J(a) = &\int_{t_0}^{t_1} (f(t, y(t,a), u(t,a)) + \lambda(t) g(t, y(t,a), u(t,a)) + y(t,a) \lambda'(t)) \mathrm{d}t \\
& + y(t_0,a)\lambda(t_0) - y(t_1,a)\lambda(t_1)
\end{align}
$$

$$
\begin{align}
\mathrm{D}J(a) = \left .\frac{d}{da} J(a) \right |_{x = a}
= &\int_{t_0}^{t_1} [(D_x f \ D_a y + \lambda D_xg \  D_a y + \lambda'D_ay) \\
& + (D_u f \ D_a u + \lambda D_u g \  D_a u ) ]
\ \mathrm{d}t \\
& + \lambda(t_0) D_ay(t_0, a) - \lambda(t_1) D_a y(t_1, a)
\end{align}
$$

因为 $y(t_0,a) = x_0$ 是定值，所以 $D_a y(t_0, a) = 0$，所以

$$
\begin{align}
\mathrm{D}J(a) = 
&\int_{t_0}^{t_1} (D_x f + \lambda D_xg + \lambda')D_ay  
+ (D_u f + \lambda D_u g)D_au
\ \mathrm{d}t \\
& - \lambda(t_1) D_a y(t_1, a)
\end{align}
$$

$a = 0$ 时，得到最优结果 $u^*(t)$ 和 $x^*(t)$，所以 $D_aJ(a=0) = 0$

$$
\begin{align}
\mathrm{D}J(a=0) = 
&\int_{t_0}^{t_1} [(D_x f + \lambda D_xg + \lambda') D_ay(t,0) 
+ (D_u f + \lambda D_u g ) h(t)]
\ \mathrm{d}t \\
& - \lambda(t_1) D_a y(t_1, 0)
 = 0
\end{align}
$$

$y(t, a)$ 和 $D_a y$ 的具体表达我们都不知道，可以利用 $\lambda(t)$ 来消除掉，令 $\lambda(t)$：

$$
\underline{
\lambda(t_1) = 0
}
$$

$$
\underline{
\lambda'(t) = - [ D_x f(t, x^*, u^*) + \lambda D_x g(t, x^*, u^*) ]
}
$$

则有：

$$
D J(a=0) = \int_{t_0}^{t_1} (D_u f + \lambda D_u g) h \ \mathrm{d}t = 0 \ \mathrm{for\ any\ } h(t)
$$

如果令 $h(t) = D_u f(t, x^*, u^*) + \lambda D_u g(t, x^*, u^*)$ ，则

$$
D J(a=0) = \int_{t_0}^{t_1} (D_u f + \lambda D_u g)^2  \ \mathrm{d}t = 0
$$

即：

$$
\underline{
D_u f(t, x^*, u^*) + \lambda D_u g(t, x^*, u^*) = 0,\ t_0 \leq t \leq t_1
}
$$

回忆考虑一开始的约束：

$$
\underline{
x^*(t_0) = x_0
}
$$

$$
\underline{
x^{*'}(t) = g(t, x^*(t), u^*(t))
}
$$

换一种表达方式，定义哈密顿函数，求最优控制量：

$$
\boxed{
H(t, x(t), u(t), \lambda(t)) = f(t,x,u) + \lambda(t) g(t,x,u)
}
$$

则对于最优结果 $x^*(t)$ 和 $u^*(t)$ 一定满足（必要条件）：

$$
\boxed{
\begin{align}
D_uH &= 0  = D_u f + \lambda D_u g \\
-D_x H &= \lambda' = -(D_x f + \lambda D_xg) \\
D_{\lambda} H &= x' = g \\
x(t_0) &= x_0, \ \lambda(t_1) = 0 \\
H_{uu}(t, x^*, u^*, \lambda) &\leq 0 \ or \ H_{uu}(t, x^*, u^*, \lambda) \geq 0 \\
\end{align}
}
$$

这里作了一点符号滥用：*行向量的转置等于某个列向量*，简写为 *行向量等于列向量*。

求解思路：利用 $D_u H = 0$ 将 $u$ 用 $x$ 和 $\lambda$ 表示，代入到 $-D_x H = \lambda'$ 和 $D_\lambda H = x'$ 中得到两个关于 $x$ 和 $\lambda$ 的微分方程，再加上两个边界条件 $x(t_0) = x_0$ 和 $\lambda(t_1) = 0$，可以先求解出 $\lambda(t)$, 再求解出 $u(t), x'(t), x(t)$

例子：

$$
\begin{align}
&\max \int_0^1 (x + u) \ \mathrm{d} t \\
&\mathrm{subject\ to }\ x' = 1 - u^2,\ x(0) = 1
\end{align}
$$

解：

$$
H(t, x, u, \lambda) = x + u + \lambda (1 - u^2)
$$

$$
\begin{align}
D_u H &= 1 - 2u\lambda = 0 \\
-D_x H &= -1 = \lambda' \\
D_\lambda H &= 1 - u^2 = x' \\
x(0) &= 1, \lambda(1) = 0 \\
H_{uu} &= -2 \lambda \leq 0
\end{align}
$$

所以

$$
\begin{align}
\lambda(t) &= 1-t \\
u(t) &= \frac{1}{ 2 \lambda(t)} =  \frac{1}{2(1-t)} \\
x'(t) &= 1 - \frac{1}{4(1-t)^2} \\
x(t) &= t - \frac{1}{4(1-t)} + \frac{5}{4}
\end{align}
$$

#### 问题变体

$$
\begin{align}
&\max \int_{t_0}^{t_1} f(t, x(t), u(t)) \ \mathrm{d}t + \phi(x_1) \\
&\mathrm{subject\ to}\ x'(t) = g(t, x, u),\ x(t_0) = x_0, t_0, t_1\ \mathrm{fixed},\ x(t_1) = x_1\ \mathrm{free}.
\end{align}
$$

哈密顿方程的条件不变，但

$$
\lambda(t_1) = 0 \rightarrow \lambda(t_1) = \phi'(x_1)
$$

### 庞特里亚金最小值原理解最优控制问题

> ref: [庞特里亚金最小值原理解最优控制问题-知乎](https://zhuanlan.zhihu.com/p/483647239)

将位置和速度作为状态变量。

$$
x_\mu(t) = 
\begin{bmatrix}
p_\mu(t) \\
    \dot{p}_\mu(t)
\end{bmatrix}
,\ \mu \in \{x,y,z\}
$$

$$
\dot{x}_\mu(t) = A x + B u
$$

$$
A = 
\begin{bmatrix}
0 & 1 \\
0 & 0
\end{bmatrix}
,\ B = 
\begin{bmatrix}
0 \\
1
\end{bmatrix}
$$

优化问题可以描述为

$$
\begin{align}
&\min \frac{1}{T} \int_{0}^{T} p_\mu^{(2)}(t) \ \mathrm{d}t \\
&\mathrm{subject\ to}\ x_\mu(0) = x_{\mu0},\ x_\mu(T) = x_{\mu 1}
\end{align}
$$

解：

重写变量，省略 $\mu$

$$
x_\mu(t) = 
\begin{bmatrix}
p_\mu(t) \\
    \dot{p}_\mu(t)
\end{bmatrix}
=
\begin{bmatrix}
p(t) \\
v(t) \\
\end{bmatrix}
,\ u(t) = p^{(2)}(t) = a(t)
$$

写出哈密顿函数：

$$
\begin{align}
H(t,x,u, \lambda) &= \frac{1}{T}u^2 + \lambda^T \dot{x} \\
&= \frac1T a^2 + \lambda_1 v + \lambda_2 a
\end{align}
$$

满足以下条件

$$
D_u H = \frac2T a + \lambda_2 = 0
$$

$$
-D_xH = 
\begin{bmatrix}
0 \\
-\lambda_1
\end{bmatrix}
=
\begin{bmatrix}
\dot{\lambda}_1 \\
\dot{\lambda}_2
\end{bmatrix}
$$

$$
D_{\lambda} H = 
\begin{bmatrix}
v \\
a
\end{bmatrix}
=
\begin{bmatrix}
v \\
a
\end{bmatrix}
$$

得到：

$$
\begin{bmatrix}
0 \\
-\lambda_1
\end{bmatrix}
=
\begin{bmatrix}
\dot{\lambda}_1 \\
\dot{\lambda}_2
\end{bmatrix}

\Rightarrow

\lambda(t) = 
\frac2T
\begin{bmatrix}
\alpha \\
- \alpha t - \beta
\end{bmatrix}
$$

$$
\begin{align}
 \frac2T a + \lambda_2 = 0 &\Rightarrow a = \alpha t + \beta \\
v = \dot{a} &\Rightarrow \boxed{ v = \frac12\alpha t^2 + \beta t + v_0} \\
p = \dot{v} &\Rightarrow \boxed{ p = \frac16\alpha t^3 + \frac12 \beta t^2 + v_0t + p_0}
\end{align}
$$

令

$$
\boxed{
\begin{align}
\Delta p &= p_1 - p_0 - v_0 T \\
\Delta v &= v_1 - v_0
\end{align}
}
$$

则方程组可以写成，即起点终点值约束

$$
\begin{bmatrix}
\Delta p\\
\Delta v\\
\end{bmatrix}
=
\begin{bmatrix}
\frac{1}{6}T^3 & \frac{1}{2}T^2 \\
\frac{1}{2}T^2 & T
\end{bmatrix}
\begin{bmatrix}
\alpha \\
\beta
\end{bmatrix}

\Rightarrow


\boxed{
\begin{bmatrix}
\alpha \\
\beta
\end{bmatrix}
=
\frac{1}{T^3}
\begin{bmatrix}
-12 & 6T \\
6T & -2T^2
\end{bmatrix}
\begin{bmatrix}
\Delta p\\
\Delta v\\
\end{bmatrix}
}
$$

代入损失函数

$$
J(T) = \frac1T \int_0^T a^2 \ \mathrm{d}t 
= \frac13 \alpha^2T^3 + \alpha \beta T^2 + \beta^2 T
$$

$$
D_T J = 0\ 可求出最优\ T
$$

### 贝塞尔曲线

> ref: [贝塞尔曲线](https://zhuanlan.zhihu.com/p/442527216)

定义阶数为n的贝塞尔曲线，我们需要在空间中选择n+1个控制点，以便它们大致确定所需曲线的形状。

移动一个或多个控制点时，Bézier曲线的形状会相应地改变。但是，曲线始终位于由控制点定义的凸包中（凸包特性），并且生成的曲线的形状没有控制多边形复杂（变化递减特性）。

具体公式如下：

$$
C(u) = \sum_{i = 0}^p B_{p, i}(u) P_i
$$

$$
B_{p. i} = \frac{p!}{i!(p-i!)} u^i (1-u)^{p-i}
$$

其中 $B_{p, i}$ 是贝塞尔曲线的基函数，$P_i$ 是控制点，可以理解为约束，决定了曲线的形状。$p$ 表示贝塞尔曲线的阶数，$i \in [0, n]$ 是控制点的下标。

#### 求曲线上某点（Evaluate）

给定参数 $u$，代入定义公式可以求得对应点的值，但不是最优算法。

实际应用中可以采用 De Casteljau's 算法迭代地求曲线上的一点 $C(u)$

$$
P_{l, i} = (1 -u) P_{l-1, i} + u P_{l-1, j+1} 
$$

$l$ 表示迭代的层数，$i$ 表示某一层的控制点的序号。每一轮/层计算出来的控制点数量会递减 1。最后一层的控制点个数只有一个，此点就是曲线在 $u$ 处的点 $C(u)$。

#### 特性

1. 凸包特性-Convex Hull Property
    - 贝塞尔曲线始终位于其控制点所围成的凸包当中。凸包是包含给定点集的最小凸多边形，仿佛橡皮筋圈住这堆点
2. 变异递减性-Variation Diminishing Property
    - 一条直线穿过凸包，与贝塞尔曲线的交点数不超过与控制多边形的交点数，说明贝塞尔曲线比其控制多边形更简单
3. 全局性-移动一个控制点影响整个曲线
    - 所以控制点越多越难以使曲线达到想要的效果，因为任意移动控制点整个曲线都会发生改变
4. 仿射不变性-Affine Invariance
    - 如果对Bézier曲线应用仿射变换，只需要对其控制点进行仿射变换，使用变换后的控制点再构造曲线就可以了

### B 样条曲线

$$
C(u) = \sum_{i = 0}^N B_{p,i}(u) P_i
$$

其中，$P_i, i \in [0,N]$ 为 B 样条曲线的控制点，$N$ 为控制点个数，$B_{p,i}(u)$，$p$ 为基函数的阶数。以上公式类似于贝塞尔曲线的定义，但不同在于基函数的构造。

$N_{n,i}$ 是 B 样条曲线的基函数，其定义依赖于 *节向量（Knot vector）*，节向量定义如下。节（Knot）把参数定义域 $u \in [0, 1]$ 分成多个节点段。

$$
U = [u_0, u_1, \cdots, u_M]
$$

节向量对应到曲线上的点 $C(u_i)$ 称为 *节点（Knot Point）*。节点仿佛是曲线上打的绳节，将曲线分成多个 *曲线段（Curve Segment）*，每个曲线段实际上是定义在 *节点段（Knot Span）* 上的 n 阶贝塞尔曲线。

B 样条曲线的 *基函数* 的定义依赖于 *节向量*

$$
B_{0,i} (u)= 
\left \{
\begin{align}
& 1,\ u_i \leq u < u_{i+1} \\
& 0,\ otherwise
\end{align}
\right .
$$

$$
B_{k, i}(u) = \frac{u - u_i}{u_{i+k} - u_i} B_{k-1, i}(u) + \frac{u_{i+k+1} - u}{u_{i+k+1} - u_{i+1}} B_{k-1, i+1}(u)
$$


