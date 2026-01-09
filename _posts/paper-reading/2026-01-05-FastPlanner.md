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

优化问题可以描述为

$$
\begin{align}
&\min \frac{1}{T} \int_{0}^{T} p_\mu^{(2)}(t) \ \mathrm{d}t \\
&\mathrm{subject\ to}\ x_\mu(0) = x_c,\ x_\mu(T) = x_g
\end{align}
$$

解：

重写变量

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

$$
H(t,x,u) = \frac{1}{T}u^2 + \lambda^T \dot{x}
$$



