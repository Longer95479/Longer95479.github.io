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
\int_{t_0}^{t_1} f(t, x(t), u(t)) \mathrm{d}t = \\
\int_{t_0}^{t_1} f(t, x(t), u(t)) + \lambda(t) x'(t) - \lambda(t) g(t, x(t), u(t)) \mathrm{d}t
$$



### 庞特里亚金最小值原理解最优控制问题

> ref: [庞特里亚金最小值原理解最优控制问题-知乎](https://zhuanlan.zhihu.com/p/483647239)



