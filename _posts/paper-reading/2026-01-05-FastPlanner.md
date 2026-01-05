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
\mathbf{ x } (t) = e^{\mathbf{A}t} \mathbf{x}(t) 
+ \int_0^t e^{\mathbf{A}(t-\tau)} \mathbf{Bu}(t-\tau) \mathrm{d}t
$$

