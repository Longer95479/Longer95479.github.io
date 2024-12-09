---
layout: post
title:  "state estimate back end"
date:   2024-11-23 15:15:00 +0800
tags: 
  - Quaternion
categories:
  - math
---

状态估计的本质是最大后验估计，一步步转化到标量函数的和，最后线性化迭代求解。

$$
\begin{align}
P(X|Z) &\propto P(X|Z)P(Z) \\
&= P(Z|X)P(X) \\
&= P(X) \prod P(z_{i}|x_{\{j\}}) 
\end{align}
$$

$$
\begin{align}
- \log [P(X) \prod P(z_{i}|X_{\{j\}})] 
&= \sum h^T(x_{\{j\}}; z_{i})h(x_{\{j\}}; z_{i}) \\
&\approx \sum (h_0 + J^h_x x)^T(h_0 + J^h_x x)
\end{align}
$$

