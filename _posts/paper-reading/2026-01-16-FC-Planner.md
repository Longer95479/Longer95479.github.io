---
layout: post
title:  "FC-Planner: A Skeleton-guided Planning Framework for Fast Aerial Coverage of Complex 3D Scenes"
date:   2026-01-16 23:20:00 +0800
tags: 
    - autonomous aerial robot
    - motion planning
categories:
    - paper reading
    - motion planning
---

### Skeleton Extraction

该方法参考自 [Tagliasacchi et al. 2009](https://dl.acm.org/doi/epdf/10.1145/1531326.1531377)

使用 ROSA( generalized rotational symmetry axis) Points 来表达点云骨架。

$$
r_p = (\mathbf{x}_p, \mathbf{v}_p), p \in \mathcal{P}_D
$$

用于计算 ROSA 的点云 $\mathcal{P}_D$ 下采样自 *输入点云* $\mathcal{P}_O$

求解方法：

1. 确定锚点以及最佳切割平面
2. 确定对应的邻域，依赖于定义的度量，参考自 [Lehtinen et al. 2008](https://dl.acm.org/doi/epdf/10.1145/1360612.1360636)

$$
\mathrm{dist}({r_p}_i,{r_p}_j) = \left\| 
{\mathbf{x}_p}_i - {\mathbf{x}_p}_j + F_{squash} 
\left< {\mathbf{x}_p}_i -  {\mathbf{x}_p}_j \cdot {\mathbf{v}_p}_j \right> 
{\mathbf{v}_p}_j 
\right\|
$$


3. 计算 $\mathbf{v}_p$，通过最小化该向量与邻域内的法向量角度方差

$$
\mathbf{v}_p^{i+1} = \mathop{\mathrm{argmin}}\limits_{\mathbf{v}_p \in \mathbb{R},\left\| \mathbf{v}_p  \right\|_2 = 1} {\mathbf{v}_p^i}^T \Sigma_p^i \mathbf{v}_p^i
$$

4. 计算 $\mathbf{x}_p$，通过最小化该点到邻域内点的



