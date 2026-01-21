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

## 1 Skeleton-based Space Decomposition

### Skeleton Extraction

该方法参考自 [Tagliasacchi et al. 2009](https://dl.acm.org/doi/epdf/10.1145/1531326.1531377)

使用 ROSA( generalized rotational symmetry axis) Points 来表达点云骨架。

$$
r_p = (\mathbf{x}_p, \mathbf{v}_p), p \in \mathcal{P}_D
$$

用于计算 ROSA 的点云 $\mathcal{P}_D$ 下采样自 *输入点云* $\mathcal{P}_O$

求解方法：

1. 确定锚点以及最佳切割平面
2. 确定对应的邻域 $\mathcal{N}_p$，依赖于定义的距离，参考自 [Lehtinen et al. 2008](https://dl.acm.org/doi/epdf/10.1145/1360612.1360636)。

$$
\mathrm{dist}({r_p}_i,{r_p}_j) = \left\| 
{\mathbf{x}_p}_i - {\mathbf{x}_p}_j + F_{squash} 
\left< {\mathbf{x}_p}_i -  {\mathbf{x}_p}_j, {\mathbf{v}_p}_j \right>
{\mathbf{v}_p}_j 
\right\|
$$

> 这个距离定义可以让 ${r_p}_j$ 周围的等值面（iso-surface）是一个椭球，且该椭球跟 ${\mathbf{v}_p}_j$ 对齐的轴会比其他两个轴短 $1/(1 + F_{squash})$。这意味着当 ${\mathbf{x}_p}_i$ 偏离于和 ${\mathbf{x}_p}_j$ 相切的平面时，距离会增长得更快。我们使用 $F_{squash} = 2$，这是一种马氏距离。

> ![iso-surface](../../assets/2026-01-16-FC-Planner/iso-surface.png)

3. 计算 $\mathbf{v}_p$，通过最小化该向量与邻域内的法向量角度方差


$$
\mathbf{v}_p^{i+1} = \mathop{\mathrm{argmin}}\limits_{\mathbf{v}_p \in \mathbb{R}^3,\left\| \mathbf{v}_p  \right\|_2 = 1} 
\mathrm{var}\left\{ \left< \mathbf{v}_p^i, \mathbf{n}(p_k) \right> \ : \ p_k \in \mathcal{N}_p^{(i)} \right\}
$$

$$
\mathbf{v}_p^{i+1} = \mathop{\mathrm{argmin}}\limits_{\mathbf{v}_p \in \mathbb{R}^3,\left\| \mathbf{v}_p  \right\|_2 = 1} 
{\mathbf{v}_p^i}^T \Sigma_p^i \mathbf{v}_p^i
$$

4. 计算 $\mathbf{x}_p$，相当于最小化该点到邻域内点的沿着法向延长的直线的距离平方和

$$
\mathbf{x}_p = 
\mathop{\mathrm{argmin}}\limits_{\mathbf{x}_p \in \mathbb{R}^3} \sum_{p_k \in \mathcal{N}_p} \left\| (\mathbf{x}_p - p_k) \times \mathbf{n}(p_k) \right\|^2
$$


### Skelekon Decomposition

度数大于 2 的节点称为 joint；度数等于 1 的节点称为 leaf。

branch 终止于 joint 或 leaf。

1. DFS 深度优先从每个 joint 出发，得到一个个 branch 
2. 对第一步得到的 branch 作进一步的细分，为的是让单个 branch 的几何足够简单

### Space Allocation 

论文中的方式是根据计算出来的 ROSA points，或者按原文的描述是对边进行离散化，得到一堆有向点，确定出一个个平面，判断那些点云属于这个平面（根据点到平面的距离来衡量），然后点云就被归属到 ROSA points 对应的分支（branch）里，组成一个子空间（subspace）。

这是一个可以改进的点，该计算过程其实可以被省略，因为在计算 ROSA point 的时候，点云和 ROSA point 的对应关系就已产生了。

## 2 Skeleton-guided Viewpoint Generation

### Internal Space and Viewpoint Sampling Ray

从骨架中的 ROSA 点 或 对边离散化得到的有向点 为起点，射向分配到该平面上的点，该方向作为 ray casting 的方向，对体素进行遍历。为了加速，也可以从边界另一端双向进行光线投射。该过程将栅格分成三类，靠近有向点的标记为内部，有点云的栅格标记为占据，继续向外的标记为空状态。

### Iterative Updates of Viewpoint Pose

从占据栅格出发，射出的射线称为 *视角采样射线（viewpoint sampling ray）* $r_{vs}$，起点和方向定义为
$$
\mathbf{sr} = [x_{sr}, y_{sr}, z_{sr}],
\mathbf{dr} = [nx_{dr}, ny_{dr}, nz_{dr}]
$$ 

定义视野为 5-DOF，表示为

$$
\mathbf{vp} = [\mathbf{p}, \theta, \phi, id]
$$

$\mathbf{p}$ 是视野或相机等传感器的位置，$\theta,\phi$ 是 pitch 和 yaw 的角度。

$$
\mathbf{p} = \mathbf{sr} + D \mathbf{dr}
$$

$$
\theta = \arcsin (-nz_{dr}/\left\| \mathbf{dr}  \right\|)
$$

$$
\phi = \arctan (-ny_{dr}/-nx_{dr})
$$

简单来说，视角位于物体表面的 $D$ 距离处，朝向是射线的反向，也就是看向物体表面。

这些视角称作初始视角，其集合表示为 $\mathcal{VP}_{ini}$。

1. 通过双向光线投射 BiRC 确定被视野 $\mathcal{VP}_{ini}$ 覆盖到的栅格，如果一个栅格被多个视角覆盖到，则被配对到保留覆盖数更多的视角。最后如果一个视角没有被分配给任何一个体素，从集合中移除其余视角
2. 合并视角，文中称之为 *gravitation-like model*，会从 $\mathcal{VP}$ 中构建 kd 树 $T_{ini}$，从覆盖数最多的视角开始，直到覆盖数最小的视角，查找给定半径内的覆盖数小于当前视角的活跃视角，更新被查询视角位置后，将活跃视角休眠
3. 重复步骤 1 BiRC 判断未被覆盖到的体素，并生成视角集合 $\mathcal{VP}_{unc}$ 以提高覆盖率
4. 对 $\mathcal{VP}_{unc}$ 执行以上步骤，可多次迭代

最后活跃的视角的 id 会分配给对应的子空间，因此按子空间可以对最后活跃视角集合进行划分。

$$
\mathcal{V} = \{ \mathcal{VP}_1,\cdots, \mathcal{VP}_N \}
$$

