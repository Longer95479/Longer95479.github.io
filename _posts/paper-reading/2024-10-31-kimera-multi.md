---
layout: post
title:  "[Paper Reading] Kimera-Multi: Robust, Distributed, Dense
Metric-Semantic SLAM for Multi-Robot Systems"
date:   2024-10-31 23:46:00 +0800
tags: 
    - slam
categories:
    - paper reading
---

I think this paper has three features: 

- Metric-Semantic mesh map
- Distributed loop closure detection
- Robust distributed trajectory estimation

**Distributed loop closure detection** 
- a --global descriptor--> b
- a <--request 3D keypoints and descriptors-- b
- a -- send 3D keypoints and descriptor --> b

Subsequently, robot β computes putative correspondences by matching the two sets of feature descriptors using nearest neighbor search implemented in OpenCV.  From the putative correspondences, robot β attempts to
compute the relative transformation using Nistér’s five-point
method [79] and Arun’s three-point method [80] combined
with RANSAC [56]. Both techniques are implemented in the
OpenGV library [81]. If geometric verification succeeds with
more than five correspondences, the loop closure is accepted
and sent to the robust distributed trajectory estimation module.

GNC 渐近非凸性，用没那么非凸的替代（surrogate）函数序列来作优化，该函数序列会收敛到原来的损失函数。

**Robust distributed initialization** Between every pair of robots,
inlier loop closures (green→) lead to similar estimates for the alignment
between frames (green-->). Each outlier loop closure (red→) produces an
outlier frame alignment (red-->), which can be rejected with GNC. 

