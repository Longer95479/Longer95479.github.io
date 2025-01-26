---
layout: post
title:  "Equivalence between Kalman Filter and Factor graph"
date:   2025-01-26 17:22:00 +0800
tags: 
  - Quaternion
categories:
  - math
---

$$
P(X,Z) = P(Z|X)P(X)
$$

先验（prior）：

$$
P(X_1)
$$

预测（prediction）：

$$
P(X_2) = \int P(X_2 | X_1) P(X_1) \bf{d} X_1
$$

更新（update）：

$$
P(X_2 | Z_2) \propto P(X_2, Z_2) = P(Z_2 | X_2) P(X_2)
$$
