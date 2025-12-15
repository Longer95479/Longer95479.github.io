---
layout: post
title:  "Handcraft Ray Tracing"
date:   2025-12-15 22:30:00 +0800
tags: 
  - programming
  - math 
categories:
  - programming
  - math 
---

本文将介绍如何手搓光线追踪。先上效果图：

![](/assets/2025-12-15-Handcraft-Ray-Tracing/my_raytracing-120p.gif)

![](/assets/2025-12-15-Handcraft-Ray-Tracing/my_raytracing_blue-120p.gif)


代码仓库：[handcraft-MVS](https://github.com/Longer95479/handcraft-MVS)。

实现了：

- 漫反射
- 基于菲涅尔定律的折射与反射
- 允许多光源
- 圆形物体

TODO：

- [ ] 长方体物体
- [ ] 另起一篇实现光栅化
