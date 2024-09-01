---
layout: post
title:  "point and line features review"
date:   2024-09-01 21:00:00 +0800
tags: 
    - slam
    - point/line feature
categories:
    - paper reading
---

## Point features

## Line features

### EDline
> [EDLINES: REAL-TIME LINE SEGMENT DETECTION BY EDGE DRAWING (ED) [AT11] ICIP2011](https://ieeexplore.ieee.org/document/6116138) <br>
> author: Cuneyt Akinlar and Cihan Topal 

EDLines的基本结构分为以下三部分：

- 输入灰度图，利用 Edge Drawing 算法进行提取，输出一系列连续的像素点
- 利用最小二乘法来连接像素点，避免连到外点
- 用 Desolneux 等提出的 Helmholtz Principle 来抑制误检的线段


Edge Drawing检测算法:

- 滤波核与图像卷积，进行高斯滤波，去噪
- 计算像素梯度，可使用 Prewitt，Sobel 等算子
- 选出梯度较大者为“锚”，“锚”(anchors)极可能为线段端点
- 连接锚点，生成初始线段

此时得到的线段连接了所有的锚点，因此并不平滑，不满足使用要求，因此使用 *最小二乘法* 拟合一条平滑的直线。

此时的直线可能欠拟合或过拟合，将图片中不是直线的部位当成直线，这时便需要使用 Helmholtz Principle 来选取合适的线段




