---
layout: post
title:  "Transformer is a clustering machines"
date:   2025-02-03 21:48:00 +0800
tags: 
    - transformer 
categories:
    - paper reading
---

## ResNets

$x_0$ 可以是图片像素强度向量，经过多层网络后，学习到特征 $x_L$ (Learned representation)，该特征相比于原始图像更利于图像分类。

$$
x_0 \rightarrow x_1 \rightarrow \dots \rightarrow x_L \\
L = \# layers
$$

将学习到的特征放入到传统的机器学习分类器（Classical ML step）中进行分类（例子为 logistic regression）：

$$
x_L \rightarrow y = \sigma (\beta^T x_L)
$$

在 2012 年以前，人们会将 $x_L$ 直接作为原始图片，放入到传统的分类器当中。
