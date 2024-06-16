---
layout: post
title:  "[Paper Reading] Autonomous Cave Surveying With an Aerial Robot (TRO2022)"
date:   2024-06-06 21:00:00 +0800
tags: slam
categories:
    - paper reading
---

> 

## 针对的问题

- 全暗洞穴环境的无人机自主飞行
- 人工探险危险，因为存在洞穴结构不坚固
- 使用机器人建图可以降低危险，且需要实时回传信息，但现有的方法基本未考虑高分辨率感知建模的同时，降低高带宽，以在低带宽场景正常工作

## 采用的方法

- 传感器和硬件设备：
    - 深度相机用于建图
    - 朝下的相机用于状态估计
    - 朝前和朝下的灯光照明

- this work compactly represents sensor observations as Gaussian
mixture models and maintains a local occupancy grid map for a
motion planner that greedily maximizes an information-theoretic
objective function. The approach accommodates both limited field
of view (FoV) depth cameras and larger FoV LiDAR sensors
and is extensively evaluated in long duration simulations on an
embedded PC.

## 达到的效果

无人机在洞穴自主飞行探索，实时回传三维地图

## 存在的不足/未来的工作

- 未对洞穴的一些特征（如钟乳石）进行语义分类和编码
- 3D 地图投影成 2D地图用于探险者使用，防止迷路
- 扩展成多机
- 引入场景重识别，以减少漂移，提高地图的一致性，也有利于多机飞行
