---
layout: post
title:  "[VINS-Fusion] Feature Manager"
date:   2024-09-16 23:37:00 +0800
tags: 
    - slam
categories:
    - fast-drone
---

feature manager 的主要数据结构为：
``` c++
class FeatureManager
    class FeaturePerId
        class FeaturePerFrame
```

`FeaturePerFrame` 包括某帧内某个路标观测的信息，如归一化平面坐标及速度、像素坐标、是否双目、时间偏移。

`FeaturePerId` 则主要包括 `vector<FeaturePerFrame> feature_per_frame`，以及 `frame_id`、`start_frame` 等路标点个体级别的信息。值得注意的是，vins假设特征点是被连续观测到的，因此只记录开始的帧，即可推断出 vector 内其他观测所属的帧。