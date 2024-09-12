---
layout: post
title:  "[Paper Reading] Balancing the Budget: Feature Selection and Tracking for Multi-Camera Visual-Inertial Odometry using SE2(3) Based Exact IMU Pre-integration"
date:   2024-09-08 16:05:00 +0800
tags: 
    - slam
categories:
    - paper reading
---



The main contributions of this work are the following:
- A novel factor graph formulation that tightly fuses tracked
features from any number of stereo and monocular
cameras, along with IMU measurements, in a single
consistent optimization process.
- A simple and effective method to track features across
cameras with overlapping FoVs to reduce duplicate
landmark tracking and improve accuracy.
- A submatrix feature selection (SFS) scheme that selects
the best landmarks for optimization with a fixed feature
budget. This bounds computational time and achieves the
same accuracy compared to using all available features.
- Extensive experimental evaluation across a range of
scenarios demonstrating superior robustness, particularly
when VIO with an individual camera fails.


The state of the sensor rig at time ti
is defined as follows,
xi , [Ri
, pi
, vi
, b
g
i b
a
i
] ∈ SO(3) × R
12 (1)
where: Ri
is the orientation, pi
is the position, vi
is the linear
velocity, and b
g
i
, b
a
i
are, respectively, the usual IMU gyroscope
and accelerometer biases. **In addition to the states, we track
point landmarks m` as triangulated visual features.**


The location of the landmarks detected by the stereo camera
pair is initialized using stereo triangulation. For landmarks
detected in monocular cameras, we triangulate the feature
location over the last Nobs frames using the Direct Linear
Transform (DLT) algorithm from [20].


VII-B. Initialization
We initialize the IMU biases by averaging the first 1 s of
data at system start up (assuming the IMU is stationary).
To solve the scale initialization problem, which is often
present in monocular visual odometry systems, we combine
preintegrated IMU measurements and depth from the stereo
camera pair. Notably, the CCFT method allows features from
the stereo camera to flow into the monocular camera, speeding
up the depth initialization process.


---

### 个人想法

相机间的特征追踪（匹配）可以分成两步：
- 使用重投影，最小距离的为其匹配者，但如果外参不准，则该方法大概率失效
- 如果相机间重叠区特征点多，但匹配的数目却很少，则可以意识到是外参的问题，此时可以
使用描述子的方法进行匹配

此外也可以线用描述子的方法校准好各个相机的外参，从而使第一步尽可能地成功，因为第二步是用来兜底的，计算量比较大
