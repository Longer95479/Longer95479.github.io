---
layout: post
title:  "CSLAM and Mutual Localization"
date:   2024-04-09 21:00:00 +0800
tags: slam
categories:
    - paper reading
---

## 引言

状态估计是机器人系统执行高级功能（路径规划、稠密地图构建）的基础，状态包括位置和姿态。单机的状态估计已经被较好地解决了，而多机则比单机更为复杂，需要自身和相互定位。

CSLAM 全称为 collaborative SLAM，用于估计机器人间的 `相对位姿` 和 `全局一致的轨迹`，可用于自然灾害多机器人搜救、未知环境多机探索、多机器人工业巡检、农业植保等。本篇将介绍 CSLAM 的整体框架，侧重于 CSLAM 的初始化问题，以及机器人间的相互定位。

先前的集群状态估计一般使用 GPS、UWB、RTK-GPS和动捕系统等外部设备，且包括中心节点集中处理数据。众多的外部设备影响实际应用的部署效率，系统对中心节点失效敏感。

去中心化的集群架构（不局限于状态估计）愈发受欢迎，因为其有以下优点：
- 无需保证所有机器人都与中心节点有稳定的通信连接，在通信受限的环境中有更强的适应性
-  每个机器人都可以独立于团队的其余部分行动，从而使整个系统更能容忍 *单点故障 (failures of individual robots)*。如果希望集群架构是去中心化的，那么其基础——状态估计，也应该是去中心化的

根据上层任务和应用场景（自组装、编队飞行、未知环境探索），得出 CSLAM 的技术要求（technical requirements）：
- 只需要机载传感器
- 架构是去中心化的
- 任务分配是分布式的，即计算不要冗余
- 机器人距离较近时，需要高精度的相互定位
- 机器人距离较远时，高精度相互定位难以获得，更重要的是保证状态估计的全局一致性
- 高精度的自身定位，保证飞行控制的稳定性

[$D^2$ SLAM](#d2-slam-decentralized-and-distributed-collaborative-visual-inertial-slam-system-for-aerial-swarm) 是一个较完整的工作，包括初始化（坐标系统一）、近场状态估计和远场状态估计，且被设计成 *去中心化* 和 *分布式*。

相互定位可以认为是近场状态估计，可分成 *基于地图* 和 *基于相互观测* 的方案。相对定位若发生在最开始，则可认为是各个坐标系的统一或初始化。

基于地图的方案仅仅适用于环境特征较为稠密的室内，在室外则有诸多局限性。地图环境定位的另一个问题是需要较大的通信带宽。

基于相互观测的方案所用传感器包括
- 视觉：如捕捉红外灯光，marker，直接对其他无人机进行视觉等，但存在匿名性（歧义性）
- UWB：以提供十厘米上下精度的测距信息。使用UWB测距信息也可以用于辅助定位，但是单一uwb信息并不可观，也缺乏定向信息。这使得我们需要和其他方法的融合

[$D^2$ SLAM](#d2-slam-decentralized-and-distributed-collaborative-visual-inertial-slam-system-for-aerial-swarm) 即为地图方案。
[Omni Swarm](#omni-swarm-a-decentralized-omnidirectional-visual-inertial-uwb-state-estimation-system-for-aerial-swarms) 既使用了地图方案，也使用了相互观测方案。

[Kimera-multi 的来龙去脉](https://www.bilibili.com/read/cv24168559/) 和 [Hydra](#foundations-of-spatial-perception-for-robotics-hierarchical-representations-and-real-time-systems)


Fei Gao组在 *基于相互观测* 的方案上的一系列工作如下（由早期到最近）：
- [*匿名条件* 下的相互定位（视觉marker）](#certifiably-optimal-mutual-localization-with-anonymous-bearing-measurements)
- [*部分观测* 下的相对定位（视觉 + tagged LED）](#bearing-based-relative-localization-for-robotic-swarm-with-partially-mutual-observations)
- [同时相对定位与*时间同步*（）](#simultaneous-time-synchronization-and-mutual-localization-for-multi-robot-system)
- [相对位姿初始化：匿名+部分观测（视觉检测）+ 主动](#fact-fast-and-active-coordinate-initialization-for-vision-based-drone-swarms)

可以看到，最新工作同时考虑匿名和部分观测，并且使用视觉检测，以实现传感器的轻量化。这些工作，联合优化所有坐标系的相对旋转，而不是像 D2SLAM 等框架两两（between each pair of robots）进行坐标系对齐。

### 可做方向

- [FACT](#fact-fast-and-active-coordinate-initialization-for-vision-based-drone-swarms) 等一系列基于相互测量的初始化方法，是将所有相对位姿作为优化变量联合优化，但并不是分布式的（是去中心化的吗？在每个节点上均运行，以我目前掌握的信息应该是去中心化的），算法在不同机器人上重复运行；且不能临时扩展集群数目？
- [D2SLAM](#d2-slam-decentralized-and-distributed-collaborative-visual-inertial-slam-system-for-aerial-swarm) 是去中心化和分布式的，但未使用相互测量，且其在初始化阶段是两两进行坐标系统一的

因此可探索一下能否实现 *基于相互观测量的去中心化和分布式的初始相对位姿联合优化*

- 针对集群相互定位的初始化阶段，精简传感器后，防止鲁棒性下降的问题
    - 提高视觉检测跟踪的鲁棒性，如 [bearing-angle 方法](#a-bearing-angle-approach-for-unknown-target-motion-analysis-based-on-visual-measurements) 利用了检测框大小，可以融合一下
    - 提高相对定位算法对视觉检测不确定的容忍阈值（太笼统）
        - 针对虚警还是漏警？
    - 视觉检测 和 UWB都比较不稳定，地图方案召回率低，如何更好地结合地图方案和相互测量方案？（omni-swarm 算是混合使用，但其作者并不满意）
    - 对于地图方案，加入 *机间语义回环检测*，提高回环检测鲁棒性，减少数据交换
- 相对定位时间同步上，D2SLAM 未考虑时间同步，可分析一下其是否有时间同步的必要 
    - 使用了匀速假设，可否松弛该假设？


## 较完整的系统

### D2 SLAM: Decentralized and Distributed Collaborative Visual-inertial SLAM System for Aerial Swarm

> T-RO 2024
<br>
原文：[arXiv:2211.01538](https://arxiv.org/abs/2211.01538)

以下来自 [从单机到多机的无人机与机器人集群的实时定位与建图技术（SLAM）：综述](https://zhuanlan.zhihu.com/p/608056877)

> 在Racer的研究中，我又得到了不少启发：
> 1. 自主无人机群仅仅在无人机互相距离较近的时候有相互高精度定位的必要，飞远了自然也没有必要互相避障了，这时候相对定位精度的重要性自然降低。 
> 2. 当无人机相距较远的时候，互相并不碰面，这时候我们更关注的是全局一致性：也就是地图不能随着飞行轨迹的飞行而飘移，要飞一圈回来，飞机仍然在之前的地方，这样地图才能进行后续的应用。
> 3. 是uwb和视觉检测用起来有时候很麻烦，容易被复杂环境干扰。我们需要一个更加可靠且通用的方案，UWB也好，视觉检测也好，可以作为“dlc”挂进去，也可以根据环境的不同不去挂载。

> 在这些思考的基础上，我又加入了分布式计算的思想，做出了我博士期间的最后一个工作 $D^2$ SLAM，分布式和去中心化的联合SLAM系统。

> $D^2$ SLAM根据前文做出的反思引入了两个（在我看来比较重要的）观念：
> 1. 无人机群的“近场“状态估计(near-field state estimation），当无人机集群中的飞机距离较近，通信良好的时候，我们有必要（为了互相避障和紧密协作）也有能力建立高精度的局部定位（自身状态估计）和无人机的互相定位。我把这部分能力称作近场估计
> 2. “远场“状态估计（far-field state estimation)，当无人机群中的飞机互相距离较远时，为了任务的完整性，我们关注的是地图全局一致性，也就是地图不能乱跑，这种能力被我称作远场估计。

> $D^2$ SLAM的贡献在于良好的解决了上面的两种问题；在延续了我们已有的全向视觉的思想的同时，引入了分布式计算来改善计算效率。


存在什么不足？

- 虽然使用分布式后端，但集群规模受限于 *通信和前端计算* 能力
- 相比之前的工作 [omni swarm](#omni-swarm-a-decentralized-omnidirectional-visual-inertial-uwb-state-estimation-system-for-aerial-swarms)，该工作更为传统，仅使用了地图方案，而未使用相对测量（视觉检测、UWB）
- 为了分布式优化的收敛性，未进行在线时间戳估计和外参矫正

**D2SLAM使用基于地图的观测来修正相对漂移，当特征稀少的时候，
就退化为了单机 VIO，而使用线特征和视角则可以解决这个问题** Environments: In open environments with limited environmental features, such as grasslands or rough walls, sparse
visual SLAM faces challenges in feature matching for relative
localization and loop closure detection. In response to these
limitations, our system is designed to downgrade to single-robot
VIO, ensuring flight safety under such conditions.

**IV. SYSTEM OVERVIEW - C. Multirobot Map Merging**: This alignment, triggered by relative measurements, integrates the coordinate systems without exchanging
landmarks, allowing each UAV to maintain its individual sparse
map. 
<br>
Specifically, the system adopts the reference frame of the
UAV with the smaller ID, and the larger ID UAV’s states in
D2VINS and D2PGO are converted to this unified system.


**V. FRONT END - A. Visual Data Preprocessing**：In D2SLAM, to conserve communication bandwidth, complete keyframe information, such as SuperPoint features and
global descriptors for each camera view, is broadcasted to other
UAVs only in discover or near communication modes. On the
other hand, the compact keyframes, include only the NetVLAD
descriptors, are used in far mode to save bandwidth.
<br>
Fig. 4(b) illustrates the map-merging process in D2SLAM.
UAVs continuously transmit heartbeat packets for detection by
others, facilitating their discovery. 


多机地图合并是怎么个流程：
- 每架无人机广播心跳包，用于被其他无人机发现


### Omni-swarm: A Decentralized Omnidirectional Visual-Inertial-UWB State Estimation System for Aerial Swarms

> T-RO 2022
<br>
原文：[arXiv:2103.04131](https://arxiv.org/abs/2103.04131)


针对什么问题？

- 可观性问题
- 复杂的初始化
- 定位精度不足
- 缺乏全局一致性

采用什么方法？

- 针对可观性：双目鱼眼（相当于全向摄像头）+ uwb
- 针对初始化：multi-drone map-based localization
- 针对自身与相对定位精度：VIO + visual drone tracking algorithms
- 针对全局一致性：  multi-drone map-based localization


达到什么效果？


存在什么不足？

- 对相机内外参标定的依赖
    - 可做工作：构建在线错误检测和校准
- 后端算法复杂度 $O(n^2)$，使得算法难以应用于大规模集群（超过100架 UAV）
- 通信距离较短，限制了机间的工作距离（22.4m）
    - 考虑路由（AODV）


### Decentralized Visual-Inertial-UWB Fusion for Relative State Estimation of Aerial Swarm

> ICRA 2020
> [arXiv:2003.05138](https://arxiv.org/abs/2003.05138)


### Kimera-Multi: Robust, Distributed, Dense Metric-Semantic SLAM for Multi-Robot Systems

> T-RO 2022
<br>
[arXiv:2106.14386](https://arxiv.org/abs/2106.14386)

Kimera-Multi是一个分布式多机器人协同SLAM系统，对于每个单独的机器人，它们通过视觉惯性传感器使用Kimera的Kimera-VIO和 Kimera-Semantics两个模块分别估计各自的局部位姿和局部mesh，当两个机器人可以互相通讯时，*初始化*基于分布式渐进式非凸性算法（distributed graduated nonconvexity algorithm）的分布式位置识别检测和位姿图优化功能，通过机器人之间的闭环检测实现对outliers的鲁棒，最后提高位姿估计的准确性和mesh重建的精度。


### Foundations of Spatial Perception for Robotics: Hierarchical Representations and Real-time Systems
> IJRR
<br>
> [arXiv:2305.07154](https://arxiv.org/abs/2305.07154)


针对什么问题？


采用什么方法？


达到什么效果？


存在什么不足？


### Multi S-Graphs: An Efficient Distributed Semantic-Relational Collaborative SLAM
> 原文：[arXiv:2401.05152](https://arxiv.org/abs/2401.05152)


针对什么问题？

- 大多数 CSLAM 技术依赖于原始传感器测量，或诸如关键帧描述符之类的低级特征，可能导致缺乏对环境的深入理解而导致错误的回环检测
- 交换原始测量值和低级特征占用较多通信带宽，限制了系统的可扩展性

采用什么方法？

- **Distributed** LiDAR-based collaborative SLAM algorithmm
- Take advantage of the **hierachical semantic information** from situational graph(S-Graph)
- Works in **multi-robot kidnapped problem**: Each robot does not know where it has started and where the other robots are.

a local S-Graph is executed in every robot，the distilled S-Graphs and room descriptors are sent to other robot

room closure detection, the local S-Graph and the distilled S-Graph from other robots are used to generate collaborative S-Graph

Room-Based Inter-Robot Loop Closure: In order to combine the raw
information of the point clouds and take advantage of the
high-level semantic information that each room contains,
we generate a hybrid descriptor for each room, a Room
Descriptor. 

- What is the Distilled S-Graphs?
    - We marginalize
the local S-graph generated by each robot to only trans-
mit semantic-relational information and not the low-level
information stored on each keyframe. Each distilled graph
includes a reference node that represents the origin of the
local coordinated system of each robot. This distilled S-graph
is transmitted to the rest of the robots.

*My thought: Pose-semantic constrains can be represented as What a 3D semantic object look like in measurements, 
or which part of the measurements are belong to one semantic object*

Scan-Context is linear motion sensity, to solve this problem, we exploit the semantic and hierarchical
information from Local S-Graph to obtain the room center
and its boundaries to extract all the keyframes obtained
from within the room and generate its corresponding Room
Centric point cloud.

达到什么效果？



存在什么不足？

 It is crucial for Room
Descriptors to remain unique and constant over time for ac-
curate matches, as identical rooms or changing environments
can lead to erroneous matches.

## 相互定位（近场）

### FACT: Fast and Active Coordinate Initialization for Vision-based Drone Swarms

> IROS 2024 已投稿
<br>
原文：[arXiv:2403.13455](https://arxiv.org/abs/2403.13455)
<br>
视频：[FACT: Fast and Active Coordinate Initialization for Vision-based Drone Swarms](https://www.bilibili.com/video/BV1sA4m1A7DD/?vd_source=e371652571b1539bbd501fb7adb6cfc4)



针对什么问题？

- 匿名+部分相互观测：针对有着 SWaP(size, weight and power) 约束的无人机集群，仅使用`基于视觉的匿名的部分相互观测`，而*不使用其他传感器*辅助（如 UWB，动捕和特制信标），如何实现`坐标系的初始化`？

- `基于视觉的匿名的相互观测` 带来了新的挑战：
    - 有限的 FOV：视角和传感器距离有限，某些无人机之间不存在相互观测
    - 匿名：无法直接知晓通过视觉检测到的无人机的 id 是多少
    - 安全性：在初始化阶段，无人机需要避免和其他无人机以及环境发生碰撞

采用什么方法？

- 问题建模：相互观测向量求和为零向量 -> 最小二乘优化问题

- 整体思路：原地旋转，判断相互测量是否完整，若不满足，则随机移动，重复以上步骤。当 $rank(Z^*) \leq N+1$，意味着所有测量均已获得，均为相互的。此时可求解出各机坐标系的相对旋转 $R_{t_0}$，不受匿名条件的影响。求解出 $R_{t_0}$ 后，视觉测量和 VIO 的关联问题可以转化为求解关联矩阵 $A_i$，等价于求解二分图匹配问题，可使用匈牙利算法求解。

- 针对有限的 FOV：原地旋转 ＋ 多轮随机移动

- 针对匿名的测量：使用匈牙利算法求解 $A_i$

- 针对安全性：设置固定的机间最小距离；设置最大移动范围；随机选择探索或向最远无人机移动；确认目标点不在障碍物内；使用 ego-planner 生成轨迹用于下次观测的目标点

- 针对解的全局最优：将非凸原问题松弛为 SDP（semi-definite programming）问题，松弛后为凸问题

- 视觉测量：检测使用 YOLOv8，训练集来自现实，使用 NOKOV 自动标注后手动微调；跟踪使用 BoT-SORT，并分配 ID

达到什么效果？
- 原地旋转和随机移动，重复多轮，能够弥补 FOV 有限的缺陷
- SDP相比与局部优化方法，计算出的旋转的 MAE 均值和波动均更小，计算消耗时间更短
- 随即移动能够打破具有对称性的初始队形

存在什么不足？
- 无人机数目 N 需要事先给定
- 相对观测的误差对系统的影响未被探索和分析，作者的未来工作是提高观测噪声的阈值，以更鲁棒地处理视觉识别中的不确定性
- 未进行机间的时间偏移估计
- 

个人疑惑
- 建模部分公式似乎有些错误
- $rank(Z^*) \leq N+1$ 是否意味着所有观测均为相互？不是的话意味着什么？


### Bearing-based Relative Localization for Robotic Swarm with Partially Mutual Observations

> RA-L 2023
<br>
原文：[arXiv:2210.08265](https://arxiv.org/abs/2210.08265)
<br>

针对什么问题？

- 先前的工作已经开发了可证明的鲁棒的求解器，用于 *每对* 机器人之间的相对变换估计，未考虑部分观测的情况

- 基于地图的相对位姿估计所需带宽大，受环境影响

- 仅使用 2D 视觉检测的基于测角的相互定位，由于常用的相机 FOV 有限，机器人通常只能观测到部分机器人，形成*部分观测图（partial observation graph）*

- 由于问题建模得到的公式极端非凸，传统局部优化方法可能陷入局部最小，这在相对定位问题中十分常见

采用什么方法？

- 针对部分相互观测的情况，本文基于不同的变量边缘化方法，推导了两种可解的问题建模公式，得到一个统一的在 Stiefel 流形上的最小值问题，*联合优化所有机器人的位姿*。作者对这两种公式化进行了细致分析和对比。

- 针对问题非凸的情况，本文将原始非凸问题松弛为 SDP（semi-definite programming） 问题，SDP 问题是凸的，能够求得全局最小值。此外，本文还提供了一个充分条件，在该条件下，能够严格保证无噪声情况下松弛的紧致性（严密性）

- 为了避免现实中噪声的影响导致解不准确，使用了 *秩约束优化*

达到什么效果？

- 相比于局部优化算法（黎曼流形优化、LM优化算法），本文的方法能够实现解的最优
- 使用了变脸消除策略，变量数目固定，因此 *计算时间* 只和机器人数目有关。1Hz的坐标系调整
- 鲁棒性高，体现在抗噪声能力比*纯SDP*、*黎曼流形优化*强
- 实际实验使用 鱼眼相机 和 *带标签的LED* 来获取相互观测

存在什么不足？

- 未来作者将把注意力放在，规划合适的编队，以满足相互定位可观性需求
- 使用了 *带标签的LED* 来产生 *非匿名* 的相互观测
- 在每个机器人内都进行了问题建模和优化求解，计算冗余了

个人疑惑

- 如何松弛？什么是 SDP 问题？
- 解的好坏可以从 $Z^*$ 的秩和某个 cost 反映出来，有待进一步了解




### Certifiably Optimal Mutual Localization with Anonymous Bearing Measurements

> RA-L with IROS 2022
<br>
原文：[arXiv:2203.09312](https://arxiv.org/abs/2203.09312)

>[SupplementaryMaterials.pdf](https://github.com/ZJU-FAST-Lab/CertifiableMutualLocalization/blob/main/SupplementaryMaterials.pdf)


针对什么问题？

- 如何仅使用匿名的相互测量和自定位里程计，实现相互定位。
    - 相互测量是匿名的：相互观测量与接收到的里程计的对应关系并非已知
- 局部优化方法对初值敏感

采用什么方法？

- 提出一个可证明的最优算法，仅使用匿名的测角测量，建模成 *混合整数二次约束二次问题（mixed-integer quadratically constrained quadratic problem, MIQCQP）*
- 原问题松弛成 SDP 问题，以求全局最优解

达到什么效果？

- 可以确定方位姿态的对应关系
- 在一定条件下能够恢复机器人之间的初始相对姿态，即 $corank(Z^*) = 1$
- 在最优性、运行时间、鲁棒性和可扩展性上，比传统局部优化算法效果好
- 可用于多机器人单目 SLAM 的 *地图融合（map fusion）*，以及多机任务中的 *坐标系对齐（coordinate alignment）*

存在什么不足？

- 使用 *动捕* 和 VIO 作为里程计估计，使用 *AprilTag* 获取测角测量
- 作者未来工作：探索本文方法的噪声容忍阈值，为实际应用提供更有力的保证。换句话说，没噪声能够容易证明解的最优性，但如果有噪音，噪声方向不确定，多大的噪声还能保证解的最优呢？
- 规模扩展性较差，当机器人个数为 5 时，使用 c++ 的计算时间已经达到 11 秒
- 不是分布式的

>作者在论坛上提到，当提供一些假设的时候，例如无人机之间能够互相观测，那么问题会变得简单很多，即退化成 $tr(QRR^T)$，这个问题在 PGO 方向已被研究，能够达到几千几百架的规模 


### Simultaneous Time Synchronization and Mutual Localization for Multi-robot System

> ICRA 2024
<br>
原文：[arXiv:2311.02948](https://arxiv.org/pdf/2311.02948.pdf)
<br>
视频：[Simultaneous Time Synchronization and Mutual Localization for Multi-robot System](https://www.bilibili.com/video/BV1ew411r7z8/?vd_source=e371652571b1539bbd501fb7adb6cfc4)

针对什么问题？

- 机器人之间的时间存在偏移

采用什么方法？


达到什么效果？


存在什么不足？

- 使用匀速假设 -> 可否扩展至匀加速？
- 建模、求解与实验均只针对两架无人机的场景，一架观测另一架



### A Bearing-Angle Approach for Unknown Target Motion Analysis Based on Visual Measurements

> IJRR <br>
原文：[arXiv:2401.17117](https://arxiv.org/pdf/2401.17117.pdf)
<br>
视频：[【IJRR最新成果】利用被忽视的视觉信息大幅提升目标定位可观性](https://www.bilibili.com/video/BV1EC411z7Lz/?spm_id_from=333.337.search-card.all.click&vd_source=e371652571b1539bbd501fb7adb6cfc4)

针对什么问题？

- 利用移动的单目相机，估计移动目标的运动状态
- bearing-only 仅利用目标的三维方向信息来估计目标的运动状态，局限性在于，要求观测者具有横向高机动性以满足可观性的需求，*如何去掉横向运动的约束条件，同时保证可观性*

采用什么方法？

- 视觉检测算法检测到目标时，会给出一个检测框（bounding box），包含两个有用信息
    - 检测框中心（已广泛研究）：给出指向目标的方向向量
    - 检测框大小（未被充分发掘）：由相对距离、目标物体尺寸和相机姿态共同决定
- 提出 bearing-angle 方法

达到什么效果？

- 理论分析表明，相比于传统的 bearing-only 算法，该方法显著提高可观性
- 不需要额外的检测算法或设备，因为该方法额外利用了检测框的大小信息

存在什么不足？

- 该算法存在一个假设：不同视角下的物体大小不变
    - 因此似乎很难处理目标尺寸/目标投影尺寸剧烈变化的情况



### CREPES: Cooperative RElative Pose Estimation System

> IROS 2023
<br>
原文：[arXiv:2302.01036](https://arxiv.org/abs/2302.01036)
<br>
视频：[CREPES: Cooperative RElative Pose EStimation towards Real-World Multi-Robot Systems](https://www.bilibili.com/video/BV1CW4y1Y79q/?spm_id_from=333.999.0.0&vd_source=e371652571b1539bbd501fb7adb6cfc4)

针对什么问题？

- 使用视觉进行，例如捕捉红外灯光，marker，直接对其他无人机进行visual detection等；优势是视觉测量精确的相对定位，缺点是歧义性：搞明白哪个飞机是哪个是最大的问题。

采用什么方法？

- 不同的灯光组合

达到什么效果？


存在什么不足？





### Meeting-Merging-Mission: A Multi-robot Coordinate Framework for Large-Scale Communication-Limited Exploration

> IROS 2022
<br>
原文：[arXiv:2203.09312](https://arxiv.org/abs/2203.09312)


针对什么问题？


采用什么方法？


达到什么效果？


存在什么不足？



### DIDO: Deep Inertial Quadrotor Dynamical Odometry

> RA-L with IROS 2022
<br> 
原文：[arXiv:2203.03149](https://arxiv.org/abs/2203.03149)
<br>
视频：[DIDO: Deep Inertial Quadrotor Dynamical Odometry](https://www.bilibili.com/video/BV1dU4y1Z773/?spm_id_from=333.999.0.0&vd_source=e371652571b1539bbd501fb7adb6cfc4)



## 分布式与去中心

### Asynchronous Distributed Smoothing and Mapping via On-Manifold Consensus ADMM

> [arXiv:2310.12320
](https://arxiv.org/abs/2310.12320)


 In this paper we present a fully distributed,
asynchronous, and general purpose optimization algorithm for
Consensus Simultaneous Localization and Mapping (CSLAM).

 we develop a CSLAM
back-end based on Consensus ADMM called MESA (Manifold,
Edge-based, Separable ADMM

In C-ADMM, local objective function relies on all optimaization parameters, meaning copy all variables to local.

Multiple robots may observe the same variable (e.g. two
robots observe the same landmark). Therefore, while 𝑀𝑖
are
disjoint subsets of 𝑀, each Θ𝑖
is a non-disjoint subset of Θ.



### Distributed Simultaneous Localisation and Auto-Calibration using Gaussian Belief Propagation

> [arXiv:2401.15036](https://arxiv.org/pdf/2401.15036.pdf)

文章的 Background 部分提到，Kimera-multi 及其先前的基础工作属于 PGO-based 的工作，需要*完整的机器人之间的相对变换* 和 *只能处理各向同性协方差*


### A Survey of Distributed Optimization Methods for Multi-Robot Systems

> [arXiv:2103.12840](https://arxiv.org/abs/2103.12840)

Cited in Distributed Simultaneous Localisation and Auto-Calibration using Gaussian Belief Propagation: 
> Consensus Alternating Direction Method of Multipliers (C-ADMM) displays superior convergence rates to alternative distributed optimization approaches. 



