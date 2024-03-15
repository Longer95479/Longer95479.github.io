---
layout: post
title:  "ego planner grid_map"
date:   2024-02-29 10:54:00 +0800
categories: 
    - fast-drone
    - slam
    - ros
    - c++
---

- `FSM` 最顶层 -> 
     - `manager` -> 
        - `bspline_optimizer_` 和 `grid_map_`

- `GridMap` 类
    - `MappingParameters` 结构体
    - `MappingData` 结构体


---

- `geometry_msgs/PoseStamped` 是 `Pose` 和 `timestamp` 的结合
- `nav_msgs/Odometry.msg` 包含 `Pose`、`twist`（速度、角速度） 和 各自的协方差矩阵

---

一些子函数的功能：
```c++
/**
* @brief 给定地图外的点，找到地图内离它最近的点
*/
Eigen::Vector3d GridMap::closetPointInMap(const Eigen::Vector3d &pt, const Eigen::Vector3d &camera_pt)


/**
* @brief 主要干两件事，一是记录某体素的访问次数，
            把第一次被访问（count_hit_and_miss == 1）的体素 id 放入 cache
*        二是若体素被击中，则被击中次数加一
*/
int GridMap::setCacheOccupancy(Eigen::Vector3d pos, int occ)

```
---
`void GridMap::raycastProcess()` 主要干的事：
- 以一帧深度图为一个执行周期
- 深度图上的点已经按一定间隔映射到世界坐标系
- 依次以相机位置和深度点位置为两个端点，连线，判断深度点是否超出地图和射线长度
    - 如果超过了地图，则找到地图上沿该方向离该点最近的点，`setCacheOccupancy(pos, occ)` 的 `occ` 设为 `0`
    - 如果超出射线长度，则将在极限距离上的那个端点 `occ` 设为 `0`
    - 都没有以上情况，则将该点 `occ` 设为 `1`，表示占用
- 之后在该点和相机之间进行 `raycasting`，起点是深度点，终点是相机位置，遍历射线经过的 `栅格 grid`，并将遍历到的点 `occ` 均设置为 `0`，因为能看到更远的点，那么这点和相机连线之间肯定没障碍了。
- 对 `local_update_range_` 内的栅格进行 `log_odd_update` 的叠加，更新范围外的栅格先置为 `clamp_min_log` 再叠加
- 加速策略：
    - **深度点可以多个落在同一栅格内，因此同一个栅格可多次计数（这使得计数有意义）**，但如果对这几个点作光线投射，投射路径将一模一样，此时则没必要重复
    - 从不同深度点射向相机位置的光线在最后都会比较接近，因此相近光线后半部分经过的栅格基本一致，此时也没必要重复

```xml
    <!-- local fusion -->
    <param name="grid_map/p_hit"  value="0.65"/>
    <param name="grid_map/p_miss" value="0.35"/>
    <param name="grid_map/p_min"  value="0.12"/>
    <param name="grid_map/p_max"  value="0.90"/>
    <param name="grid_map/p_occ"  value="0.80"/>
    <param name="grid_map/min_ray_length" value="0.1"/>
    <param name="grid_map/max_ray_length" value="4.5"/>
```
```c++
mp_.prob_hit_log_ = logit(mp_.p_hit_);
mp_.prob_miss_log_ = logit(mp_.p_miss_);
mp_.clamp_min_log_ = logit(mp_.p_min_);
mp_.clamp_max_log_ = logit(mp_.p_max_);
mp_.min_occupancy_log_ = logit(mp_.p_occ_);
mp_.unknown_flag_ = 0.01;
```
---
`void GridMap::clearAndInflateLocalMap()` :
- `void GridMap::raycastProcess()` 计算出了 `local_bound_`，叠加上`local_map_margin_`, 在叠加上局部变量 `vec_margin`，为 `local_bound` 增加裕量
- 将 `vec_margin` 范围内的体素设置为 `unknown`，意为**清除**
- 在 `local_bound` 范围内的体素膨胀，并更新占用状态，先清零后使用 `阈值 min_occupancy_log_` 判断占用情况（涉及 `occupancy_buffer_inflate_` 变量）

---

[`Mat::convertTo`: Converts an array to another data type with optional scaling.](https://docs.opencv.org/2.4/modules/core/doc/basic_structures.html#mat-convertto)

---

在初始化部分，为 `message filter synchronizer` 绑定回调函数，使用了如下代码：

```c++
/* 初始化 synchronizer */
sync_image_pose_.reset(new message_filters::Synchronizer<SyncPolicyImagePose>(
        SyncPolicyImagePose(100), *depth_sub_, *pose_sub_));

/* 绑定回调函数 */
sync_image_pose_->registerCallback(boost::bind(&GridMap::depthPoseCallback, this, _1, _2));
```

其中用到了 `boost::bind`，其用于`绑定成员函数`的具体规则如下：
- 类的成员函数不同于普通的函数，因为成员函数指针不能直接调用operator(),它必须被绑定到一个对象或指针，然后才能得到this指针进而调用成员函数。因此bind需要 “牺牲”一个占位符，要求提供一个类的实例、引用或者指针，通过对象作为第一个参数来调用成员函数，即
    ```c++
    bind(&X::func,x,_1,_2,…)
    ```
- 这意味着使用成员函数时只能最多绑定8个参数。例如，有一个类demo
    ```c++
    struct demo{
     int f(int a,int b){return a+b;}
    };
    ```
    那么，下面的bind表达式都是成立的：
    ```c++
    demo a,&ra = a;    //类的实例对象和引用
    demo * p = & a;     //指针
    cout<<bind(&demo::f,a,_1,20)(10)<<endl;
    cout<<bind(&demo::f,ra,_2,_1)(10,20)<<endl;
    cout<<bind(&demo::f,p,_1,_2)(10,20)<<endl;
    ```
- 注意：我们`必须在成员函数前面加上取地址的操作符&`，表明这是一个成员函数指针，否则会无法编译通过，这是与绑定函数的一个小小的不同。bind同样支持绑定虚拟成员函数，用法与非虚函数相同，虚函数的行为将由实际调用发生时的实例来决定。

参考：[Boost::bind使用详解](https://www.cnblogs.com/blueoverflow/p/4740093.html)


