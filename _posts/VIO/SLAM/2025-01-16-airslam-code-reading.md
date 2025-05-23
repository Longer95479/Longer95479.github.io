---
layout: post
title:  "AirSLAM Code Reading" 
date:   2025-01-16 15:40:00 +0800
tags: 
    - slam
categories:
    - AirSLAM Improvement
---

## 代码实现

首先看一下 airslam 代码的整体思路。

对于视觉惯性里程计，其核心是三个并行的线程：
->(处理msg)->(前端)->(后端)->

使用 `生产者-消费者` 的框架实现数据的有向流动。例如 map_builder.AddInput(data) 往里加数据，然后另外开一个线程消耗数据，提取特征，最后再开一个线程进行特征追踪匹配和后端优化。


### 参数的初始化

对于参数的配置，airslam 在 `read_config.h` 内定义了许多结构体，其中有三个最顶层的结构体，对应 airslam 的三大模块的参数，分别是：
- `VisualOdometryConfigs`
- `MapRefinementConfigs`
- `RelocalizationConfigs`

在这三大结构体内，还有许多子结构体，对应于一些子模块/子功能需要配置的参数。这些大结构体都有**构造函数**，在构造函数内:
- 读取总的 yaml 配置文件
- 之后将获取的配置信息分发给各个子config，也就是各个子config存储了相应配置
- 其他 非config 的模块初始化的时候会传入config结构体的实例，在其构造函数内将子 config 存储的参数赋值给成员变量，从而实现参数的传递。

以 `VisualOdometryConfigs` 为例，其内部声明了一系列子 config：
```c++
struct VisualOdometryConfigs{
  ...
  PLNetConfig plnet_config;
  SuperPointConfig superpoint_config;
  PointMatcherConfig point_matcher_config;
  LineDetectorConfig line_detector_config;
  KeyframeConfig keyframe_config;
  OptimizationConfig tracking_optimization_config;
  OptimizationConfig backend_optimization_config;
  RosPublisherConfig ros_publisher_config;
  RosSubscriberConfig ros_subscriber_config;    
  ...
```
在它的构造函数里 ` VisualOdometryConfigs(const std::string& config_file_, const std::string& model_dir_)`，会调用 yaml-cpp 的接口获取所有的键值对：
```c++
YAML::Node file_node = YAML::LoadFile(config_file_);
...
ros_publisher_config.Load(file_node["ros_publisher"]);
...
```

每个子 config 都有一个 `Load` 成员函数，在获取键值对后，每个子 config 成员均会调用各自的 `Load` 函数，传入上面声明的变量 `file_node`。每个 `Load` 函数本质上是对 yaml-cpp 接口的封装，以 `plnet_config.Load()` 为例：
```c++
  void Load(const YAML::Node& plnet_node){
    use_superpoint = plnet_node["use_superpoint"].as<int>();

    max_keypoints = plnet_node["max_keypoints"].as<int>();
    keypoint_threshold = plnet_node["keypoint_threshold"].as<float>();
    remove_borders = plnet_node["remove_borders"].as<int>();

    line_threshold = plnet_node["line_threshold"].as<float>();
    line_length_threshold = plnet_node["line_length_threshold"].as<float>();
  }
```

是否使用 imu，参数值在 camera 的配置文件里给出



### 特征检测器

主要文件：feature_detector.h 和 feature_detector.cc

类 FeatureDtector，有 6 个 `Detector()` 成员函数的重载，这 6 个重载可以由两类属性组合得到：
- 相机个数：2 种
  - 单目
  - 双目
- 特征类型：3 种
  - 点特征
  - 点特征、线特征
  - 点特征、线特征、线端点

2 * 3 = 6 种

根据 `Detector()` 和 以下的类构造函数可以看到，其调用了更底层的模块 `plnet`，因此如果需要*修改特征检测方法*，可以从此处入手。

```c++
  FeatureDetectorPtr feature_detector = std::shared_ptr<FeatureDetector>(new FeatureDetector(plnet_config));
```


### 增加 rosbag 支持

原始的 airslam 仅支持 ASL 格式的数据集，也就是图片文件形式的数据集，没有读取 rostopic 的接口，因此无法实时运行，也无法跑 rosbag 格式的数据集。

为了适配 rosbag 的功能，需要增加的代码包括：
- 某帧图像对应的 imu batch 合成代码
- 话题的订阅 read_configs.h
- 新线程用于读取 msg，处理后得到给特征提取的
- 新数据集的内外参配置

imu 数据 和 img 数据的频率有差异，imu 数据频率高，因此需要对两帧图像帧间的imu数据进行缓存打包，用于后续的预积分。差异如下：
```
img	  +  +   +  +
imu	***************
```

airslam 的 imu 预积分用的数据与 vins 有所不同，相比 vins 多用了一个数据，每两个相邻的 imu batch 都有图像前后两个imu数据的重叠,如下图：

![imu_batch](/assets/2024-09-01-airslam/imu_batch.png)

如果某个图像帧，在其之前无任何 imu 数据，则会丢弃该图像帧，因为无 imu 可以用于预积分。




