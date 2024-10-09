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

`FeaturePerFrame` 包括某帧内某个路标被观测的信息，如归一化平面坐标及速度、像素坐标、是否是双目、时间偏移。

`FeaturePerId` 则主要包括 `vector<FeaturePerFrame> feature_per_frame`，以及 `frame_id`、`start_frame` 等路标点个体级别的信息。值得注意的是，vins假设特征点是被连续观测到的，因此只记录开始的帧，即可推断出 vector 内其他观测所属的帧。
- 传入 `start_frame`的变量则是外部的 `frameCnt`，`frameCnt` 指明了滑窗内的哪一帧。

## 主体函数解释

### bool FeatureManager::addFeatureCheckParallax(int , const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> &, double)

输入包括：
- frame_count ：指明当前帧将会是滑窗内的哪一帧
- image：`map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>>` 的数据结构，记录了当前获得的新特征点集的信息，分级索引，分别是 id、相机id，信息则是点的二维几何信息。
- td：与 imu 的时间戳延迟

可分为两大步骤：
- 添加 image 到 `list<FeaturePerId>`类型的数据结构中，并作一些统计
- 判断是否为关键帧

第一步：遍历输入 image 里的点，将其构造成 `FeaturePerFrame` 类型数据结构，判断该点是否被双目观测，若是则将右目信息也初始化进 `FeaturePerFrame`。判断 `list<FeaturePerId>`中是否有当前 image 的 id，不存在则 push 进新 id 并初始化对应的内存，存在则在对应 id 下新增二维几何信息等信息。
```c++
for (auto &id_pts : image)
    {
        FeaturePerFrame f_per_fra(id_pts.second[0].second, td);
        assert(id_pts.second[0].first == 0);
        if(id_pts.second.size() == 2)
        {
            f_per_fra.rightObservation(id_pts.second[1].second);
            assert(id_pts.second[1].first == 1);
        }

        int feature_id = id_pts.first;
        auto it = find_if(feature.begin(), feature.end(), [feature_id](const FeaturePerId &it)
                          {
            return it.feature_id == feature_id;
                          });

        if (it == feature.end())
        {
            feature.push_back(FeaturePerId(feature_id, frame_count));
            feature.back().feature_per_frame.push_back(f_per_fra);
            new_feature_num++;
        }
        else if (it->feature_id == feature_id)
        {
            it->feature_per_frame.push_back(f_per_fra);
            last_track_num++;
            if( it-> feature_per_frame.size() >= 4)
                long_track_num++;
        }
    }
```

第二步：若是窗口内前两帧，则直接视为关键帧；若追踪点数少于20个，视为关键帧；若连续4帧以上追踪的点少于 40 个，也视为关键帧；若新特征点数目大于追踪特征点的一半，亦认为是关键帧。

之后检查第二新和第三新的帧的特征点视差（前提是这个特征点有第二新和第三新的被观测帧），若平均视差大于阈值，则设置为关键帧。

> 20 和 40 是不是该调换一下，目前认为不太合理，因为 long_track_num <= last_track_num 是一定成立的（参考上个代码块的最后几行）。

> 如果面向一面白墙，一帧的特征点追踪个数始终小于阈值，则会被一直判定为关键帧，但这并不是我们的本意。我们的本意是，由于视角移动较大，因此会有一些被追踪的特征点消失在视野内，因此特征点追踪的数目下降，但新增的特征点会多一些。

```c++
    if (frame_count < 2 || 
        last_track_num < 20 || 
        long_track_num < 40 || 
        new_feature_num > 0.5 * last_track_num)
        return true;
    
    for (auto &it_per_id : feature)
    {
        if (it_per_id.start_frame <= frame_count - 2 &&
            it_per_id.start_frame + int(it_per_id.feature_per_frame.size()) - 1 >= frame_count - 1)
        {
            parallax_sum += compensatedParallax2(it_per_id, frame_count);
            parallax_num++;
        }
    }

    if (parallax_num == 0)
    {
        return true;
    }
    else
    {
        ROS_DEBUG("parallax_sum: %lf, parallax_num: %d", parallax_sum, parallax_num);
        ROS_DEBUG("current parallax: %lf", parallax_sum / parallax_num * FOCAL_LENGTH);
        last_average_parallax = parallax_sum / parallax_num * FOCAL_LENGTH;
        return parallax_sum / parallax_num >= MIN_PARALLAX;
    }
```



