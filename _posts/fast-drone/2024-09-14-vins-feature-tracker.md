---
layout: post
title:  "VINS-Fusion Feature Tracker"
date:   2024-09-14 00:52:00 +0800
tags: 
    - slam
categories:
    - fast-drone
---


```c++
class FeatureTracker {
 public:
  vector<int> ids;					// 特征点ID
  vector<cv::Point2f> prev_pts;		// 特征点在前两帧中的坐标
  vector<cv::Point2f> cur_pts;		// 特征点在前一帧中的坐标
  vector<cv::Point2f> forw_pts;		// 特征点在当前帧中的坐标
  vector<int> track_cnt;			// 特征点成功追踪帧数
  vector<cv::Point2f> cur_un_pts;	// 特征点归一化坐标系下的坐标
}
```

```c++
void reduceVector(vector<cv::Point2f> &v, vector<uchar> status) {
  int j = 0;
  for (int i = 0; i < int(v.size()); i++)
    if (status[i])
      v[j++] = v[i];
  v.resize(j);
}
```