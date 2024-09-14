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
// vins-mono
class FeatureTracker {
 public:
  vector<int> ids;					// 特征点ID
  vector<cv::Point2f> prev_pts;		// 特征点在前两帧中的坐标
  vector<cv::Point2f> cur_pts;		// 特征点在前一帧中的坐标
  vector<cv::Point2f> forw_pts;		// 特征点在当前帧中的坐标
  vector<int> track_cnt;			// 特征点成功追踪帧数
  vector<cv::Point2f> cur_un_pts;	// 特征点归一化坐标系下的坐标
}

// vins-fusion
class FeatureTracker
{
public:
    vector<cv::Point2f> predict_pts;
    vector<cv::Point2f> predict_pts_debug;
    //特征点在前一帧的坐标、当前左目帧的坐标、当前右目帧的坐标
    vector<cv::Point2f> prev_pts, cur_pts, cur_right_pts;
    // 特征点在前一帧归一化平面中的坐标、当前帧左目、当前帧右目
    vector<cv::Point2f> prev_un_pts, cur_un_pts, cur_un_right_pts; 
    vector<cv::Point2f> pts_velocity, right_pts_velocity;
    vector<int> ids, ids_right;     // 左右目特征点id
    vector<int> track_cnt;
}
```

`reduceVector()` 空间复杂度 O(n)，时间复杂度 O（n）
```c++
void reduceVector(vector<cv::Point2f> &v, vector<uchar> status) {
  int j = 0;
  for (int i = 0; i < int(v.size()); i++)
    if (status[i])
      v[j++] = v[i];
  v.resize(j);
}
```

光流是基于以下假设下工作的： 
1. 在连续的两帧图像之间，目标对象的像素的灰度值不改变。 
2. 像素的运动比较微小。
3. 相邻像素具有相似的运动。

```c++
 cv::calcOpticalFlowPyrLK(prev_img, cur_img, prev_pts, cur_pts, status, err, cv::Size(21, 21), 1, cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 30, 0.01), cv::OPTFLOW_USE_INITIAL_FLOW);
```

>现在我们要使用第三条假设，邻域内的所有点都有相似的运动。LucasKanade 法就是利用一个 3x3 邻域中的 9 个点具有相同运动的这一点。这样我们就可以找到$$ (f_x, f_y, f_t) $$这 9 个点的光流方程，用它们组成一个具有两个未知数 9 个等式的方程组，这是一个约束条件过多的方程组。一个好的解决方法就是使用最小二乘拟合。下面就是求解结果：
>$$ 
\begin{bmatrix} u \ v \end{bmatrix} = \begin{bmatrix} \sum_{i}{f_{x_i}}^2 & \sum_{i}{f_{x_i} f_{y_i} } \ \sum_{i}{f_{x_i} f_{y_i}} & \sum_{i}{f_{y_i}}^2 \end{bmatrix}^{-1} \begin{bmatrix} - \sum_{i}{f_{x_i} f_{t_i}} \ - \sum_{i}{f_{y_i} f_{t_i}} \end{bmatrix} 
$$


>你会发现上边的逆矩阵与 Harris 角点检测器非常相似，这说明角点很适合被用来做跟踪） 从使用者的角度来看，想法很简单，我们去跟踪一些点，然后我们就会获得这些点的光流向量。但是还有一些问题。直到现在我们处理的都是很小的运动。如果有大的运动怎么办呢？图像金字塔。当我们进入金字塔时，小运动被移除，大运动变成小运动。因此，通过在那里应用Lucas-Kanade，我们就会得到尺度空间上的光流。<br>
> [6.2. 光流](https://opencv-python-tutorials.readthedocs.io/zh/latest/6.%20%E8%A7%86%E9%A2%91%E5%88%86%E6%9E%90/6.2.%20%E5%85%89%E6%B5%81/)

>金字塔Lucas-Kanade跟踪方法是：在图像金字塔的最高层计算光流，用得到的运动估计结果作为下一层金字塔的起始点，重复这个过程直到到达金字塔的最底层。这样就将不满足运动的假设可能性降到最小从而实现对更快和更长的运动的跟踪。