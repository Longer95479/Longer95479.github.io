---
layout: post
title:  "[VINS-Fusion] Feature Tracker"
date:   2024-09-14 00:52:00 +0800
tags: 
    - slam
categories:
    - fast-drone
---

来自相机的图像被发布，缓存在 `img0_buf` 和 `img1_buf` 队列中，新图像放在 back，抽取 front 的图像进行特征追踪。如果是双目，不同步的会被按时间戳先后丢弃，只有双目图像帧的时间戳完全同步才会被使用。

`InputImage()` 和 `InputIMU()` 是前端。其实只有 `InputImage()` 是真正的前端，IMU的预积分其实是放在 `processIMU()` 里面。


一些重要的变量：

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
 cv::calcOpticalFlowPyrLK(prev_img, 
                          cur_img, 
                          prev_pts, 
                          cur_pts, 
                          status, 
                          err, 
                          cv::Size(21, 21), 
                          1, 
                          cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 30, 0.01), 
                          cv::OPTFLOW_USE_INITIAL_FLOW);
```

>现在我们要使用第三条假设，邻域内的所有点都有相似的运动。LucasKanade 法就是利用一个 3x3 邻域中的 9 个点具有相同运动的这一点。这样我们就可以找到$$ (f_x, f_y, f_t) $$这 9 个点的光流方程，用它们组成一个具有两个未知数 9 个等式的方程组，这是一个约束条件过多的方程组。一个好的解决方法就是使用最小二乘拟合。下面就是求解结果：
>$$ 
\begin{bmatrix} u \ v \end{bmatrix} = \begin{bmatrix} \sum_{i}{f_{x_i}}^2 & \sum_{i}{f_{x_i} f_{y_i} } \ \sum_{i}{f_{x_i} f_{y_i}} & \sum_{i}{f_{y_i}}^2 \end{bmatrix}^{-1} \begin{bmatrix} - \sum_{i}{f_{x_i} f_{t_i}} \ - \sum_{i}{f_{y_i} f_{t_i}} \end{bmatrix} 
$$


>你会发现上边的逆矩阵与 Harris 角点检测器非常相似，这说明角点很适合被用来做跟踪） 从使用者的角度来看，想法很简单，我们去跟踪一些点，然后我们就会获得这些点的光流向量。但是还有一些问题。直到现在我们处理的都是很小的运动。如果有大的运动怎么办呢？图像金字塔。当我们进入金字塔时，小运动被移除，大运动变成小运动。因此，通过在那里应用Lucas-Kanade，我们就会得到尺度空间上的光流。<br>
> [6.2. 光流](https://opencv-python-tutorials.readthedocs.io/zh/latest/6.%20%E8%A7%86%E9%A2%91%E5%88%86%E6%9E%90/6.2.%20%E5%85%89%E6%B5%81/)<br>
[zhihu -（三十八）稀疏光流----KLT](https://zhuanlan.zhihu.com/p/88033287)

>金字塔Lucas-Kanade跟踪方法是：在图像金字塔的最高层计算光流，用得到的运动估计结果作为下一层金字塔的起始点，重复这个过程直到到达金字塔的最底层。这样就将不满足运动的假设可能性降到最小从而实现对更快和更长的运动的跟踪。

## FeatureTracker::trackImage() 解读

```c++
map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> 
FeatureTracker::trackImage(double _cur_time, const cv::Mat &_img, const cv::Mat &_img1)
```

输入是当前帧的时间、左目图像帧、右目图像帧

函数开头定义了一些局部变量，并清空用于存储当前帧特征点像素坐标的 vector：
```c++
TicToc t_r;
cur_time = _cur_time;
cur_img = _img;
row = cur_img.rows;
col = cur_img.cols;
cv::Mat rightImg = _img1;

cur_pts.clear();
```


首先判断上一帧的特征点个数是否大于 0，如果大于 0，则进行特征点的跟踪，否则什么也不做。

若进行特征点的跟踪，首先判断是否使用 gpu 加速，之后无论是否使用 gpu 加速，均判断是否使用预测特征点和是否使用反向光流跟踪
- 如果使用了预测，则利用预测的先验值来帮助LK光流跟踪，如果没有使用预测，则使用普通的三层金字塔LK光流跟踪。
- 如果使用了反向光流跟踪，正反向都能跟踪上且反向跟踪的点和原始点的误差小于 0.5 像素的点会被保留

而对于使用 gpu 加速的情况，需要将对应的图像矩阵变量和光流跟踪函数替换成对应的 cuda 版本：

```c++
cv::cuda::GpuMat prev_gpu_img(prev_img);
cv::cuda::GpuMat cur_gpu_img(cur_img);
cv::cuda::GpuMat prev_gpu_pts(prev_pts);
cv::cuda::GpuMat cur_gpu_pts(cur_pts);
cv::cuda::GpuMat gpu_status;

// flow forward gpu version
cv::Ptr<cv::cuda::SparsePyrLKOpticalFlow> d_pyrLK_sparse = cv::cuda::SparsePyrLKOpticalFlow::create(
cv::Size(21, 21), 3, 30, false);
d_pyrLK_sparse->calc(prev_gpu_img, cur_gpu_img, prev_gpu_pts, cur_gpu_pts, gpu_status);

vector<cv::Point2f> tmp1_cur_pts(cur_gpu_pts.cols);
cur_gpu_pts.download(tmp1_cur_pts);
cur_pts = tmp1_cur_pts;

vector<uchar> tmp1_status(gpu_status.cols);
gpu_status.download(tmp1_status);
status = tmp1_status;

```

之后对剩下的点作最后一层筛选，即判断当前帧的特征点是否在边界内，不在的将 state 设为 0：

```c++
for (int i = 0; i < int(cur_pts.size()); i++)
    if (status[i] && !inBorder(cur_pts[i]))
        status[i] = 0;
reduceVector(prev_pts, status);
reduceVector(cur_pts, status);
reduceVector(ids, status);
reduceVector(track_cnt, status);
```

然后将最终跟踪下来的特征点的 cnt 加一：
```c++
for (auto &n : track_cnt)
    n++;
```

`void FeatureTracker::setMask()` 内将 cnt、pts、ids 合并成 vector，之后按 cnt 的大小进行排序，然后利用 mask 实现均匀化，也就是反复仿佛雨滴首先滴在 cnt 大的点上，其他点如果在这个雨滴内，就不会被放入 vector 里了，从而会删去一些点，这样也能够保证 cnt 大的特征点优先保留下来：

```c++
void FeatureTracker::setMask()
{
    mask = cv::Mat(row, col, CV_8UC1, cv::Scalar(255));

    // prefer to keep features that are tracked for long time
    vector<pair<int, pair<cv::Point2f, int>>> cnt_pts_id;

    for (unsigned int i = 0; i < cur_pts.size(); i++)
        cnt_pts_id.push_back(make_pair(track_cnt[i], make_pair(cur_pts[i], ids[i])));

    sort(cnt_pts_id.begin(), cnt_pts_id.end(), [](const pair<int, pair<cv::Point2f, int>> &a, const pair<int, pair<cv::Point2f, int>> &b)
         {
            return a.first > b.first;
         });

    cur_pts.clear();
    ids.clear();
    track_cnt.clear();

    for (auto &it : cnt_pts_id)
    {
        if (mask.at<uchar>(it.second.first) == 255)
        {
            cur_pts.push_back(it.second.first);
            ids.push_back(it.second.second);
            track_cnt.push_back(it.first);
            cv::circle(mask, it.second.first, MIN_DIST, 0, -1);
        }
    }
}
```

将跟踪到的点均匀化并设置 mask 后，如果当前特征点个数小于 **最大跟踪数目**，则检测新的 goodfeature，检测也可以分成使用 gpu 加速和不使用 gpu 加速的。检测出新的点后，坐标都只是先存在 `vector<cv::Point2f> n_pts` 里，需要将其添加到 `cur_pts`、`ids`、`track_cnt` 里，ids就是总的ids记录上累加 1，track_cnt 均等于 1：

**NOTE: 若要加协调器，分配的最大检测特征点数目就对应于此**

```c++
int n_max_cnt = MAX_CNT - static_cast<int>(cur_pts.size());
        if(!USE_GPU)
        {
            if (n_max_cnt > 0)
            {
                TicToc t_t;
                if(mask.empty())
                    cout << "mask is empty " << endl;
                if (mask.type() != CV_8UC1)
                    cout << "mask type wrong " << endl;
                cv::goodFeaturesToTrack(cur_img, n_pts, MAX_CNT - cur_pts.size(), 0.01, MIN_DIST, mask);
                // printf("good feature to track costs: %fms\n", t_t.toc());
                std::cout << "n_pts size: "<< n_pts.size()<<std::endl;
            }
            else
                n_pts.clear();
            // sum_n += n_pts.size();
            // printf("total point from non-gpu: %d\n",sum_n);
        }
        
        // ROS_DEBUG("detect feature costs: %fms", t_t.toc());
        // printf("good feature to track costs: %fms\n", t_t.toc());
        else
        {
            if (n_max_cnt > 0)
            {
                if(mask.empty())
                    cout << "mask is empty " << endl;
                if (mask.type() != CV_8UC1)
                    cout << "mask type wrong " << endl;
                TicToc t_g;
                cv::cuda::GpuMat cur_gpu_img(cur_img);
                cv::cuda::GpuMat d_prevPts;
                TicToc t_gg;
                cv::cuda::GpuMat gpu_mask(mask);
                // printf("gpumat cost: %fms\n",t_gg.toc());
                cv::Ptr<cv::cuda::CornersDetector> detector = cv::cuda::createGoodFeaturesToTrackDetector(cur_gpu_img.type(), MAX_CNT - cur_pts.size(), 0.01, MIN_DIST);
                // cout << "new gpu points: "<< MAX_CNT - cur_pts.size()<<endl;
                detector->detect(cur_gpu_img, d_prevPts, gpu_mask);
                // std::cout << "d_prevPts size: "<< d_prevPts.size()<<std::endl;
                if(!d_prevPts.empty())
                    n_pts = cv::Mat_<cv::Point2f>(cv::Mat(d_prevPts));
                else
                    n_pts.clear();
                // sum_n += n_pts.size();
                // printf("total point from gpu: %d\n",sum_n);
                // printf("gpu good feature to track cost: %fms\n", t_g.toc());
            }
            else 
                n_pts.clear();
        }

        ROS_DEBUG("add feature begins");
        TicToc t_a;
        addPoints();
        // ROS_DEBUG("selectFeature costs: %fms", t_a.toc());
        // printf("selectFeature costs: %fms\n", t_a.toc());
    }
```
```c++
void FeatureTracker::addPoints()
{
    for (auto &p : n_pts)
    {
        cur_pts.push_back(p);
        ids.push_back(n_id++);
        track_cnt.push_back(1);
    }
}
```




---

相机的参数传递流：

rosNodeTest.cpp
```c++
string config_file = argv[1];
printf("config_file: %s\n", argv[1]);

readParameters(config_file);
estimator.setParameter();
```

parameters.cpp
```c++
int pn = config_file.find_last_of('/');
std::string configPath = config_file.substr(0, pn);

std::string cam0Calib;
fsSettings["cam0_calib"] >> cam0Calib;
std::string cam0Path = configPath + "/" + cam0Calib;
CAM_NAMES.push_back(cam0Path);
```

estimator.cpp

```c++
void Estimator::setParameter()
{
    ···
    featureTracker.readIntrinsicParameter(CAM_NAMES);
    ···
}
```


feature_tracker.cpp

```c++
void FeatureTracker::readIntrinsicParameter(const vector<string> &calib_file)
{
    for (size_t i = 0; i < calib_file.size(); i++)
    {
        ROS_INFO("reading paramerter of camera %s", calib_file[i].c_str());
        camodocal::CameraPtr camera = CameraFactory::instance()->generateCameraFromYamlFile(calib_file[i]);
        m_camera.push_back(camera);
    }
    if (calib_file.size() == 2)
        stereo_cam = 1;
}
```

对剩余的点去畸变，抬升到归一化平面，并计算在该平面上的速度：
```c++
cur_un_pts = undistortedPts(cur_pts, m_camera[0]);
pts_velocity = ptsVelocity(ids, cur_un_pts, cur_un_pts_map, prev_un_pts_map);
```

若有右目，则将到此为止的左目特征点跟踪到右目，并且 id 和坐标单独记录，不记录跟踪帧数，因为右目的特征点由已均匀化的左目特征点跟踪而来，因此没必要均匀化了。


最后则是将筛选后幸存下来的特征点存储在 `featureFrame` 里。

```c++
map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> featureFrame;
for (size_t i = 0; i < ids.size(); i++)
{
    int feature_id = ids[i];
    double x, y ,z;
    x = cur_un_pts[i].x;
    y = cur_un_pts[i].y;
    z = 1;
    double p_u, p_v;
    p_u = cur_pts[i].x;
    p_v = cur_pts[i].y;
    int camera_id = 0;
    double velocity_x, velocity_y;
    velocity_x = pts_velocity[i].x;
    velocity_y = pts_velocity[i].y;

    Eigen::Matrix<double, 7, 1> xyz_uv_velocity;
    xyz_uv_velocity << x, y, z, p_u, p_v, velocity_x, velocity_y;
    featureFrame[feature_id].emplace_back(camera_id,  xyz_uv_velocity);
}
```