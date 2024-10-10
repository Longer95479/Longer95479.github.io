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

### bool FeatureManager::addFeatureCheckParallax(int frame_count, const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> &image, double td)

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



## void FeatureManager::triangulate(int frameCnt, Vector3d Ps[], Matrix3d Rs[], Vector3d tic[], Matrix3d ric[])

遍历特征点，若已有深度则不需要再进行三角，跳过该点。若无深度，则需要进行三角化，且无论双目还是单目，都使用了点被观测的第一帧坐标。若是单目，且特征点只有一个观测，无法三角化，因此也什么都不执行。

若是双目观测，则直接使用双目特征点三角化，*不考虑追踪的时间长短*。

若是单目观测，且至少被观测两次，则用 start_frame 和 start_frame+1 这两帧的点作三角化（此时这两帧的 $P$ 也都已知了
），*也不考虑追踪的时间长短（追踪连续四帧及以上谓长）*。

代码的核心语句是 `triangulatePoint(leftPose, rightPose, point0, point1, point3d);`，其前后都是由 `imu_T_w` 构造出 `cam_T_w` 作为该函数的输入。

### void FeatureManager::triangulatePoint(Eigen::Matrix<double, 3, 4> &Pose0, Eigen::Matrix<double, 3, 4> &Pose1, Eigen::Vector2d &point0, Eigen::Vector2d &point1, Eigen::Vector3d &point_3d)

齐次坐标表示，等号本质上表示等价，成比例，并不是数值上的相等，因此一个点提供两个约束
$p_i$ 是 $P$ 的行，$P_{3\times 4}$ 是世界系到相机系的投影矩阵，到此函数已经把 2D 特征点表示在 **归一化平面** 了，因此 $P = K[R|t] = [R|t]$。列出投影的齐次方程：
$$
x = PX, x'=P'X
$$

$$
\begin{bmatrix}
x\\
y\\
1
\end{bmatrix}
=
\begin{bmatrix}
p_1\\
p_2\\
p_3\\
\end{bmatrix}
X
,
\begin{bmatrix}
x'\\
y'\\
1
\end{bmatrix}
=
\begin{bmatrix}
p'_1\\
p'_2\\
p'_3\\
\end{bmatrix}
X
$$
得到真正的约束
$$
x = \frac{p_1X}{p_3X},
y = \frac{p_2X}{p_3X}
$$
$$
x' = \frac{p'_1X}{p'_3X},
y' = \frac{p'_2X}{p'_3X}
$$

分母乘过去，合并同类项

$$
(xp_3-p_1)X = 0\\
(yp_3-p_2)X = 0\\
(x'p'_3-p'_1)X = 0\\
(y'p'_3-p'_2)X = 0\\
$$

写成矩阵形式

$$
\begin{bmatrix}
(xp_3-p_1)\\
(yp_3-p_2)\\
(x'p'_3-p'_1)\\
(y'p'_3-p'_2)\\
\end{bmatrix}
X = 0
$$

之后使用 SVD 方法求解最小二乘解，即最小奇异值对应的右奇异向量。

对应的代码如下：

```c++
void FeatureManager::triangulatePoint(Eigen::Matrix<double, 3, 4> &Pose0, Eigen::Matrix<double, 3, 4> &Pose1,
                        Eigen::Vector2d &point0, Eigen::Vector2d &point1, Eigen::Vector3d &point_3d)
{
    Eigen::Matrix4d design_matrix = Eigen::Matrix4d::Zero();
    design_matrix.row(0) = point0[0] * Pose0.row(2) - Pose0.row(0);
    design_matrix.row(1) = point0[1] * Pose0.row(2) - Pose0.row(1);
    design_matrix.row(2) = point1[0] * Pose1.row(2) - Pose1.row(0);
    design_matrix.row(3) = point1[1] * Pose1.row(2) - Pose1.row(1);
    Eigen::Vector4d triangulated_point;
    triangulated_point =
              design_matrix.jacobiSvd(Eigen::ComputeFullV).matrixV().rightCols<1>();
    point_3d(0) = triangulated_point(0) / triangulated_point(3);
    point_3d(1) = triangulated_point(1) / triangulated_point(3);
    point_3d(2) = triangulated_point(2) / triangulated_point(3);
}
```

## void FeatureManager::initFramePoseByPnP(int frameCnt, Vector3d Ps[], Matrix3d Rs[], Vector3d tic[], Matrix3d ric[])

首先确保当前帧不是滑窗内的第一帧，因为单目的话，第一帧无法三角化点，也就无法 pnp 了。之后遍历所有特征点 id，选取有深度的点（已被三角化），并判断这个点的被观测帧是否包含当前帧，如果包含，则将其 3D 坐标由世界系转化到相机系，并分别将点在当前帧的 2D 坐标和 3D 坐标放入 vector 容器内。

之后将这些 2D-3D 点对传入函数 `if(solvePoseByPnP(RCam, PCam, pts2D, pts3D))`，点对数不够则 pnp 失败，返回 false，成功则放回 true，`Rcam` 和 `PCam` 会被赋值，表示世界系到相机系的变换，需要再作处理，得到机体系到世界系的变换矩阵。并且 pnp 求解过程会用到 R、t 的初始值，由 imu 提供。

### bool FeatureManager::solvePoseByPnP(Eigen::Matrix3d &R, Eigen::Vector3d &P,  vector<cv::Point2f> &pts2D, vector<cv::Point3f> &pts3D)

核心语句是 `pnp_succ = cv::solvePnP(pts3D, pts2D, K, D, rvec, t, 1);`，其前后都是做一些合法判断、矩阵的变换。需要做一些矩阵变换的原因是，最外部输入和最终的输出，均是 w_T_cam（机体坐标相对于世界坐标，cam2w），而 opencv 的 pnp 函数需要的位姿代表的是 cam_T_w。

```c++
bool FeatureManager::solvePoseByPnP(Eigen::Matrix3d &R, Eigen::Vector3d &P, 
                                      vector<cv::Point2f> &pts2D, vector<cv::Point3f> &pts3D)
{
    Eigen::Matrix3d R_initial;
    Eigen::Vector3d P_initial;

    // w_T_cam ---> cam_T_w 
    R_initial = R.inverse();
    P_initial = -(R_initial * P);

    //printf("pnp size %d \n",(int)pts2D.size() );
    if (int(pts2D.size()) < 4)
    {
        printf("feature tracking not enough, please slowly move you device! \n");
        return false;
    }
    cv::Mat r, rvec, t, D, tmp_r;
    cv::eigen2cv(R_initial, tmp_r);
    cv::Rodrigues(tmp_r, rvec);
    cv::eigen2cv(P_initial, t);
    cv::Mat K = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);  
    bool pnp_succ;
    pnp_succ = cv::solvePnP(pts3D, pts2D, K, D, rvec, t, 1);
    //pnp_succ = solvePnPRansac(pts3D, pts2D, K, D, rvec, t, true, 100, 8.0 / focalLength, 0.99, inliers);

    if(!pnp_succ)
    {
        printf("pnp failed ! \n");
        return false;
    }
    cv::Rodrigues(rvec, r);
    //cout << "r " << endl << r << endl;
    Eigen::MatrixXd R_pnp;
    cv::cv2eigen(r, R_pnp);
    Eigen::MatrixXd T_pnp;
    cv::cv2eigen(t, T_pnp);

    // cam_T_w ---> w_T_cam
    R = R_pnp.transpose();
    P = R * (-T_pnp);

    return true;
}
```




