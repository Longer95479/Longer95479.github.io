---
layout: post
title:  "[VINS-Fusion] Process Measurements"
date:   2024-09-16 23:37:00 +0800
tags: 
    - slam
categories:
    - fast-drone
---

## void Estimator::processMeasurements()

这个函数单独在一个线程内运行，无限循环，间隔2ms。

提取出各帧的特征点并收到了一系列的 IMU 数据后，就该开始处理这些数据了。

首先判断 `featureBuf` 队列是否为空，如果是空的，就什么也不做。若不空，则从 `featureBuf` 里提取出最早的特征点数据帧：

```c++
void Estimator::processMeasurements()
{
    while (1)
    {
        //printf("process measurments\n");
        TicToc t_process;
        pair<double, map<int, vector<pair<int, Eigen::Matrix<double, 7, 1> > > > > feature;
        vector<pair<double, Eigen::Vector3d>> accVector, gyrVector;
        if(!featureBuf.empty())
        {
            feature = featureBuf.front();
            curTime = feature.first + td;
```

之后如果使用 IMU 且 IMU 数据缓存不够新，则无限循环 “wait for imu ...”，否则（不使用 IMU 或 IMU 数据足够新）跳出循环，进行下一步。

```c++
while(1)
{
    if ((!USE_IMU  || IMUAvailable(feature.first + td)))
        break;
    else
    {
        printf("wait for imu ... \n");
        if (! MULTIPLE_THREAD)
            return;
        std::chrono::milliseconds dura(5);
        std::this_thread::sleep_for(dura);
    }
}
```

如果使用 IMU，则从 `accBuf` 和 `gyrBuf` 里提取前一帧和当前帧之前的 IMU 数据到 `accVector` 和 `gyrVector` 里：

```c++
mBuf.lock();
if(USE_IMU)
    getIMUInterval(prevTime, curTime, accVector, gyrVector);

featureBuf.pop();
mBuf.unlock();
```

如果使用 IMU，且还未初始化第一帧位姿，则利用加速度计的读数来初始化位姿，并强制 yaw 为 0。如果已经初始化第一帧位姿，则进入 IMU 真正的后端 `processIMU()` 里面包含预积分和将因子送入因子图。积分离散化使用 **零阶反向**。

```c++
if(USE_IMU)
{
    if(!initFirstPoseFlag)
        initFirstIMUPose(accVector);
    for(size_t i = 0; i < accVector.size(); i++)
    {
        double dt;
        if(i == 0)
            dt = accVector[i].first - prevTime;
        else if (i == accVector.size() - 1)
            dt = curTime - accVector[i - 1].first;
        else
            dt = accVector[i].first - accVector[i - 1].first;
        processIMU(accVector[i].first, dt, accVector[i].second, gyrVector[i].second);
    }
}
```
##  void Estimator::processIMU()

接下来进入 `processIMU()` 的内部。首先是判断是否收到过第一帧 IMU 数据，如果没收到，则

```c++
void Estimator::processIMU(double t, double dt, const Vector3d &linear_acceleration, const Vector3d &angular_velocity)
{
    if (!first_imu)
    {
        first_imu = true;
        acc_0 = linear_acceleration;
        gyr_0 = angular_velocity;
    }
```

介绍一下 `IntegrationBase *pre_integrations[(WINDOW_SIZE + 1)]`：
- IntegrationBase *：表示一个指向 IntegrationBase 类对象的指针。
-  pre_integrations：是一个数组的名称。
-  [(WINDOW_SIZE + 1)]：定义了数组的大小，即数组有 WINDOW_SIZE + 1 个元素。

如果  `!pre_integrations[frame_count]` 为空指针，则将 IMU 数据初始化到其中：

```c++
    if (!pre_integrations[frame_count])
    {
        pre_integrations[frame_count] = new IntegrationBase{acc_0, gyr_0, Bas[frame_count], Bgs[frame_count]};
    }
```

如果不是滑窗内的第一帧，则可以处理当前帧和上一帧之间的 IMU 数据，具体则是将当前的 imu 数据和上一帧 IMU 数据的时间差、加速度、角速度传入 `pre_integrations` 和 buf 中（将 IMU 数据缓存到这些ector中的作用是什么呢？）。之后**利用 IMU 数据 持续预测当前的姿态，最终将预测出当前特征帧的姿态**。最后保存记录 `acc_0` 和 `gyr_0`。

```c++
if (frame_count != 0)
    {
        pre_integrations[frame_count]->push_back(dt, linear_acceleration, angular_velocity);
        //if(solver_flag != NON_LINEAR)
            tmp_pre_integration->push_back(dt, linear_acceleration, angular_velocity);

        dt_buf[frame_count].push_back(dt);
        linear_acceleration_buf[frame_count].push_back(linear_acceleration);
        angular_velocity_buf[frame_count].push_back(angular_velocity);

        int j = frame_count;         
        Vector3d un_acc_0 = Rs[j] * (acc_0 - Bas[j]) - g;
        Vector3d un_gyr = 0.5 * (gyr_0 + angular_velocity) - Bgs[j];
        Rs[j] *= Utility::deltaQ(un_gyr * dt).toRotationMatrix();
        Vector3d un_acc_1 = Rs[j] * (linear_acceleration - Bas[j]) - g;
        Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);
        Ps[j] += dt * Vs[j] + 0.5 * dt * dt * un_acc;
        Vs[j] += dt * un_acc;
    }
    acc_0 = linear_acceleration;
    gyr_0 = angular_velocity; 
}
```

## void Estimator::processImage()

执行完 `processIMU()` 后便是执行最重要的 `processImage()`。以下是 gpt 的回答：

>这段代码是一个名为 `processImage` 的函数，属于 `Estimator` 类，主要用于处理视觉惯性导航系统（VINS）中的新图像数据。这段代码中，系统接收图像特征数据，并结合惯性测量单元（IMU）的信息，执行关键帧管理、初始化、非线性优化、滑动窗口、故障检测等一系列操作。以下是代码的总结与主要步骤解释：
>
>1. **图像处理和特征点添加**:
>       - 函数接收图像特征数据 `image` 和时间戳 `header`。
>       - 记录图像特征点的数量，并将其添加到特征管理器中进行视差检查，决定是否将当前帧标记为关键帧。
>       - 关键帧管理：基于特征的视差决定是否进行边缘化操作。
>   
>2. **初始化图像帧和预积分**:
>       - 创建 `ImageFrame` 对象并设置其预积分信息。
>       - 将当前帧添加到 `all_image_frame` 映射中。
>       - 初始化或更新预积分对象 `tmp_pre_integration`。
>
>3. **外参估计**:
>       - 如果需要估计外参，系统在足够旋转运动时通过双目视差数据计算外参，并更新外参矩阵 >`ric`。
>
>4. **初始化阶段**:
>       - **单目 + IMU 初始化**: 当使用单目相机和IMU时，系统会在时间窗口填满时尝试初>始化。如果初始化成功，系统进入非线性优化阶段，并执行滑动窗口操作。
>       - **双目 + IMU 初始化**: 对于双目相机和IMU的初始化，系统会通过PnP方法初始化帧>姿态，进行特征点三角化，并解决陀螺仪偏置问题。
>       - **仅双目初始化**: 不使用IMU时，系统通过双目视差完成初始化，并在滑动窗口填满后>进入非线性优化。
>
>5. **优化与滑动窗口管理**:
>       - 进行非线性优化，并检查和移除异常特征点。
>       - 对于未使用多线程的情况，特征跟踪器将进一步移除异常点并预测下一个帧中的特征位置。
>       - 检测系统故障，若发生故障，清除系统状态并重新启动。
>
>6. **状态更新**:
>       - 更新系统状态，包括关键帧姿态、最后的位姿和状态参数等。
>
>总的来说，这段代码的主要作用是结合图像和IMU数据进行视觉惯性导航系统的初始化、优化和状态更新，从而提高系统的定位精度和鲁棒性。

接下来进行人为总结。

通过检测视差来判断最新到达的图像帧是不是关键帧，如果是，后续的边缘化则将滑窗内最旧的帧边缘化，如果不是，则丢弃第二新的帧。

```c++
void Estimator::processImage(const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> &image, const double header)
{
    ROS_DEBUG("new image coming ------------------------------------------");
    ROS_DEBUG("Adding feature points %lu", image.size());
    if (f_manager.addFeatureCheckParallax(frame_count, image, td))
    {
        marginalization_flag = MARGIN_OLD;
        //printf("keyframe\n");
    }
    else
    {
        marginalization_flag = MARGIN_SECOND_NEW;
        //printf("non-keyframe\n");
    }

```

之后打印一些 debug 信息，如当前帧是否是关键帧、当前帧属于滑窗内第几帧、特征点数量。

创建 `ImageFrame imageframe(image, header);` 变量，并进行初始化，使其包含了当前帧的特征点、时间戳、是否是关键帧、姿态、位置和其和上一帧之间的 IMU 预积分数据，其定义如下：

```c++
class ImageFrame
{
    public:
        ImageFrame(){};
        ImageFrame(const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>>& _points, double _t):t{_t},is_key_frame{false}
        {
            points = _points;
        };
        map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>> > > points;
        double t;
        Matrix3d R;
        Vector3d T;
        IntegrationBase *pre_integration;
        bool is_key_frame;
};
```

将创建好的 `imageframe` 放入 `all_image_frame` 的 map 之中，以时间戳为键。然后重新初始化 `tmp_pre_integration`，为下一关键帧的预积分作准备。

```c++
    ROS_DEBUG("%s", marginalization_flag ? "Non-keyframe" : "Keyframe");
    ROS_DEBUG("Solving %d", frame_count);
    ROS_DEBUG("number of feature: %d", f_manager.getFeatureCount());
    Headers[frame_count] = header;

    ImageFrame imageframe(image, header);
    imageframe.pre_integration = tmp_pre_integration;
    all_image_frame.insert(make_pair(header, imageframe));
    tmp_pre_integration = new IntegrationBase{acc_0, gyr_0, Bas[frame_count], Bgs[frame_count]};
```

>  Eigen::Matrix3d 使用 `swap()` 方法可能比直接赋值更快，用在 `void Estimator::slide()` 内。


相机外参在线校准：

```c++
 if(ESTIMATE_EXTRINSIC == 2)
    {
        ROS_INFO("calibrating extrinsic param, rotation movement is needed");
        if (frame_count != 0)
        {
            vector<pair<Vector3d, Vector3d>> corres = f_manager.getCorresponding(frame_count - 1, frame_count);
            Matrix3d calib_ric;
            if (initial_ex_rotation.CalibrationExRotation(corres, pre_integrations[frame_count]->delta_q, calib_ric))
            {
                ROS_WARN("initial extrinsic rotation calib success");
                ROS_WARN_STREAM("initial extrinsic rotation: " << endl << calib_ric);
                ric[0] = calib_ric;
                RIC[0] = calib_ric;
                ESTIMATE_EXTRINSIC = 1;
            }
        }
    }
```

### 初始化

外参在线校准完成后，则进行初始化阶段，可以分为三类：
- 单目 + IMU
- 双目 + IMU
- 双目

首先讨论单目 + IMU 的情况。如果滑窗没满，则不做任何操作。如果滑窗已满，则判断 *外参是否已估计 以及 当前帧距离第一帧是否大于 0.1 s*，若是，则进行 `initialStructure()`，当初始化成功时，将进入**优化与滑窗**的状态，否则只进行滑窗且保持待初始化的状态。

```c++
   if (solver_flag == INITIAL)
    {
        // monocular + IMU initilization
        if (!STEREO && USE_IMU)
        {
            if (frame_count == WINDOW_SIZE)
            {
                bool result = false;
                if(ESTIMATE_EXTRINSIC != 2 && (header - initial_timestamp) > 0.1)
                {
                    result = initialStructure();
                    initial_timestamp = header;   
                }
                if(result)
                {
                    solver_flag = NON_LINEAR;
                    optimization();
                    slideWindow();
                    ROS_INFO("Initialization finish!");
                }
                else
                    slideWindow();
            }
        }

```

如果是双目 + IMU 的初始化，则直接使用 pnp 初始化当前帧的 P、R，然后利用该P、R和其他帧（哪一帧呢）三角化当前帧的特征点。

如果滑窗已经满了，则将滑窗内的 P、R 更新给 `all_image_frame`，之后使用这些位姿来求解陀螺仪的偏置初始值。然后利用陀螺仪偏置的最新初值，来更新滑窗内的各个预积分值。

最后进入优化滑窗的状态。

**NOTE**：
-  在最开始的阶段，所有特征点都没有深度，pnp 函数实际什么也没执行，因此没完成对当前帧（第一帧，frame_count = 0）的位姿初始化

- 顺序执行到三角化函数，此时 Ps=0 和 Rs=I 都是初始值，因为是双目，因此可利用左右目“瞬间”完成特征点的三角化（“瞬间”是指只使用同一时刻的左右帧即可完成三角化）。

- 此时 frame_count != WINDOW_SIZE，if 内的代码均不执行

- 第二帧到来，imu 预积分预测出 Rs[1]、Ps[1] 作为 pnp 初值，并使用跟踪的已三角化的点完成 pnp 位姿初始化

- 之后用刚计算出的位姿对新检测（未三角化，无深度）的点进行三角化

- 反复执行


```c++
// stereo + IMU initilization
if(STEREO && USE_IMU)
{
    f_manager.initFramePoseByPnP(frame_count, Ps, Rs, tic, ric);
    f_manager.triangulate(frame_count, Ps, Rs, tic, ric);
    if (frame_count == WINDOW_SIZE)
    {
        map<double, ImageFrame>::iterator frame_it;
        int i = 0;
        for (frame_it = all_image_frame.begin(); frame_it != all_image_frame.end(); frame_it++)
        {
            frame_it->second.R = Rs[i];
            frame_it->second.T = Ps[i];
            i++;
        }
        solveGyroscopeBias(all_image_frame, Bgs);
        for (int i = 0; i <= WINDOW_SIZE; i++)
        {
            pre_integrations[i]->repropagate(Vector3d::Zero(), Bgs[i]);
        }
        solver_flag = NON_LINEAR;
        optimization();
        slideWindow();
        ROS_INFO("Initialization finish!");
    }
}
```

如果是只使用双目，不使用 IMU的情况，到来一帧，无论窗口满或未满，使用 pnp 求解当前帧的初始位姿，**然后就直接优化，与前二者窗口满了才优化不同**，如果窗口满了，才进行滑窗，并进入优化滑窗状态。

```c++
if(STEREO && !USE_IMU)
{
    f_manager.initFramePoseByPnP(frame_count, Ps, Rs, tic, ric);
    f_manager.triangulate(frame_count, Ps, Rs, tic, ric);
    optimization();

    if(frame_count == WINDOW_SIZE)
    {
        solver_flag = NON_LINEAR;
        slideWindow();
        ROS_INFO("Initialization finish!");
    }
}
```

在初始化阶段，无论是否为关键帧，均放入滑窗内。如果窗口还没满，则将当前的 P、V、R、B 传给下一帧作为初值，之后会在 `processIMU()` 累加。

```c++
if(frame_count < WINDOW_SIZE)
{
    frame_count++;
    int prev_frame = frame_count - 1;
    Ps[frame_count] = Ps[prev_frame];
    Vs[frame_count] = Vs[prev_frame];
    Rs[frame_count] = Rs[prev_frame];
    Bas[frame_count] = Bas[prev_frame];
    Bgs[frame_count] = Bgs[prev_frame];
}
```

### 滑窗与优化

如果初始化完成，后续该函数将重复执行滑窗与优化。此时 `frame_count` 将不会再改变，始终等于 `WINDOW_SIZE`，代码中的窗口大小实际为 `WINDOW_SIZE + 1`。

 1. 首先，如果不使用 IMU，则需要 pnp 求解出新到达帧的初始位姿，否则无需此步骤，因为 IMU 预积分会给出新到达帧的位姿预测。

2.  使用窗口里的历史位姿和刚得到的初始位姿，对还没有深度的点三角化。

3. `optimization();`，进行优化，细节见下文

4. 外点去除。先是得到需要去除外点的 id 集合，之后在已有的 fearture_manage 和 feature_tracker 的点集里删去这些点
    ```c++
    set<int> removeIndex;
    f_manager.removeOutlier(removeIndex);
    if (! MULTIPLE_THREAD)
    {
        featureTracker.removeOutliers(removeIndex);
        predictPtsInNextFrame();
    }
    ```

5. 失效检测，如果满足一些指标，则判定为里程计失效，重置变量，重启系统

6. `slideWindow();` 滑窗，对表示窗口内状态的几个数组操作，边缘化最早的帧或次新帧

7. 删除优化失败的路标点，即以 id 遍历路标点，如果其 `solve_flag==2` 则删去

8. 最后对一些用于输出的接口变量（latest state）更新


## void Estimator::optimization()

该函数被 `processImage()` 调用，是后端的核心函数。

首先将用 `Eigen` 表示的状态转换成 `double` 类型的数组，用于后续的 ceres 优化。

实例化 `问题` 和 `损失函数`，即 `ceres::Problem problem;` 和 `ceres::LossFunction *loss_function;`。初始化鲁棒优化核函数 Huber 函数。

添加参数块（ParammeterBlock），包括位姿、速度、偏置、外参、时间偏移。步骤如下：

1. 实例化局部参数化变量：`ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();`

2. 向问题添加参数块：`problem.AddParameterBlock(para_Ex_Pose[i], SIZE_POSE, local_parameterization);`

对参数可以有一些额外的操作，如`冻结某些参数`：
```c++
// 若未使用 imu，则不对第一帧的位姿作优化
if(!USE_IMU)
    problem.SetParameterBlockConstant(para_Pose[0]);
```

接下来构建或更新因子图。

如果存在上次边缘化的信息，则构造新的边缘化因子。
```c++
if (last_marginalization_info && last_marginalization_info->valid)
{
    // construct new marginlization_factor
    MarginalizationFactor *marginalization_factor = new MarginalizationFactor(last_marginalization_info);
    problem.AddResidualBlock(marginalization_factor, NULL,
                                last_marginalization_parameter_blocks);
}
```

如果使用 imu，则添加预积分因子：
```c++
if(USE_IMU)
{
    for (int i = 0; i < frame_count; i++)
    {
        int j = i + 1;
        if (pre_integrations[j]->sum_dt > 10.0)
            continue;
        IMUFactor* imu_factor = new IMUFactor(pre_integrations[j]);
        problem.AddResidualBlock(imu_factor, NULL, para_Pose[i], para_SpeedBias[i], para_Pose[j], para_SpeedBias[j]);
    }
}
```

接下来便是添加相机观测相关的因子。遍历 feature_manager 内的路标点，如果路标点被跟踪帧数小于 4 帧，则不参与优化。对于优化的路标点观测，用到了三种组合：

- `ProjectionTwoFrameOneCamFactor`：pts_i, pts_j
- `ProjectionTwoFrameTwoCamFactor`：pts_i, pts_j_right（i != j）
- `ProjectionOneFrameTwoCamFactor`：pts_i, pts_j_right（i = j）

可视化便是星型网，中心是左目的 start_frame，连接所有其他在 `vector<FeaturePerFrame>` 的元素。

> 星型意味着对中心敏感，可改进的点？

此时已经将涉及的不同类别的因子添加到因子图中，开始对 ceres 求解器作一些设置：

```c++
options.linear_solver_type = ceres::DENSE_SCHUR;
//options.num_threads = 2;
options.trust_region_strategy_type = ceres::DOGLEG;
options.max_num_iterations = NUM_ITERATIONS;
//options.use_explicit_schur_complement = true;
//options.minimizer_progress_to_stdout = true;
//options.use_nonmonotonic_steps = true;
if (marginalization_flag == MARGIN_OLD)
    options.max_solver_time_in_seconds = SOLVER_TIME * 4.0 / 5.0;
else
    options.max_solver_time_in_seconds = SOLVER_TIME;
TicToc t_solver;
ceres::Solver::Summary summary;
ceres::Solve(options, &problem, &summary);
```

`double2vector();` 将求解的结果转变为 Eigen 数据结构。

至此，优化计算的部分已经完成。如果此时窗口内的关键帧还未满，则已经可以退出函数了；但若窗口已满，则需要为下一帧的到来预留位置，因此需要执行边缘化。

```c++
if(frame_count < WINDOW_SIZE)
    return;

TicToc t_whole_marginalization;
if (marginalization_flag == MARGIN_OLD)
{
```
    