---
layout: post
title:  "[VINS-Fusion] Process Measurements"
date:   2024-09-16 23:37:00 +0800
tags: 
    - slam
categories:
    - fast-drone
---


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

如果使用 IMU，且还未初始化第一帧位姿，则利用加速度计的读数来初始化位姿，并强制 yaw 为 0。如果已经初始化第一帧位姿，则进入 IMU 真正的后端 `processIMU()` 里面包含预积分和将因子送入因子图。

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


