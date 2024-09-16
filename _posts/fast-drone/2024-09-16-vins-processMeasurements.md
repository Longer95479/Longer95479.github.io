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

如果使用 IMU，且还未初始化第一帧位姿，则利用加速度计的读数来初始化位姿，并强制 yaw 为 0。

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


