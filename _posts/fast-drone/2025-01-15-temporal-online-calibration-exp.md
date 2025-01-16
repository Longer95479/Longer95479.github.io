---
layout: post
title:  "Realization of Online Temporal Calibration on AirSLAM" 
date:   2025-01-15 14:00:00 +0800
tags: 
    - slam
    - calibration
categories:
    - coding 
---


AirSLAM 在公开数据集上的效果很不错，但在笔者自己录制的数据集上效果很差，具体如下：

- AirSLAM 使用 imu 和相机，定位漂移很大
- 而相同的外参在 vins-fusion 上的效果却还不错。
- AirSLAM 仅使用双目，漂移小，较为正常

根据第一条和第三条信息，可以推断出是 imu 和相机之间的协调出了问题，可能的原因是: 

- 时间基准不同：公开数据集的 imu 与 图像帧有硬件同步且参考同一个时间基准，而自己录制的数据集未作同步且不是同一个时间基准，而 vins-fusion 在运行过程中在线校准 imu 和图像帧的时间偏移。这是二者存在的一个较为明显的区别，
- 相机与 imu 之间的外参不够准确：虽然 vins-fusion 和 AirSLAM 使用的外参一致，但 vins 有对外参做在线估计校准，而 AirSLAM 没有。

本篇主要验证是否是未进行时间偏移估计导致的 AirSLAM 效果较差。笔者在 airslam 上增加了时间偏移估计，思路参考 vins-fusion，github 上 fork 的仓库内对应的提交为 
[Commit b16d3ea: add time composition between camera and imu data.](https://github.com/sair-lab/AirSLAM/commit/b16d3eac07ceb580c26b62f7f392e20f4d80b24d)。

最终的实验结果证明，自己的数据集中，相机的时间基准比 imu 的时间基准快 1.6 ms，影响较小，即使不进行在线时间偏移估计，也不会导致轨迹的大幅度漂移，因此导致 AirSLAM 在自己录制的数据集上漂移较大的并不是时间基准偏移，而是另有其因。

### 如何将时间偏移加入到优化框架中

对于多个传感器时间戳的质量，可以分成三类：
- 既同步，又有相同时间基准
- 不同步，但有相同时间基准
- 既不同步，又无相同时间基准

其中，无相同的时间基准对 VIO 或 SLAM 系统的影响最大

接下来我们需要搞清楚，是哪些数据组合喂给了前端：

- 接收到一帧新图像
- 根据这帧图像，以及上一帧图像的时间戳，收集在这段时间内的 imu 数据

如果存在时间基准的偏移，那么我们就会**把图像相对于自身时间基准的时间戳，当成是相对于 imu 时间基准的时间戳，然后收集打包两个时间戳之间的 imu 数据**

根据下图可以看到: 

- (1) 应当对应的 imu 数据，时间区间为 $[t_2, t_4]$
- (2) 实际打包给前端的 imu 数据，时间区间未 $[t_1, t_3]$

二者不再是同一个区间。在统一的时间基准下（以 imu 的时间基准为参考），图像帧对应的时间区间是 $[t_2, t_4]$，而打包的 imu 却是在 $[t_1, t_3]$

```
 td
|->|
   |----x-----x----->
        |<--->|      (1)
|--ooooooooooooo->
     |<--->|         (2) 
|----x-----x----->
|----t1-t2-t3-t4---->
```

解决思路的核心是用现有的信息预测其他时间的数据，类似插值，之后将修正后的数据带入原有的优化表达式，由于修正的数据的表达式中含有时间差，因此优化器会对 时间差 做优化，从而估计出时间差。

那什么是现有的数据呢？$t_2$、$t_4$ 两帧图像中的特征点位置，以及它们的速度就是现有的信息。需要预测的便是 $t_1$、$t_3$ 时刻的特征点位置。

某个投影约束的表达式为：

$$
r_i(Z,X) = z_i - \pi(T_{cw}\ ^wp)
$$

其中，约束的 id 是 $i$，特征点位置为 $z_i$，$pi$ 把相机系下的 3D 点投影到像素或归一化平面上。

对特征点进行修正后，代入残差表达式：

$$
r_i(Z,X) = (z_i - V_i t_d ) - \pi(T_{cw}\ ^wp)
$$

其中，特征点的运动速度为 $V_i$。在 $t_d$ 时间内特征点被假设为匀速运动。

这样我们便得到了最初版本的，包含时间偏移的残差表达式。但这还不是最终用在程序里的表达式。

最值得思索的是:

- 用已有的特征点去估计 imu batch 对应的起始或结束时刻的特征点位置

- 在应用的时候是以迭代的形式使用，使得算法能够处理较大时间偏移，因为匀速运动的假设会随着迭代的进行愈加正确。


$$
r_i(Z,X) = (z_i - V_i (t_d - t_d') ) - \pi(T_{cw}\ ^wp)
$$

### 实验

给同步的数据集人为增加时间偏移，作为真值。使用的数据集是 `MH_05_difficult.bag`。

对数据集中 imu 的时间戳偏移不同程度，并分别运行里程计。里程计漂移在本文定义为，里程计给出的终点位置与终点真值的距离。时间偏移估计值的初始值均设置为 0 ms。结果如下：

|实际时间基准偏移 [s]|偏移估计值 [s]|里程计漂移(w) [m]|里程计漂移(w/o) [m]|
|-|-|-|-|
|0.030|0.035|0.095|0.382|
|0.060|0.068|0.363|1.523|
|0.100|0.108|0.614|2.867|

下图展示了 100 ms 偏移情况下，时间偏移估计值是如何收敛到真实偏移附近的：

![temporal calibration](/assets/2025-01-15-temporal-online-calibration-exp/temporal_calibration.png)


需要注意的是，在 rosbag 中，有两个和时间相关的量，一个是和 topic、msg 同级的 t，另一个是在消息内部的 msg.header.stamp。二者是不同的，t 表示的是在 rosbag 中消息被记录或者被发布的时刻，而 msg.header.stamp 才是传感器数据的时间戳。

下面是将已作时间同步的数据集人为进行偏移一定时间的程序：

```python
#!/usr/bin/env python          
                               
import rosbag                  
import rospy
from std_msgs.msg import Header

# Time offset (30ms)
time_offset = 0.10  # 30ms = 0.03s

def adjust_timestamp_in_rosbag(input_bag_file, output_bag_file, topic_name):
    # Open the original bag file and the new bag file
    with rosbag.Bag(input_bag_file, 'r') as in_bag, rosbag.Bag(output_bag_file, 'w') as out_bag:
        for topic, msg, t in in_bag.read_messages():
            # If it's the specified topic, modify the timestamp
            if topic == topic_name:         
                new_time = msg.header.stamp + rospy.Duration(time_offset)
                # Update the timestamp in the message (if the message contains a Header)
                if hasattr(msg, 'header') and hasattr(msg, 'header'):
                    msg.header.stamp = new_time     
                else:
                    rospy.logwarn(f"Message {topic} does not have a Header, skipping timestamp adjustment")

            # Write the modified message to the new bag file
            out_bag.write(topic, msg, t)    

        rospy.loginfo(f"Timestamps have been offset by {time_offset} seconds. Output file: {output_bag_file}")

if __name__ == '__main__':
    # Specify the input and output bag file paths, and the topic name to adjust
    input_bag_file = '../rosbags/MH_05_difficult.bag'  # Input ROS bag file
    output_bag_file = '../rosbags/' + 'MH_05_difficult_offset' + str(int(time_offset*1000)) + "ms.bag" # Output ROS bag file
    topic_name = '/imu0'  # The topic name to adjust timestamps for

    # Call the function to adjust the timestamps
    adjust_timestamp_in_rosbag(input_bag_file, output_bag_file, topic_name)
```

### 其他测试

为了直观地观察 imu 和图像时间戳的分布情况，写了一个脚本将其绘制出来。




