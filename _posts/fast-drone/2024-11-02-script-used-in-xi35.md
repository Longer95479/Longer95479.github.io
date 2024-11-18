---
layout: post
title:  "Script used in XI35"
date:   2024-11-02 13:00:00 +0800
tags: 
    - script
categories:
    - fast-drone
---

利用自动化脚本，执行重复、确定的操作。

## 统一配置参数

算法以代码为实体部署后，会存在许多可调节的参数，当参数未被合理地给定，即使逻辑正确，算法的效果也可能并不如意。因此能够快速简便地调整参数便十分重要。


Fast-drone-xi35 里的模块较多，每个模块有不同的参数配置方法，可分成这几类：

- 在 launch 文件里给出参数具体值，使用 `roslaunch` 命令读取参数配置。

- 在 yaml 文件里给出参数具体值，使用 `cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);` 函数读取参数配置文件，给变量赋值。如 VINS-Fusion-gpu。

- 在 yaml 文件里给出参数具体值，在 launch 文件里调用该 yaml 文件，最后使用 `roslaunch` 命令将参数值赋值到程序里的变量中。如 px4ctrl：
    ``` xml
    <launch>
        <node pkg="px4ctrl" type="px4ctrl_node" name="px4ctrl" output="screen">
                <!-- <remap from="~odom" to="/vicon_imu_ekf_odom" /> -->
                <remap from="~odom" to="/vins_fusion/imu_propagate" />
                <remap from="~cmd" to="/position_cmd" />
                <rosparam command="load" file="$(find px4ctrl)/config/ctrl_param_fpv.yaml" />
        </node>
    </launch>
    ```

对于整个工程来说，需要调整的参数分布在不同的 launch 或 yaml 文件中，所在的目录各异，并不能直观方便地统一调整，还需要动脑想一下这个参数在哪里。

为了解决这个问题，利用 GPT 大法，写了个 python 脚本，配合 yaml 文件，能够统一地管理修改频率较高的参数。原理是在 yaml 文件里给出某个包/模块的配置文件所在路径，之后利用正则表达式搜索该文件里的某一行，将其数值部分替换成我们想要的参数值。

yaml 文件如下：

```xml
# 工程顶层目录所在路径
path_prefix: /root/Fast-Drone-XI35/
# path_prefix: /home/orin01/Fast-Drone-XI35/

paths:
  src/auto_search/target_merge/launch/target_merge.launch:
    - 
      # 组别 id
      <param name="drone_id" value="\d+" type="int"/>: <param name="drone_id" value="2" type="int"/>

  src/realflight_modules/VINS-Fusion-gpu/config/fast_drone_250.yaml:
    -
      # 里程计类型，0 不加偏置，1 加偏置（统一集群的世界坐标系）
      "odometry_type: \\d+": "odometry_type: 1"
      # 无人机个体 id
      "drone_id: \\d+": "drone_id: 2"
      # 偏置/坐标变换的数值
      "single_offset: -*\\d+.\\d+": "single_offset: 2.0"

      # 显示特征点追踪
      "show_track: \\d+": "show_track: 0"

  src/auto_search/search_plan/launch/search_plan.launch:
    - 
      # search plan 判断是否到达给定点的阈值半径
      <param name="arrive_threshold" value="\d+.\d+" type="double"/>: <param name="arrive_threshold" value="0.5" type="double"/>
...
```

python 文件见 [config_scipt/set_global_config.py](https://github.com/Longer95479/Fast-Drone-XI35/blob/board/config_scipt/set_global_config.py)


## 同时打开多个多窗口的终端

想要启动一架自主四旋翼的所有功能，需要十一个终端，数目众多，如果要启动集群，以八架为例，则需要打开八个大窗口，再将每个窗口分割为十一个窗口，并在每个窗口里输入所需的命令，极其繁杂。因此，必须利用脚本工具来自动化执行这些重复性、确定性的操作。

解决办法是利用 terminator 的布局配置文件，参考 [使用 Terminator 在一个窗口中运行多个终端](https://linux.cn/article-11409-1.html):

> 将窗口分为多个部分后，将自定义的 Terminator 设置设置为默认非常容易。从弹出菜单中选择“首选> 项”，然后从打开的窗口顶部的选项卡中选择“布局”。接着你应该看到列出了“新布局”。只需单击底部的> “保存”，然后单击右下角的“关闭”。Terminator 会将你的设置保存在 ~/.config/terminator/> config 中，然后每次使用到时都会使用该文件。
> 
> 你也可以通过使用鼠标拉伸来扩大整个窗口。再说一次，如果要保留更改，请从菜单中选择“首选项”，“布局”，接着选择“保存”和“关闭”。

针对 XI35，[hh900](https://github.com/hh900) 写了一个 terminator 的脚本，能够自动让每个终端进入容器，并预先打印出将要执行的命令。笔者在此基础上增加了非容器的版本。脚本见 [terminator/config-orin01-nodocker](https://github.com/Longer95479/Fast-Drone-XI35/blob/board/terminator/config-orin01-nodocker)，若要阅读，倒序阅读方便查看有意义的命令。


## PX4 配置文件导出与加载

为了简化其他无人机的飞控配置过程，应该使用地面站将已配好的无人机的飞控参数导出成文件，之后仍是使用地面站将该文件导入到其他无人机当中。
