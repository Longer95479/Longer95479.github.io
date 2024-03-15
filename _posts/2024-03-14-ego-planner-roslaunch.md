---
layout: post
title:  "ego planner roslaunch"
date:   2024-02-29 10:54:00 +0800
categories: 
    - fast-drone
    - ros
    - xml
---

以下是 `advanced_param.xml` 的简略版本：
```xml
<launch>
  <arg name="map_size_x_"/>
  <arg name="map_size_y_"/>
  <arg name="map_size_z_"/>

  <!-- main node -->
  <!-- <node pkg="ego_planner" name="ego_planner_node" type="ego_planner_node" output="screen" launch-prefix="valgrind"> -->
  <node pkg="ego_planner" name="drone_$(arg drone_id)_ego_planner_node" type="ego_planner_node" output="screen">

    <remap from="~grid_map/odom" to="/drone_$(arg drone_id)_$(arg odometry_topic)"/>
    <remap from="~grid_map/cloud" to="/drone_$(arg drone_id)_$(arg cloud_topic)"/>
    <remap from="~grid_map/pose"   to = "/drone_$(arg drone_id)_$(arg camera_pose_topic)"/> 
    <remap from="~grid_map/depth" to = "/drone_$(arg drone_id)_$(arg depth_topic)"/>
    

    <!-- planning fsm -->
    <param name="fsm/flight_type" value="$(arg flight_type)" type="int"/>
    <param name="fsm/thresh_replan_time" value="1.0" type="double"/>
    <param name="fsm/thresh_no_replan_meter" value="1.0" type="double"/>
    <param name="fsm/planning_horizon" value="$(arg planning_horizon)" type="double"/>


  </node>
</launch>
```
- 对于 `</node>` 标签，`type=` 指明`可执行文件`，`name = ` 用作 node 运行后使用 `rosnode list` 列出的节点的名字
- `remap` 中的 `from` 是 node 自己以为订阅的，`to` 是实际上订阅的
- `"~grid_map/odom"` 中的波浪线表示节点的私有命名空间，因此等价于 `"drone_$(arg drone_id)_ego_planner_node/grid_map/odom"`
    - 实例：`/drone_0_ego_planner_node/grid_map/depth`
    - 对应于 `grid_map.cpp` 中的源代码
        ```c++
        depth_sub_.reset(new message_filters::Subscriber<sensor_msgs::Image>(node_, "grid_map/depth", 50));
        ```
        由于 `grid_map/depth` 没有以 `/` 开头，所以默认加上节点的命名空间？
- `arg标签` 是用于将参数从命令行传递到launch文件中，`param标签` 是用于将参数从launch文件传递到ROS节点代码中，从而可以在 c++ 代码里通过 `node.param("param_name", var, default_val)` 获取到该参数（[ROS学习笔记（四）- ROS的launch文件](https://www.cnblogs.com/lihan829/p/17341176.html)）
    
