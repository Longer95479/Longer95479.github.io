---
layout: post
title:  "ego planner roslaunch"
date:   2024-02-29 10:54:00 +0800
tags: 
    - fast-drone
    - ROS
    - xml
categories:
  - fast-drone
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
- 对于 `</node>` 标签，`type=` 指明 `可执行文件`，`name = ` 用作 node 运行后使用 `rosnode list` 列出的节点的名字
- `remap` 中的 `from` 是 node 自己以为订阅的，`to` 是实际上订阅的
- `"~grid_map/odom"` 中的波浪线表示节点的私有命名空间，因此等价于 `"drone_$(arg drone_id)_ego_planner_node/grid_map/odom"`
    - 实例：`/drone_0_ego_planner_node/grid_map/depth`
    - 对应于 `grid_map.cpp` 中的源代码
        ```c++
        depth_sub_.reset(new message_filters::Subscriber<sensor_msgs::Image>(node_, "grid_map/depth", 50));
        ```
        由于 `grid_map/depth` 没有以 `/` 开头，所以默认加上节点的命名空间？
- `arg标签` 是用于将参数从命令行传递到launch文件中，`param标签` 是用于将参数从launch文件传递到ROS节点代码中，从而可以在 c++ 代码里通过 `node.param("param_name", var, default_val)` 获取到该参数（[ROS学习笔记（四）- ROS的launch文件](https://www.cnblogs.com/lihan829/p/17341176.html)）
    

---

在 ROS（Robot Operating System）中，确保包的正确编译顺序是非常重要的，特别是当一个包依赖另一个包时。要确保包的编译顺序，可以使用以下几种方法：

### 1. **使用 `find_package` 和 `catkin_package`**
在每个 ROS 包的 `CMakeLists.txt` 文件中，通过使用 `find_package` 和 `catkin_package` 声明依赖关系，可以确保 ROS 编译系统知道包之间的依赖性。

#### 示例：
```cmake
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  your_dependency_package  # 依赖的包
)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs your_dependency_package  # 依赖的包
)
```
这会确保 `your_dependency_package` 在编译当前包之前被正确编译。

### 2. **修改 `package.xml` 声明依赖**
`package.xml` 文件中也需要声明依赖关系。使用 `<build_depend>` 和 `<exec_depend>` 来声明构建和运行时依赖。

#### 示例：
```xml
<build_depend>your_dependency_package</build_depend>
<exec_depend>your_dependency_package</exec_depend>
```
确保依赖的包被声明为 `build_depend`，以便在编译时可以自动按照正确的顺序构建。

### 3. **使用 `catkin_make` 或 `catkin build`**
- 如果使用 `catkin_make`，ROS 会根据 `CMakeLists.txt` 和 `package.xml` 文件中的依赖关系，自动确定包的编译顺序。
- 如果使用 `catkin_tools` 的 `catkin build`，编译时也会自动根据依赖性来调整包的编译顺序，推荐使用这个工具，因为它可以更高效地处理依赖关系。

### 4. **使用 `rosdep` 安装依赖**
在编译之前，使用 `rosdep install --from-paths src --ignore-src -r -y` 来安装所有依赖包。`rosdep` 会检查依赖关系并安装缺失的包，以避免编译时找不到依赖包的错误。

### 5. **确保包路径顺序正确**
如果你在工作空间中有多个包，确保 `CMakeLists.txt` 和 `package.xml` 的依赖声明是正确的，并且 `catkin_make` 或 `catkin build` 能识别所有路径。你可以通过检查 `ROS_PACKAGE_PATH` 环境变量，确保 ROS 能找到依赖的包。

### 6. **在工作空间内进行分步构建**
如果某些包的依赖关系复杂，无法自动解析，你可以分两步构建：
1. 先编译依赖包：
   ```bash
   catkin_make --only-pkg-with-deps <your_dependency_package>
   ```
2. 然后再编译依赖这些包的目标包：
   ```bash
   catkin_make --only-pkg-with-deps <your_target_package>
   ```

通过这些方法，可以保证 ROS 包的编译顺序，避免 `Could not find a package configuration file provided by` 这类错误。