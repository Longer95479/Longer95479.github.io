---
layout: post
title:  "ROS Dependency Management"
date:   2024-02-26 19:05:00 +0800
categories: slam; ros
---

## package.xml

```xml
<package>
  ...
  <name>example_pkg</name>
  <buildtool_depend>catkin</buildtool_depend>
  <build_depend>cpp_common</build_depend>
  <build_depend>log4cxx</build_depend>
  <test_depend>gtest</test_depend>
...
  <run_depend>cpp_common</run_depend>
  <run_depend>log4cxx</run_depend>
</package>
```

- `<build_depend>` 和 `<buildtool_depend>` 将保证给定的依赖项先被配置，当依赖项在同一个工作空间时
    - 当进行交叉编译时，`<buildtool_depend>` 指定的是用于当前架构上的依赖，而不是目标架构上的依赖
    - 当未进行交叉编译时，`<buildtool_depend>` 等价于 `<build_depend>`
- `<run_depend>` 有两种作用
    - 声明包中的哪些可执行文件需要运行
    - 定义使用了当前包的其他包的依赖