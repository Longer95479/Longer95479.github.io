---
layout: post
title:  "Handcraft Ray Tracing"
date:   2025-12-15 22:30:00 +0800
tags: 
  - programming
  - math 
categories:
  - programming
  - math 
---

本文将介绍如何手搓光线追踪。先上效果图：

![](/assets/2025-12-15-Handcraft-Ray-Tracing/my_raytracing-120p.gif)

![](/assets/2025-12-15-Handcraft-Ray-Tracing/my_raytracing_blue-120p.gif)


代码仓库：[handcraft-MVS](https://github.com/Longer95479/handcraft-MVS)。

实现了：

- 漫反射
- 基于菲涅尔定律的折射与反射
- 允许多光源
- 圆形物体

TODO：

- [ ] 长方体物体
- [ ] 另起一篇实现光栅化


## 简介

通过程序模拟光线从光源发出后到达相机光心的路径，来渲染出接近现实光照变化的图像。

## 漫反射的实现

本节考虑的物体表面都只会发生漫反射。漫反射是指，由于物体表面的粗糙，一束光线击中表面后，会向各个方向反射，反射光线与入射光线角度越大，反射光线的强度越弱。

最朴素的思考则是从光源出发，思考光线的传播。考虑只有单次漫反射这一最简单的情况，当光源发出的光线第一次打到物体时，如前文所述，会向各个方向反射，而这么多法反射的光线中，只有穿过成像平面的光线是对渲染有意义的，其余的反射光线我们则不关心。

因此，利用光路的可逆性，我们反过来思考:

- 从相机光心出发射出一条光线，判断是否会打到物体表面的一个点上
- 若能打到，再从这个点上发出一条光线，判断是否会打到光源上
- 若能打到，则可以计算出该视角下该点的像素值

伪代码如下：

```c++
// retval: surface_color
vec3 trace(const vec3& ray_ori, 
           const vec3& ray_dir, 
           const vector<ObjSharedPtr>& objs, 
           int depth) 
{
    obj_ptr = nullptr;
    getClosestHitPt(ray_ori, ray_dir, objs, &obj_ptr, &t);

    if (obj_ptr == nullptr ) return vec(0.1, 0.1, 0.1); 

    pt_hit = ray_ori + ray_dir * t;
    norm_hit = obj_ptr->calNormHit(pt_hit);

    for (light: lights) {
        shadow_ray_dir = norm(light->center_ - pt_hit);
        is_shadow = checkIntersect(pt_hit+norm_hit*eps, shadow_ray,dir);

        if (is_shadow) continue;

        ratio = max(0.0, norm_hit.transpose() * shadow_ray_dir);
        surface_color += ratio * light->emission_color_;
    }

    surface_color += obj_ptr->emission_color_;

    return surface_color;
}
```

## 折射与反射的实现

**菲涅尔方程组** 与 **递归实现** 为两个核心。

### 菲涅尔方程组

菲涅尔方程组描述了光线在两种介质分界处反射和折射的现象。给定入射角 $\theta_i$，能够利用该方程组计算出反射角 $\theta_r$、折射角 $\theta_t$、反射率（反射能量占比） $K_r = R_{eff}$ 、折射率 $K_t = i - K_r$。

$$
\theta_r = \theta_i
$$

$$
n_1 \sin \theta_i = n_2 \sin \theta_t
$$

$$
r_s = \left |
\frac{n_1 \cos \theta_i - n_2 \cos \theta_t}{n_1 \cos \theta_i + n_2 \cos \theta_t}
\right |
$$

$$
r_p = \left |
\frac{n_2 \cos \theta_i - n_1 \cos \theta_t}{n_2 \cos \theta_i + n_1 \cos \theta_t}
\right |
$$

$$
R_{eff} = \frac{R_s + R_p}{2} = \frac{r_s^2 + r_p^2}{2}
$$

$$
K_r = \min (1, R_{eff})
$$

$$
K_t = 1 - K_r
$$

### 递归实现




## 着色器（shader）的实现

为每个像素的 RGB 通道分配一个值，这便是着色器所作的事。

## 附录

### 反射变换的矩阵表示

### NDC 坐标系

### 射线生成