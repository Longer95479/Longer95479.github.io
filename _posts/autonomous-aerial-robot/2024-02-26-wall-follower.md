---
layout: post
title:  "Wall Follower"
date:   2024-03-16 21:05:00 +0800
tags: 
    - motion planning
categories:
    - autonomous aerial robot
---

How to make a drone follow a wall while flying?

Pesudocode of finding next waypoint to follow the wall is shown as the flollwing.

<pre class="pseudocode">
\begin{algorithm}
\caption{Find\_next\_waypoint\_for\_wall\_following}
\begin{algorithmic}

\REQUIRE pts\_end $^{body}$ are initialzed
\ENSURE some postconditions
\INPUT pts\_end $^{body}$, $\vec{p} \in$ pts\_end $^{body}$, $T_{body}^w$
\OUTPUT next\_way\_point
\FUNCTION{Find-next-waypoint-for-wall-following}{pts\_end $^{body}$}
    \IF{has\_reached\_waypoint}
    \FOR{$\mathbf{each}$ pt\_end$^{body}$ $\in$ pts\_end$^{body}$}
        \STATE pt\_end = pt\_end$^{body}$ $\cdot R_{body}^w + t_{body}^w$
        \STATE raycaster.\CALL {Set-input}{body\_position/resolution, pt\_end/resolution}
        \WHILE {raycaster. \CALL {step}{ray\_pt}}
            \IF {\CALL {is-known-occupied}{ray\_pt}}
                \STATE Occupied\_pts.\CALL{push-back}{ray\_pt}
            \ENDIF
        \ENDWHILE
    \ENDFOR
    \IF {Occupied\_pts.\CALL {Size}{} > certain\_threshold}
        \COMMENT{Wall existing}
        \STATE $\vec{v}$ = \CALL{Plane-Fitting}{Occupied\_pts}
        \STATE next\_way\_point = $R_{body}^w \vec{p} +$ \CALL {Sign}{(body\_position - $\vec{p}$) $\cdot \vec{v}$} $\cdot d_w  \frac{\vec{v}}{||\vec{v}||}$
        \IF{\CALL {is-known-occupied}{next\_way\_point}}
        \STATE next\_way\_point = \CALL {Ray-casting}{body\_position, next\_way\_point}
        \STATE next\_way\_point = body\_position + k $\cdot$ (next\_way\_point - body\_position)
        \COMMENT {0 < k < 1}
        \ENDIF
    \ELSE
        \COMMENT {Wall absent}
        \STATE next\_way\_point = \CALL{Move-right-forward}{$T_{body}^w$}
    \ENDIF
    \STATE Occupied\_pts.\CALL{clean}{}
    \RETURN next\_way\_point
    \ENDIF
\ENDFUNCTION

\end{algorithmic}
\end{algorithm}
</pre>

<pre class="pseudocode">
\begin{algorithm}
\caption{Ray-casting}
\begin{algorithmic}

\FUNCTION{Ray-casting}{pt\_start, pt\_end}
\STATE to do
\ENDFUNCTION

\end{algorithmic}
\end{algorithm}
</pre>

<pre class="pseudocode">
\begin{algorithm}
\caption{Plane-Fitting}
\begin{algorithmic}

\FUNCTION{Plane-Fitting}{pts}
\STATE to do
\ENDFUNCTION

\end{algorithmic}
\end{algorithm}
</pre>


## FOV points initialization

<pre class="pseudocode">
\begin{algorithm}
\caption{pts\_end\_body\_initialization}
\begin{algorithmic}

\FUNCTION {Pts-end-body-initialization}{f,deltaY,deltaZ,X}

width_idx = (int)std::ceil(f_/X_ * deltaY_)

\ENDFUNCTION

\end{algorithmic}
\end{algorithm}
</pre>

In VINS-Fusion, in order to simplify the notation, author denotes the IMU as body\'s center (if the IMU isn\'t used, left camera is denoted as body\'s center).

NED (North-East-Down) convenstion is used as that of body frame in PX4, but NWU (North-West-Up) is denoted as body frame in VINS-Fusion and fast-drone-250.

In ego-planner (or fast-drone-250), planner node subscribes `entrinsic` topic and `depthOdom` synchronizer in order to compute the `cam_r` and `cam_pos` using entrinsic and odom (body frame) translation.


## Modification in fast-drone source code

`Wall follower` class should be added in `EGOPlannerManager` class, rather than creating a new nodes, as `GridMap::Ptr grid_map_` can only be observable in `EGOPlannerManager` class to be reused.

### Three steps need to be done while adding a new package in the catkin workspace to develop a project

1. create a new package, including writing `package.xml, Cmakelists.txt, *.cpp, *.h` files.
2.  Modify .cpp .h of the package which depend on the new one.
    - add `#include` into `.h` of the package using the new one
    - add initialization function of new package into `.cpp` of the package using the new one
3. Modify `package.xml Cmakelists.txt` of the package using the new one

More details is shown as follow.

### wall_follower.h and wall_follower.h under wall_follower dir

wall_follower.cpp: 

- Move `ptsEndFovGeneration()` from class `WallFollower` to `WallFollower::PtsEndFov`, and modify corsponding varias name (e.g. `X_` to `X = pts_end_fov_ptr->X_;`)
- add some param print
- Move `pts_end_fov_pub_` from constructor of `WallFollower` to that of `WallFollower::PtsEndFov`
- **fix a bug** in the function `publicPtsEndFov()`
    ```c++
        void WallFollower::PtsEndFov::publicPtsEndFov()
    ... {
    327      for (auto& pt_end_world: pts_end_world) {
    328          pt.x = pt_end_world(0);
    329          pt.y = pt_end_world(1);
    330          pt.z = pt_end_world(2);
    331 +        cloud.push_back(pt);
    332      }
    ... }

    ```
- **fix the logic** of projecting `pts_end_body_` to `pts_end_world_` by varify odom before using it.
    ```c++
    199 +    if (grid_map_ptr_->md_.has_odom_) {
    ...
    214 +        for (auto& pt_end: pts_end) {
    215 +            pt_end = camera_r_m * pt_end + camera_pos;
    216 +        }
    ...
    221 +        pts_end_fov_ptr_->pts_end_world_ = pts_end;
    222 +    }
    ```
- add `odom_sub_`

wall_follower.h: 




### grid_map.h and grid_map.cpp under plan_env fir

All modification in this file is to make `camera_pos_` and `camera_r_m_` update and observable for projecting `pts_end_body_` to `pts_end_world_` in `wall_follwer.cpp`

grid_map.h: 

- make `mp_` and `md_` public
    ```c++
    9 -private:
    10    MappingParameters mp_;
    11   MappingData md_;
    12  
    13 +  private:
    14 +
    ```

grid_map.cpp: 

- ~~add oritation update in `void GridMap::odomCallback(const nav_msgs::OdometryConstPtr &odom)`~~ The reason why this change was reversed is that extrinsic of exp and sim are different. subscribing `grid_map/odom` in wall_follower.cpp directly instead of using `grid_map_ptr_->md.camera_pos_` and `grid_map_ptr_->md.camera_r_m_`
    ```c++
    22 @@ -735,6 +735,11 @@ void GridMap::odomCallback(const nav_msgs::OdometryConstPtr &odom)
    {
    23    md_.camera_pos_(1) = odom->pose.pose.position.y;
    24    md_.camera_pos_(2) = odom->pose.pose.position.z;
    25  
    26 +  md_.camera_r_m_ = Eigen::Quaterniond(odom->pose.pose.orientation.w,
    27 +                                       odom->pose.pose.orientation.x,
    28 +                                       odom->pose.pose.orientation.y,
    29 +                                       odom->pose.pose.orientation.z).toRotationMatrix();
    30 +
    31    md_.has_odom_ = true;
    32  }
    ```

### ego_replan_fsm.h and ego_replan_fsm.cpp under plan_manage dir

### package.xml and CmakeLists.txt under plan_manage dir


## Reference

- [Project 2: Robot Wall Following by Reinforcement Learning](https://hcr.cs.umass.edu/courses/compsci603/projects/Compsci_603_Project2_WF.pdf)
- [Chapter 9. Motion Planning in Simple Geometric Spaces](http://motion.cs.illinois.edu/RoboticSystems/GeometricMotionPlanning.html)


---

## Debug log

### Invalid use of non-static data member

Nested classes are not connected to any instance of the outer class.

My case:

```c++
class WallFollower {
    struct PtsEndFov {
        ...
        void publicPtsEndFov();
    }

    GridMap::Ptr grid_map_ptr_;
}
```

`grid_map_ptr_` is used in `void publicPtsEndFov();` leading to error "Invalid use of non-static data member".

[stackoverflow: invalid use of non-static data member](https://stackoverflow.com/questions/9590265/invalid-use-of-non-static-data-member)

### No matching function for call to ‘ros::NodeHandle::param’

```c++
nh.param("wall_follower/f", f_, 100);
```
It fails on builds with the following error:
```bash
error: No matching function for call to ‘ros::NodeHandle::param’
```

`f_` is `double` type and `100` is `int` type. They are not matching.
The correction is shown as below:

```c++
nh.param("wall_follower/f", f_, 100.0);
```

### Why catkin_make make nothing change after I edited the source code

Answer in [ROS Answers](https://answers.ros.org/question/331239/why-does-not-catkin_make-work/): 
> I understand it as both your src folder contains files with the same names and same CMakeList but only some lines of code differs between the two. That being said when you run catkin_make after swapping your folders catkin doesn't notice any change in your files (I don't really know how, probably CMake related, but if someone can enlighten me on this part) so if nothing has changed catkin won't rebuild everything as usual.
>
>To avoid that you have two options :
> - Delete the build and devel folder each time you change the src folder
> - Change a little thing in a file so that catkin notice a change and rebuild the package

My problem has been solved following "Delete the build and devel folder each time you change the src folder".



### `WallFollower::~WallFollower()' is defined multiple times

I wrote deconstruct function outside the class in .h file, 
which should be enclosed by class that it belong to.

### static member function

class members should not be used in static member function, as there is no instance while using a static member function.

### std::vector can't be access the element whose index out of range

if there are not elements pushed back (i.e. vector is clear), any elements can't be accessed using index.


### If no occupied points, it shouldn't use planeFitting()
 
 ```c++
rand()%occupied_pts.size(); //will be rand()%0, wrong! 
 ```

### Invalid argument passed to canTransform argument source_frame in tf2 frame_ids cannot be empty

### %0 is no define
Just change the order of a variable

### next_waypoint need to be initialized

```c++
void WallFollower::findWayPointCallback(const ros::TimerEvent& /*event*/)
{
    if (have_odom_) {
        if (!is_next_waypoint_initialized_) {
            next_way_point_ = body_pos_;
            is_next_waypoint_initialized_ = true;
        }
        // std::cout << "MY_DEBUG: dist = " << (body_pos_-next_way_point_).norm() << std::endl;
        if ((body_pos_-next_way_point_).norm() < 0.5) {
            findNextWayPoint();
        }
    }
}
```

### fix most forward point in plane
```c++
    /* get the point of most front in the best plane */
    double max_temp = 0;
    for (auto &pt: occupied_pts) {
        double d = plane_fitter_ptr_->solveDistance(pt, last_best_plane);
        if (d < plane_fitter_ptr_->sigma_) {
            // double temp = body_pos_.normalization().transpose() * pt;
            double x_local = (body_r_m_.transpose() * (pt - body_pos_))(0);
            if (x_local > max_temp) {
                max_temp = x_local;
                last_best_plane.p_ = pt;
            }
        }
    }
```

