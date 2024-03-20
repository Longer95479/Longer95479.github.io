---
layout: post
title:  "Wall Follower"
date:   2024-03-16 21:05:00 +0800
tags: 
    - motion planning
categories:
---

How to make a drone follow a wall while flying?

Pesudocode of finding next waypoint to follow the wall is shown as the flollwing.

<pre class="pseudocode">
\begin{algorithm}
\caption{Find\_next\_waypoint\_for\_wall\_following}
\begin{algorithmic}

\REQUIRE pts\_end $^{body}$ are initialzed
\ENSURE some postconditions
\INPUT pts\_end $^{body}$, $\vec{p} \in$ pts\_end $^{body}$
\OUTPUT next\_way\_point
\FUNCTION{Find-next-waypoint-for-wall-following}{pts\_end $^{body}$}
    \IF{has\_reached\_waypoint}
    \FOR{all pt\_end$^{body}$ $\in$ pts\_end$^{body}$}
        \STATE pt\_end = pt\_end$^{body}$ $\cdot R_{body}^w$
        \STATE raycaster.\CALL {Set-input}{body\_position/resolution, pt\_end/resolution}
        \WHILE {raycaster. \CALL {step}{ray\_pt}}
            \IF {\CALL {is-known-occupied}{ray\_pt}}
                \STATE Occupied\_pts.\CALL{push-back}{ray\_pt}
            \ENDIF
        \ENDWHILE
    \ENDFOR
    \STATE $\vec{v}$ = \CALL{Plane-Fitting}{Occupied\_pts}
    \STATE Occupied\_pts.\CALL{clean}{}
    \STATE next\_way\_point = $R_{body}^w \vec{p} + d_w \frac{\vec{v}}{||\vec{v}||}$
    \IF{\CALL {is-known-occupied}{next\_way\_point}}
        \STATE next\_way\_point = \CALL {Ray-casting}{body\_position, next\_way\_point}
        \STATE next\_way\_point = body\_position + k $\cdot$ (next\_way\_point - body\_position)
        \COMMENT {0 < k < 1}
    \ENDIF
    \RETURN next\_way\_point
    \ENDIF
\ENDFUNCTION

\FUNCTION{Ray-casting}{pt\_start, pt\_end}
\STATE to do
\ENDFUNCTION
\FUNCTION{Plane-Fitting}{pts}
\STATE to do
\ENDFUNCTION

\end{algorithmic}
\end{algorithm}
</pre>


### FOV points initialization

<pre class="pseudocode">
\begin{algorithm}
\caption{pts\_end\_body\_initialization}
\begin{algorithmic}

\FUNCTION {Pts-end-body-initialization}{}

\ENDFUNCTION

\end{algorithmic}
\end{algorithm}
</pre>

In VINS-Fusion, in order to simplify the notation, author denotes the IMU as body\'s center (if the IMU isn\'t used, left camera is denoted as body\'s center).

NED (North-East-Down) convenstion is used as that of body frame in PX4, but NWU (North-West-Up) is denoted as body frame in VINS-Fusion and fast-drone-250.

In ego-planner (or fast-drone-250), planner node subscribes `entrinsic` topic and `depthOdom` synchronizer in order to compute the `cam_r` and `cam_pos` using entrinsic and odom (body frame) translation.


### Modification in fast-drone source code

`Wall follower` calss should be added in `EGOPlannerManager` class, rather than creating a new nodes, as `GridMap::Ptr grid_map_` can only be observable in `EGOPlannerManager` class and can be reused.

### Reference

- [Project 2: Robot Wall Following by Reinforcement Learning](https://hcr.cs.umass.edu/courses/compsci603/projects/Compsci_603_Project2_WF.pdf)
- [Chapter 9. Motion Planning in Simple Geometric Spaces](http://motion.cs.illinois.edu/RoboticSystems/GeometricMotionPlanning.html)


---

### Debug log

#### - Invalid use of non-static data member

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

#### - No matching function for call to ‘ros::NodeHandle::param’

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

#### - Why catkin_make make nothing change after I edited the source code

Answer in [ROS Answers](https://answers.ros.org/question/331239/why-does-not-catkin_make-work/): 
> I understand it as both your src folder contains files with the same names and same CMakeList but only some lines of code differs between the two. That being said when you run catkin_make after swapping your folders catkin doesn't notice any change in your files (I don't really know how, probably CMake related, but if someone can enlighten me on this part) so if nothing has changed catkin won't rebuild everything as usual.
>
>To avoid that you have two options :
> - Delete the build and devel folder each time you change the src folder
> - Change a little thing in a file so that catkin notice a change and rebuild the package

My problem has been solved following "Delete the build and devel folder each time you change the src folder".



#### - `WallFollower::~WallFollower()' is defined multiple times

I wrote deconstruct function outside the class in .h file, 
which should be enclosed by class that it belong to.

