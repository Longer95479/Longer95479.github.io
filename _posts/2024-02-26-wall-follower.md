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

