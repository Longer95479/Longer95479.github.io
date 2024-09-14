---
layout: post
title:  "[Paper Reading] A Tutorial on Quantitative Trajectory Evaluation for Visual(-Inertial) Odometry"
date:   2024-09-12 15:58:00 +0800
tags: 
    - slam
categories:
    - paper reading
---


Quantitatively comparing the estimated trajectory with
the groundtruth, however, is not an easy task. There are
two major difficulties. 

- First, the estimated trajectory and
the groundtruth are usually expressed in different reference
frames, and, therefore, cannot be compared directly.

-  Second,
a trajectory consists of the states at many different times and,
therefore, is high-dimensional data. 

Thus, how to summarize
the information of the whole trajectory into concise accuracy
metrics is not trivial. 

- To address the first problem, the
estimated trajectory requires to be properly transformed into
the same reference frame as the groundtruth, which is often
called trajectory alignment. 

- To address the second problem,
meaningful error metrics need to be used and their properties
well understood.



Roughly speaking, we integrate the raw IMU
measurements to get the relative rotation ∆ R̃ ik , velocity
∆ṽ ik and position ∆p̃ ik between two states x i and x k , and
the integration is formulated to be independent of the states
(except for the biases) so that re-integration is not needed
when the states change (e.g., during optimization iterations).
The corresponding measurement model is


(6) has infinite solutions that have the same minimum
cost. The reason is that the predicted measurements f (X)
are invariant to certain transformations g(·) of the parameter,
namely $f (X) = f (X' )$ with $X' = g(X)$. Since the measure-
ments z̃ are constant, the cost function (7) is also invariant to
