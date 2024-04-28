---
layout: post
title:  "Attitude Rpresentation"
date:   2024-03-24 00:58:00 +0800
tags: 
  - linear algebra
categories:
  - math
---

Coordinate transformation: map a vector expressed in the coordinate frames $\mathcal{I}$ to the same vector expressed in $\mathcal{B}$, i.e. map $_I\vec{r}$ to $_B\vec{r}$

$$
\begin{align}
&
\begin{pmatrix}
\vec{e}^B_x & \vec{e}^B_x & \vec{e}^B_x
\end{pmatrix}
\begin{pmatrix}
c_1\\
c_2\\
c_3
\end{pmatrix} \\
=
&
\begin{pmatrix}
\vec{e}^I_x & \vec{e}^I_x & \vec{e}^I_x
\end{pmatrix}
\begin{pmatrix}
_I\vec{e}^B_x & _I\vec{e}^B_x & _I\vec{e}^B_x
\end{pmatrix}_{3 \times 3}
\begin{pmatrix}
c_1\\
c_2\\
c_3
\end{pmatrix}
\end{align}
$$

linear transformation: map a vector to another vector which both are expressed in the same coordinate frame (i.e. $\mathcal{I}$)

$$
\begin{align}
&
\begin{pmatrix}
\vec{e}^I_x & \vec{e}^I_x & \vec{e}^I_x
\end{pmatrix}
\begin{pmatrix}
c_1\\
c_2\\
c_3
\end{pmatrix} \\
\rightarrow
&
\begin{pmatrix}
\vec{e}^I_x & \vec{e}^I_x & \vec{e}^I_x
\end{pmatrix}
\begin{pmatrix}
_I\vec{e}^B_x & _I\vec{e}^B_x & _I\vec{e}^B_x
\end{pmatrix}_{3 \times 3}
\begin{pmatrix}
c_1\\
c_2\\
c_3
\end{pmatrix}
\end{align}
$$
