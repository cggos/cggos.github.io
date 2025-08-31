---
title: Observability and Inconsistency in a Nutshell
tags:
  - State Estimation
  - Observability and Consistency
  - Lie Theory
  - Observability Matrix
  - FEJ
  - OC-VINS
  - Degeneracy
categories:
  - State Estimation
index_img: /img/post/observability_consistency/nullspace_disappear.png
sticky: 10000
key: observability-inconsistency-overview
abbrlink: 881febbe
date: 2022-07-16 00:00:00
---

[TOC]

# Overview

What is **observability** ?

> In control theory, observability is a measure for how well internal states of a system can be inferred by knowledge of its external outputs.

What is **consistency** ?

> A recursive estimator is consistent when the estimation errors are zero-mean and have covariance matrix equal to that reported by the estimator.

<p class="note note-primary">
Observability $\longrightarrow$ Consistency
</br>
</br>
Mismatch (actual vs true) in observability $\longrightarrow$ Inconsistency
</br>
</br>
VINS observability properties $\longrightarrow$ estimator inconsistency
</p>


# Basics

## Nullspace [^1]

<p align="center">
  <img src="/img/post/observability_consistency/nullspace_svd.jpg">
</p>

## Lie Derivative [^2]

已知，光滑标量函数 $h$ 以及 光滑向量场 $f$ 和 $g$

$$
\begin{aligned}
h(x):& \; R^n \rightarrow R \\
f(x):& \; R^n \rightarrow R^n \\
g(x):& \; R^n \rightarrow R^n
\end{aligned}
$$

行向量 梯度 $\nabla h$ 乘以 向量场 $f$，其结果 $L_f h$ 正好是个标量

$$
L_f h = \nabla h \cdot f = \left( \frac{\partial h}{\partial x} \right)^T f
$$

$L_g L_f h$ 结果依然是个标量

$$
L_g L_f h = L_g (L_f h) 
= \nabla(L_f h) g 
= \left( \frac{\partial (L_f h)}{\partial x} \right)^T g
= \left( \frac{\partial \left( (\frac{\partial h}{\partial x})^T f \right)}{\partial x} \right)^T g
$$

总结一下：Lie Derivative与一般的Derivative的区别是，Lie Derivative是定义在两个函数 $h$ 和 $f$ 之间的，它俩都是向量 $x$ 的函数，通过共同的 $x$ 联系起来；一般的Derivative是某个函数对 $x$ 定义的。


# Observability Analysis [^4]

* what: 控制理论中的 可观察性（observability）[^3] 是指系统可以由其外部输出推断其内部状态的程度。

* why: 为了能让系统不可观的维度与真实系统一致，从而提高系统精度

* how: 通过计算可观性矩阵，分析其零空间的秩，来分析系统哪些状态维度可观/不可观；可观性矩阵对应系统可观测的维度，零空间对应系统不可观的维度


## Unobservable DoF (Gauge Freedom) in SLAM

* Mono vSLAM: 7
  - 6 DoF 绝对位姿 + 尺度
* Stereo vSLAM: 6
  - 6 DoF 绝对位姿
* Mono + IMU SLAM: 4
  - 3 DoF 绝对位置 + 绝对yaw角
  - roll 和 pitch 由于重力的存在而可观，尺度因子由于加速度计的存在而可观

## Observability Matrix

Discrete state space equations of nonlinear systems (linearized without considering noise) is

$$
\begin{cases} x_{k+1} = \Phi_k x_k \\ y_k = H_k x_k \end{cases}
$$

according to the **Lie derivative**, the **observability matrix** is

$$
\mathbf{\mathcal{O}} \left(\mathbf{x}^{\star}\right) =
\left[
  \begin{array}{c}
  \mathbf{H}_{1} \\
  \mathbf{H}_{2} \boldsymbol{\Phi}_{2,1} \\
  \vdots \\
  \mathbf{H}_{k} \boldsymbol{\Phi}_{k, 1}
  \end{array}
\right]
$$

then, the unobservable dimensions of the system are

$$
\text{rank}(N(\mathcal{O}))
$$

### Observability Matrix vs Hessian(Information) Matrix

对于SLAM系统而言（如单目VO），当我们改变状态量时，测量不变意味着损失函数不会改变，更意味着求解最小二乘时对应的信息矩阵H存在着零空间。

for the monocular VO based on optimization methods, the dimension of null space of **the Hessian (Information) matrix** $H$ is 7, that is, the unobservable dimensions are

$$
\text{rank}(N(H)) = 7
$$

$$
J^T J \Delta x = - J^T r \quad \longrightarrow \quad H \Delta x = b
$$

**What is the relationship between the Hessian matrix $H$ and the observability matrix $\mathcal{O}$ in the optimization based VO/VIO ?**

* paper: ***Observability-Based Guidance and Sensor Placement*** (Chapter 2 - OBSERVABILITY MEASURES)
  > As a note, the measurement Jacobian, $dY$, is equivalent to the observability matrix, $d\mathcal{O}$, evaluated at a nominal state, $x_0$.


贺一家博士给的总结：

<p align="center">
  <img src="/img/post/observability_consistency/obs_vs_hessian00.jpeg">
  <img src="/img/post/observability_consistency/obs_vs_hessian01.png">
</p>

## NEES (normalized estimation error squared)

* NEES closer to 6 for VIO


# Inconsistency of Estimator

> a state estimator is consistent if the estimation errors (i) are zero-mean, and (ii) have covariance matrix smaller or equal to the one calculated by the filter.

## Degeneracy (Insufficient Restraint) / Inconsistency in SLAM

### Motion

* constant acceleration
* pure translation

### Structure

## Maintain(Solve) Consistency(Inconsistency)

* [open_vins #171](https://github.com/rpng/open_vins/issues/171): Consistency maintenance methods, FEJ vs Observability-constrained(OC) ones

* [MSCKF笔记：可观性问题](https://shisirqxz.com/FEJ.html)
* [如何理解EKF中的consistency？](https://www.zhihu.com/question/59784440)

paper:

* ***VINS on Wheels***
  - Odometry measurements
  - Planar-motion constraints

### FEJ (First-Estimate Jacobians)

* paper: ***A First-Estimates Jacobian EKF for Improving SLAM Consistency***

* **estimation from the first time**
  
  > to ensure that the state transition and Jacobian matrices are evaluated at correct linearization points such that the above observability analysis will hold true
  

<p class="note note-info">
FEJ 算法：不同残差对同一个状态求雅克比时，线性化点必须一致，这样就能避免零空间退化而使得不可观变量变得可观。
</p>

<p align="center">
  <img src="/img/post/observability_consistency/nullspace_disappear.png" style="width:100%"/>
</p>

app:

* OKVIS
* DSO
  - [DSO - First Estimates Jacobian](https://tongling916.github.io/2020/04/24/DSO-First-Estimates-Jacobian/)
* ElasticFusion 改进版 (IROS2107, Stefan Leutenegger)
* OpenVINS
  - [First-Estimate Jacobian Estimators (OpenVINS)](https://docs.openvins.com/fej.html)
  - [OpenVINS (7)- 能观一致性分析和FEJ](https://zhuanlan.zhihu.com/p/101478814)

ref: 

* [如何理解SLAM中的First-Estimates Jacobian？](https://www.zhihu.com/question/52869487)
* [FEJ经典论文中的EKF SLAM的error-state equation的推导过程](https://blog.csdn.net/jessecw79/article/details/82532032)
* [VSLAM之边缘化 Marginalization 和 FEJ (First Estimated Jocobian)](https://blog.csdn.net/Hansry/article/details/104412753)


### FEJ2

TODO


### Observability Constraint (OC)-VINS

<p align="center">
  <img src="/img/post/observability_consistency/oc_vins_alg.png" style="width:60%">
</p>

#### App: OC-MSC-KF

MSCKF-VIO (S-MSCKF):

Modification of the State Transition Matrix $\Phi$

```cpp
// Modify the transition matrix to make the observability matrix have proper null space
Matrix3d R_kk_1 = quaternionToRotation(imu_state.orientation_null);
Phi.block<3, 3>(0, 0) = quaternionToRotation(imu_state.orientation) * R_kk_1.transpose();

Vector3d u = R_kk_1 * IMUState::gravity;
RowVector3d s = (u.transpose() * u).inverse() * u.transpose();

Matrix3d A1 = Phi.block<3, 3>(6, 0);
Vector3d w1 = skewSymmetric(imu_state.velocity_null - imu_state.velocity) * IMUState::gravity;
Phi.block<3, 3>(6, 0) = A1 - (A1 * u - w1) * s;

Matrix3d A2 = Phi.block<3, 3>(12, 0);
Vector3d w2 = skewSymmetric(dtime * imu_state.velocity_null + imu_state.position_null - imu_state.position) * IMUState::gravity;
Phi.block<3, 3>(12, 0) = A2 - (A2 * u - w2) * s;
```

Modification of the Measurement Jacobian $H$

```cpp
// Modifty the measurement Jacobian to ensure observability constrain. Ref: OC-VINS
Matrix<double, 4, 6> A = H_x;
Matrix<double, 6, 1> u = Matrix<double, 6, 1>::Zero();
u.block<3, 1>(0, 0) = quaternionToRotation(cam_state.orientation_null) * IMUState::gravity;
u.block<3, 1>(3, 0) = skewSymmetric(p_w - cam_state.position_null) * IMUState::gravity;
H_x = A - A * u * (u.transpose() * u).inverse() * u.transpose();
H_f = -H_x.block<4, 3>(0, 3);
```


# Gauge Freedom Handling

> It is well known that visual-inertial systems have four degrees of freedom that are not observable: the global position and the rotation around gravity. These unobservable degrees of freedom (called gauge freedom) have to be handled properly in visual-inertial state estimation **to obtain a unique state estimate**.

* paper: *On the Comparison of Gauge Freedom Handling in Optimization-based Visual-Inertial State Estimation*
* code: [Covariance Transformation for Visual-Inertial Systems](https://github.com/uzh-rpg/rpg_vi_cov_transformation)

H有正确的零空间，比如，对于单目VO，rank(N(H)) = 7，则H为奇异矩阵，那么增量方程始终存在病态或求解不稳定问题；通过处理 规范自由度 解决。


In optimization-based methods, three approaches are usually used:

* **Gauge Fixation**: fixing the initial state,
* **Gauge Prior**: adding a prior to the initial state,
* **Free Gauge**: allowing the parameters to evolve freely during optimization.

<p align="center">
  <img src="/img/post/observability_consistency/gauge_freedom_handling.png">
</p>

<p align="center">
  <img src="/img/post/observability_consistency/gauge_freedom_deal.png">
</p>

ref:

- [VI中的几种自由度处理方法的性能对比](http://epsilonjohn.club/2020/04/06/SLAM%E4%BB%A3%E7%A0%81%E8%AF%BE%E7%A8%8B/VI%E4%B8%AD%E7%9A%84%E5%87%A0%E7%A7%8D%E8%87%AA%E7%94%B1%E5%BA%A6%E5%A4%84%E7%90%86%E6%96%B9%E6%B3%95%E7%9A%84%E6%80%A7%E8%83%BD%E5%AF%B9%E6%AF%94/)
- [手写VIO课后作业（五）](https://codeantenna.com/a/qUUBuna1Bx)


# FAQ

* 为什么位置不可观，对于单目VO，第一帧不是固定住了吗？


[^1]: [一文看尽4种SLAM中零空间的维护方法](https://zhuanlan.zhihu.com/p/341322063)
[^2]: [(17)李导数与李括号](https://zhuanlan.zhihu.com/p/33381200)
[^3]: [可观察性（observability）](https://zh.m.wikipedia.org/zh-hans/%E5%8F%AF%E8%A7%80%E6%B8%AC%E6%80%A7)
[^4]: [通过能观性分析理解SLAM系统的可观维度](https://blog.csdn.net/Walking_roll/article/details/120612989)