---
title: (Paper Reading) A Novel Feedback Mechanism-Based Stereo Visual-Inertial SLAM
tags:
  - Visual SLAM
  - VIO
  - MSCKF
  - Loop Closure
  - Paper Reading
categories:
  - SLAM
key: slam-stereo-msckf-vio-feedback
abbrlink: ef371053
date: 2021-12-12
---

## Overview

<p align="center">
  <img src="/img/post/msckf/feedback/vislam_feedback_mechanism.png" style="width:60%"/>
</p>

- the filtering-based frontend VIO
- the optimization-based backend with loop closure
- the state feedback

> Inspired by the idea of [38], we feed the optimized state produced by the backend back to the frontend to correct the state estimation

> The frontend is used for pose estimation and map construction, whereas the backend is responsible for loop closure detection and pose optimization.

## FRONTEND VIO

## BACKEND OPTIMIZATION

> The local BA optimizes the last N frames in the sliding window and all map points seen by those N frames.
> 

## FEEDBACK MECHANISM

- the common KF update rule for the old state
- the nonlinear optimization method for the new state

<p align="center">
  <img src="/img/post/msckf/feedback/state_feedback.png" style="width:90%"/>
</p>

### OLD STATE UPDATE

### NEW STATE UPDATE

> we only use the last keyframe and the new frames for optimization through minimizing the objective function which is the same with (1).
> 

## RELOCALIZATION

<p align="center">
  <img src="/img/post/msckf/feedback/reloc.png" style="width:60%"/>
</p>

<p align="center">
  <img src="/img/post/msckf/feedback/opti.png" style="width:90%"/>
</p>

> Then N keyframes before and after the aligned keyframe are selected to update the local map points by retrieving the points corresponding to the keyframes in the built map.
>