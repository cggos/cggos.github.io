---
title: 百度Apollo课程学习笔记
tags:
  - Autonomous Driving
  - Baidu Apollo
categories:
  - Robotics
key: baidu-apollo-note
abbrlink: efc8f2a
date: 2019-07-27 00:00:00
---

[TOC]

# Overview

<p align="center">
  <iframe src="//player.bilibili.com/player.html?bvid=BV1oA411W7uz&page=1"
    width="780" height="480" frameborder="no" scrolling="no" allowfullscreen="true">
  </iframe>
</p>

* the note is from [Apollo Self-Driving Car Lesson](http://apollo.auto/devcenter/devcenter.html)

# Self-Driving Overview

* why need self-driving car  
  ![](/img/post/apollo/why_need.png)

* 5 driving levels  
  ![](/img/post/apollo/driving-level.png)

* self-driving car history
  ![](/img/post/apollo/self-driving-history.png)

* how self-driving car work  
  ![](/img/post/apollo/5-components.png)

* hardware  
  ![](/img/post/apollo/car_hardware.png)

* Open Sofware Stack
  - RTOS: Ubuntu + Apollo Kernel
  - ROS
  - Decentralization: No ROS Master Scheme
  - Protobuf

* Cloud Services
  - HD Map
  - Simulation
  - Data Platform
  - Security
  - OTA(Over-The-Air) updatea
  - DuerOS


# High-Definition Map

* 3d representation of the road
* centimeter-level precision
* localization: data match
* OpenDRIVE standard

* HD map construction  
  ![](/img/post/apollo/HD-map-construction.png)

* HD map crowdsourcing  
  ![](..//img/post/apollo/HD-map-crowdsourcing.png)

# Localization

* need 10 centi-meter accuracy, but GPS error 1-3 meter

* localization  
  ![](/img/post/apollo/localization.png)

* GNSS RTK  
  ![](/img/post/apollo/gps_rtk.png)

* Inertial Navigation
  - accelerator
  - gyroscope

* LiDAR localization
  - ICP
  - filter: Histogram
  - advantage: robust

* visual localization
  - match

* multi-sensor fusion: kalman filter: prediction(Inertial) and update(GNSS LiDAR)  
    ![](/img/post/apollo/localization_kf.png)

# Perception

* perception overview  
  ![](/img/post/apollo/perception_overview.png)

* classification pipeline  
  ![](/img/post/apollo/classification_pipeline.png)

* Camera images

* LiDAR images

* Machine Learning
  - born in 1960s
  - supervised learning
  - unsupervised learning

* Neural Network

* Backpropagation

* Convolutional Neural Network

* Detection and Classification

* Tracking

* Segmentation  
  ![](/img/post/apollo/full_cnn.png)

* Sensor Data Comparision  
  ![](/img/post/apollo/sensor_comparision.png)

* Perception Fusion

# Prediction

* realtime & accuracy
* approaches: model-based & data-driven  
  ![](/img/post/apollo/prediction_approaches.png)

* Trajectory Generation
  - polynomial model

# Planning

* goal: find the best path from A to B on the map
* input: map, our position and destination

* World to Graph

* A star algorithm

* 3D trajectory  
  ![](/img/post/apollo/3D_trajectory.png)

* Evaluating a Trajectory  
  ![](/img/post/apollo/evaluate_trajectory.png)

* Frenet Coordinates

* Path-Velocity Decoupled Planning

# Control

* steering, acceleration and brake  
  ![](/img/post/apollo/control_input.png)

* PID

* LQR

* MPC
