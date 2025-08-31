---
title: 'Intel RealSense Cameras: D435i, T265, ZR300'
tags:
  - Camera
  - Stereo Vision
  - Visual SLAM
categories:
  - Computer Vision
key: intel-realsense-cams
abbrlink: d468fb33
date: 2019-07-25 00:00:00
---

[TOC]

# Overview

* https://www.intelrealsense.com/developers/

* [The basics of stereo depth vision](https://www.intelrealsense.com/stereo-depth-vision-basics/)

* [调整realsense相机以获得最佳性能](https://blog.csdn.net/ahelloyou/article/details/115513850)

* [How-to: Getting IMU data from D435i and T265](https://www.intelrealsense.com/how-to-getting-imu-data-from-d435i-and-t265/)

* [RS Cam related scripts (CCV)](https://github.com/cggos/ccv/tree/master/apps/camkit/rs_cam)


# Install and Run [^2]

## Install SDK

Add Keys

```sh
sudo apt-key adv --keyserver keys.gnupg.net --recv-key C8B3A55A6F3EFCDE

# or
sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key C8B3A55A6F3EFCDE
```

Add Repositories

```sh
# Ubuntu 18.04

# maybe Error
sudo add-apt-repository "deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo bionic main" -u

# ok: Remove the offending APT repo and add the new secure repo
sudo add-apt-repository --remove "deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo bionic main" -u
sudo add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo bionic main" -u
```

Install Libs

```sh
sudo apt-get install librealsense2-dkms
sudo apt-get install librealsense2-utils
```

Install Dev and Debug tools

```sh
sudo apt-get install librealsense2-dev
sudo apt-get install librealsense2-dbg
```

## Install ROS

Install ROS package [^3] [^4]

```sh
sudo apt install ros-melodic-realsense2-camera
```

## Run

```sh
realsense-viewer
```

run with ROS

```sh
roslaunch realsense2_camera rs_camera.launch
```

### run multiple RS Cams

* https://github.com/cggos/ccv/blob/master/apps/camkit/rs_cam/ros/rs_multiple_devices.launch


# RS D435i [^1]

<p align="center">
    <img src="https://www.intelrealsense.com/wp-content/uploads/2020/05/imu_stereo_DT_d435_front-crop1a_cn.png" style="width:60%;">
</p>

<p align="center">
    <img src="https://www.intelrealsense.com/wp-content/uploads/2020/05/d435_inside_depth_camera_cn.jpg" style="width:80%;">
</p>

<p align="center">
    <img src="/img/post/vi_modules/rs_d435i_view.png" style="width:100%;">
</p>


# RS T265 [^5]

## Tech Specs

* **Intel® Movidius™ Myriad™ 2.0 VPU**: Visual Processing Unit optimized to run V‑SLAM at low power
* **Two Fisheye lenses**
  - FOV: close to hemispherical 163±5° field of view
  - Resolution: 840x800
  - Frame Rate: 30 FPS
  - Image Format: Y8
  - sensor: OV9282
* **BMI055 IMU**
  - Gyroscope
    - Frame Rate: 200 FPS
  - Accelerator
    - Frame Rate: 62 FPS
* **infrared cut filter**: The T265 features an infrared cut filter over the lenses, allowing it to ignore the projected patterns from D400 series depth cameras
* pose output
  - Format: 6DOF
  - Frame Rate: 200 FPS

<p align="center">
    <img src="/img/post/vi_modules/t265_view.png" style="width:100%;">
</p>


## Test

### Results

* 相机前视，运行正常，回到原点
  <p align="center">
      <img src="/img/post/vi_modules/t265_trajectory_01.png" style="width:100%;">
  </p>
  <p align="center">
      <img src="/img/post/vi_modules/t265_trajectory_02.png" style="width:100%;">
  </p>

* 相机下视，运行正常，最终离原点误差 0.8m 左右
  <p align="center">
      <img src="/img/post/vi_modules/t265_trajectory_downview.png" style="width:100%;">
  </p>

* 快速运动（某些地方来回快速挥动），正常运行，不飞，最终离原点误差 2m 左右
  <p align="center">
      <img src="/img/post/vi_modules/t265_trajectory_hightspeed.png" style="width:100%;">
  </p>

* 遮挡右目，正常运行，最终离原点误差 0.7m 左右
  <p align="center">
      <img src="/img/post/vi_modules/t265_trajectory_only_left.png" style="width:100%;">
  </p>

* 遮挡左目，正常运行，最终离原点误差 2m 左右
  <p align="center">
      <img src="/img/post/vi_modules/t265_trajectory_only_right.png" style="width:100%;">
  </p>

### Conclusion

* pros

  * 相机前视，正常运动，回到原点
  * 快速运动（手快速来回挥动）不会飞，不过最终到原点的误差会差些
  * 相机下视（看到的都是地毯），也会正常运行，不过最终到原点的误差会差些
  * 遮挡任一相机（单目状态），也会正常运行，不过最终到原点的误差会差些

* cons

  * 回环检测（貌似没有，待确定）
  * 运行中静止一段时间，有时SLAM挂掉
  * SLAM挂掉后有时需要重新插拔T265后才能正常启动


# RS ZR300 [^6]

* [安装&运行 ZR300驱动](https://cgabc.xyz/cl6cbqtby0079tqdjcuxs7kt6/#%E5%AE%89%E8%A3%85%E8%BF%90%E8%A1%8C-zr300%E9%A9%B1%E5%8A%A8)


[^1]: [深度摄像头 D435i](https://www.intelrealsense.com/zh-hans/depth-camera-d435i/)

[^2]: [Intel RealSense D435i:简介、安装与使用(ROS、Python)](https://zhaoxuhui.top/blog/2020/09/09/intel-realsense-d435i-installation-and-use.html)

[^3]: [从零开始使用Realsense D435i运行VINS-Mono](https://blog.csdn.net/weixin_44580210/article/details/89789416)

[^4]: [Realsense D435i 在ubuntu上安装SDK与ROS Wrapper 运行ORB-SLAM2、RTAB和VINS-Mono](https://blog.csdn.net/qq_41839222/article/details/86503113)

[^5]: [Intel RealSense Tracking Camera T265](https://www.intelrealsense.com/tracking-camera-t265)

[^6]: [realsense_sdk_zr300](https://github.com/IntelRealSense/realsense_sdk_zr300)
