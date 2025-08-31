---
title: Camera Models Summary
tags:
  - Computer Vision
  - Camera
  - Camera Model
  - Fisheye Camera
  - Visual SLAM
categories:
  - Computer Vision
index_img: /img/post/camera_model/perspective_fisheye_imaging.png
key: camera-models-overview
abbrlink: c60702bf
date: 2019-01-26 00:00:00
---

[TOC]

# Overview

<p align="center">
  <img src="/img/post/camera_model/cam_imaging.png">
</p>

## Lens Projections

* [About the various projections of the photographic objective lenses](http://michel.thoby.free.fr/Fisheye_history_short/Projections/Various_lens_projection.html)

* [Models for the various classical lens projections](http://michel.thoby.free.fr/Fisheye_history_short/Projections/Models_of_classical_projections.html)

* [鱼眼相机成像、校准和拼接(笔记)](http://blog.sciencenet.cn/blog-465130-1052526.html)

* [Computer Generated Angular Fisheye Projections](http://paulbourke.net/dome/fisheye/)

![](/img/post/camera_model/five_various_theoretical_classical_projections_01.jpg)
![](/img/post/camera_model/five_various_theoretical_classical_projections_02.jpg)

* Perspective and fisheye imaging process

<p align="center">
  <img src="/img/post/camera_model/perspective_fisheye_imaging.png">
</p>

## Optics: Terminology

* Dioptric: All elements are refractive (lenses)
* Catoptric: All elements are reflective (mirrors)
* Catadioptric: Elements are refractive and reflective (mirrors + lenses)


# Pinhole Camera

* **pinhole model (rectilinear projection model) + radial-tangential distortion**

$$
K = 
\left[\begin{array}{ccc}
f_x & 0 & c_x \\
0 & f_y & c_y \\
0 & 0 & 1
\end{array}\right]  
$$

$$
D = \left[ k_1, k_2, p_1, p_2 \right]
$$

The Pinhole camera model is the most common camera model for consumer cameras. In this model, the image is mapped onto a plane through **perspective projection**. The projection is defined by the camera intrinsic parameters such as focal length, principal point, aspect ratio, and skew.

# Fisheye / Omnidirectional Camera [^4] [^7] [^8]

## OpenCV (Equidistant / KannalaBrandt) fisheye camera model [^9] [^10] :smile:

* **pinhole model (rectilinear projection model) + fisheye distortion**

$$
K = 
\left[\begin{array}{ccc}
f_x & 0 & c_x \\
0 & f_y & c_y \\
0 & 0 & 1
\end{array}\right]  
$$

$$
D = \left[ k_1, k_2, k_3, k_4 \right]
$$

The Fisheye camera model is a camera model utilized for wide field of view cameras. This camera model is neccessary because **the pinhole perspective camera model is not capable of modeling image projections as the field of view approaches 180 degrees**.

### Pinhole (Rectilinear) Projection

$$
r = f \cdot \tan(\theta)
$$

Given a point $ X=[x_c \quad  y_c \quad  z_c] $ from **the camera $z_c=1$ plane** in camera coordinates, the **pinhole projection** is:

$$
\begin{cases}
r = \sqrt{x_c^2 + y_c^2} \\
\theta = \arctan(r, |z_c|) = \arctan(r, 1) = \arctan(r)
\end{cases}
$$

### Fisheye (Equidistant) Projection

$$
r = f \cdot \theta
$$

**fisheye distortion**:

$$
\theta_d =
\theta (1 + k1 \cdot \theta^2 + k2 \cdot \theta^4 +
k3 \cdot \theta^6 + k4 \cdot \theta^8)
$$

The distorted point coordinates are

$$
\begin{cases}
x_d = \frac{\theta_d}{r} \cdot x_c \\
y_d = \frac{\theta_d}{r} \cdot y_c
\end{cases}
$$

convert into pixel coordinates, the final pixel coordinates vector (assume $\alpha=0$)

$$
\begin{cases}
u = f_x \cdot x_d + c_x \\
v = f_y \cdot y_d + c_y
\end{cases}
$$

write in matrix form

$$
\begin{aligned}
\left[\begin{array}{c}u\\v\\1\end{array}\right] =  
\left[\begin{array}{ccc}
f_x & 0 & c_x \\
0 & f_y & c_y \\
0 & 0 & 1
\end{array}\right]  
\left[\begin{array}{c}x_d\\y_d\\1\end{array}\right]
\end{aligned}
$$

in ORB-SLAM3:

```cpp
Eigen::Vector2f KannalaBrandt8::project(const Eigen::Vector3f &v3D) {
    const float x2_plus_y2 = v3D[0] * v3D[0] + v3D[1] * v3D[1];
    const float theta = atan2f(sqrtf(x2_plus_y2), v3D[2]);
    const float psi = atan2f(v3D[1], v3D[0]);

    const float theta2 = theta * theta;
    const float theta3 = theta * theta2;
    const float theta5 = theta3 * theta2;
    const float theta7 = theta5 * theta2;
    const float theta9 = theta7 * theta2;
    const float r = theta + mvParameters[4] * theta3 + mvParameters[5] * theta5
                     + mvParameters[6] * theta7 + mvParameters[7] * theta9;

    Eigen::Vector2f res;
    res[0] = mvParameters[0] * r * cos(psi) + mvParameters[2];
    res[1] = mvParameters[1] * r * sin(psi) + mvParameters[3];

    return res;
}
```


## ATAN model

* **pinhole model (rectilinear projection model) + FOV distortion**

$$
K = 
\left[\begin{array}{ccc}
f_x & 0 & c_x \\
0 & f_y & c_y \\
0 & 0 & 1
\end{array}\right]  
$$

$$
D = \left[ \omega \right]
$$

Used in

* PTAM [^2]
* LSD-SLAM

This is an alternative representation **for camera models with large radial distortion (such as fisheye cameras) where the distance between an image point and principal point is roughly proportional to the angle between the 3D point and the optical axis**. This camera model is first proposed in ***Straight Lines Have to be Straight: Automatic Calibration and Removal of Distortion from Scenes of Structured Environments***.

Given a point $ X=[x_c \quad  y_c \quad  z_c] $ from **the camera $z_c=1$ plane** in camera coordinates, the **pinhole projection** is:

$$
r = \sqrt{\frac{x_c^2 + y_c^2}{z_c^2}} = \sqrt{x_c^2 + y_c^2}
$$

**FOV distortion**:

$$
r_d = \frac{1}{\omega} \arctan( 2 \cdot r \cdot \tan(\frac{\omega}{2}) )
$$

where $\omega$ is the **FOV distortion coefficient**

The distorted point coordinates are

$$
\begin{cases}
x_d = \frac{r_d}{r} \cdot x_c \\
y_d = \frac{r_d}{r} \cdot y_c
\end{cases}
$$

convert into pixel coordinates, the final pixel coordinates vector

$$
\begin{aligned}
\left[\begin{array}{c}u\\v\\1\end{array}\right] =  
\left[\begin{array}{ccc}
f_x & 0 & c_x \\
0 & f_y & c_y \\
0 & 0 & 1
\end{array}\right]  
\left[\begin{array}{c}x_d\\y_d\\1\end{array}\right]
\end{aligned}
$$

in PTAM

```cpp
// Project from the camera z=1 plane to image pixels,
// while storing intermediate calculation results in member variables
Vector<2> ATANCamera::Project(const Vector<2>& vCam){
    mvLastCam = vCam;
    mdLastR = sqrt(vCam * vCam);
    mbInvalid = (mdLastR > mdMaxR);
    mdLastFactor = rtrans_factor(mdLastR);
    mdLastDistR = mdLastFactor * mdLastR;
    mvLastDistCam = mdLastFactor * mvLastCam;

    mvLastIm[0] = mvCenter[0] + mvFocal[0] * mvLastDistCam[0];
    mvLastIm[1] = mvCenter[1] + mvFocal[1] * mvLastDistCam[1];

    return mvLastIm;
}

// Un-project from image pixel coords to the camera z=1 plane
// while storing intermediate calculation results in member variables
Vector<2> ATANCamera::UnProject(const Vector<2>& v2Im)
{
    mvLastIm = v2Im;
    mvLastDistCam[0] = (mvLastIm[0] - mvCenter[0]) * mvInvFocal[0];
    mvLastDistCam[1] = (mvLastIm[1] - mvCenter[1]) * mvInvFocal[1];
    mdLastDistR = sqrt(mvLastDistCam * mvLastDistCam);
    mdLastR = invrtrans(mdLastDistR);
    double dFactor;
    if(mdLastDistR > 0.01)
        dFactor =  mdLastR / mdLastDistR;
    else
        dFactor = 1.0;
    mdLastFactor = 1.0 / dFactor;
    mvLastCam = dFactor * mvLastDistCam;
    return mvLastCam;
}
```


## MEI (Unified Camera Model) [^3]

![](/img/post/camera_model/mei.png)

## PolyFisheye [^3]

![](/img/post/camera_model/polyfisheye.png)

## EUCM [^5] [^6] :smile:

<p align="center">
  <img src="/img/post/camera_model/eucm.png" style="width:60%">
</p>

* **相机坐标系**： 真实世界坐标中的位置坐标，单位为m，一般对应在表达为Ｘ

* **椭球面坐标系**：是一个中转球面，与对应点的世界坐标相差一个scale的系数；也成为Ｐ平面，对应表达为Xp

* **Z=1平面**：是椭球面上的点在z=1平面上的投影，也称为M平面，其x，y值与Xp一样，只是z值固定为1，对应表达为m, 也是归一化平面

* **像素坐标系**： 图片中点像素点坐标，左上角为(0,0)，由 M平面 中的点经过内参矩阵K计算而来；对应表达为p(u,v)


# Models in Code

* [Supported models in Kalibr](https://github.com/ethz-asl/kalibr/wiki/supported-models)

* Distortion Models in ROS (distortion_models.h)
  - **plumb_bob**: a 5-parameter polynomial approximation of radial and tangential distortion
  - **rational_polynomial**: an 8-parameter rational polynomial distortion model
  - **equidistant** (lunar)

* PTAM [^2] : support **ATAN camera model**

* [uzh-rpg/rpg_vikit/vikit_common](https://github.com/uzh-rpg/rpg_vikit/tree/master/vikit_common/src): support **pinhole, atan and omni camera**

* [ethz-asl/aslam_cv2](https://github.com/ethz-asl/aslam_cv2/tree/master/aslam_cv_cameras/src): support **pinhole and unified peojection** and **radtan, fisheye and equidistant distortion**

* [cggos/okvis_cg](https://github.com/cggos/okvis_cg/tree/master/okvis/okvis_cv/include/okvis/cameras/implementation): support **pinhole peojection** and **radtan and equidistant distortion**

* VINS-Mono: Pinhole, Fisheye (Kannala-Brandt, MEI)
  * [hengli/camodocal](https://github.com/hengli/camodocal)
    - Pinhole camera model
    - Unified projection model
    - Equidistant fish-eye model

* OpenVINS: Pinhole, Fisheye (Kannala-Brandt)

* ORB-SLAM3: Pinhole, Fisheye (Kannala-Brandt)


# Impact of Model Changes on vSLAM

Eg. Pinhole --> EUCM，将去畸变坐标变成了椭球面坐标

* 投影和反投影
  - Pinhole: 像素坐标系 to 归一化平面坐标系
  - EUCM: 像素坐标系 to 椭球面坐标系

* 三角化
  - Pinhole: 去畸变 归一化平面坐标系
  - EUCM: 椭球面坐标系

* 计算基本矩阵和本质矩阵

* 优化 (残差 或 雅可比矩阵)
  - Pinhole: 去畸变 像素坐标系 或 归一化平面坐标系
  - EUCM: 像素坐标系 或 椭球面坐标系 (或 平面坐标系)


# Papers

* *Straight Lines Have to be Straight: Automatic Calibration and Removal of Distortion from Scenes of Structured Environments*

* *A Generic Camera Model and Calibration Method for Conventional, Wide-Angle, and Fish-Eye Lenses*

* *Single View Point Omnidirectional Camera Calibration from Planar Grids*

* *The Double Sphere Camera Model* [^1]



[^1]: [Double Sphere Camera Model (TUM)](https://vision.in.tum.de/research/vslam/double-sphere)

[^2]: [ptam_cg/src/ATANCamera.cc](https://github.com/cggos/ptam_cg/blob/master/src/ATANCamera.cc)

[^3]: [鱼眼相机模型与标定与重映射](https://epsavlc.github.io/2019/10/21/fisheye_calib.html)

[^4]: [鱼眼镜头的成像原理到畸变矫正（完整版）](https://blog.csdn.net/qq_16137569/article/details/112398976)

[^7]: [鱼眼镜头是怎么「鱼眼」的？](https://zhuanlan.zhihu.com/p/29273352)

[^8]: [VIO标定（二）广角相机模型](https://zhuanlan.zhihu.com/p/93822726)

[^5]: [EUCM鱼眼相机模型详解](https://blog.csdn.net/qq_25458977/article/details/100084401)

[^6]: [鱼眼相机模型EUCM（一）](https://zhuanlan.zhihu.com/p/138298495)

[^9]: [Fisheye camera model (OpenCV)](https://docs.opencv.org/master/db/d58/group__calib3d__fisheye.html)

[^10]: [FisheyeCameraModel (theia-sfm)](http://www.theia-sfm.org/cameras.html#fisheyecameramodel)