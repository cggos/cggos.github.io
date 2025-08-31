---
title: 基于RGBD相机的多模态结构特征融合定位 (Draft)
tags:
  - Multi-Sensor Fusion
  - Visual SLAM
  - Struct SLAM
  - Image Features
  - Parameterization
categories:
  - MSF
index_img: /img/post/msf/multi_modal_struct/opt_reprojection_error_line.png
sticky: 10000
key: msf-multi-modal-struct-features
abbrlink: ec177663
date: 2022-03-06 00:00:00
---

[TOC]

# 概述

目前基于RGBD相机的视觉SLAM系统，都是基于特征点或者稠密点云进行定位，计算量大，而且在无纹理或弱纹理的环境，容易跟丢，鲁棒性较差。本文提出了一种基于RGBD相机的多模态结构化数据融合定位方法，算法主要框架如下图所示，通过充分利用视觉特征，融合二维特征点、二维曲线（边缘点）、二维线段和三维平面等，结合状态估计算法进行实时、鲁邦、准确的位姿估计。在二维特征点的基础上，同时提取边缘特征，使其在弱纹理的环境也能快速、鲁棒定位；另外，在每个关键帧提取2D线段和3D平面等特征，在不增加较多计算量的情况下，引入更多稳定的视觉约束，使位姿估计更准确。

![](/img/post/msf/multi_modal_struct/feats_vo.drawio.png)

* Preprint (RG): [Multimodal Structural Feature Fusion Localization Based on RGBD Camera](https://www.researchgate.net/publication/359392146_Multimodal_Structural_Feature_Fusion_Localization_Based_on_RGBD_Camera)

# 初始化

（1）在第一帧图像提取FAST特征点和进行边缘检测

（2）根据深度图，通过相机投影模型，得到对应的3D路标点，创建初始地图

（3）固定第一帧位姿为单位阵

# 数据降维

我们将输入的图像（灰度图和深度图），进行特征提取，获取一些不变的特征，为后面的状态估计提供数据支持。本算法用到的视觉特征主要包括以下四种：

* 点
  - 2D特征点
* 线
  - 2D线段
  - 2D曲线或边缘点
* 面
  - 3D平面

## 特征提取

### 2D图像特征检测

- 2D特征点：FAST / ORB
- 2D线段：LSD
- 2D曲线或边缘点：Canny边缘检测

![](/img/post/msf/multi_modal_struct/feat_2d_fast.png)
![](/img/post/msf/multi_modal_struct/feat_2d_lsd.png)
![](/img/post/msf/multi_modal_struct/feat_2d_canny.png)

### 3D点云平面提取

![](/img/post/msf/multi_modal_struct/feat_3d_plane.png)

3D平面特征是SLAM系统中减小漂移误差的一种稳定标志。从密集点云中提取平面是一种简单的方法，常用于RGB-D相机或激光雷达。但基于稠密点云提取平面，算力较高，不利于SLAM定位的实时性；我们利用RANSAC方法从上面得到的稀疏路标点中提取3D平面。

（1）根据2D特征点和2D边缘点对应的路标点（见下面三角化章节）作为输入的3D稀疏点云

（2）随机选取三个不共线的点，计算平面参数（质心和法向量，其表示可见下面3D路标表示章节）

（3）计算稀疏点云中每个点到平面的距离，并累加，记录距离累加和、平面参数

（4）重复步骤（2）~（3），获取距离累加和最小时的平面参数

（5）收集内点：计算所有点到该平面的距离，并设定阈值，小于阈值的所有点即为平面点内点

（6）计算所有内点的均值为最终平面的 **质心**

（7）根据均值计算其 **协方差矩阵**，对该矩阵进行奇异值分解，对应奇异值最小的奇异向量即为最终平面的 **法向量**

example code: 

* [extract_domain_plane_ransac.cpp (CCV)
](https://github.com/cggos/ccv/blob/master/libs/pcl/pcl_demo/extract_domain_plane_ransac.cpp)

# 数据关联

我们在提取了2D特征后，需要通过2D特征间的匹配，三角化得到3D路标，并与对应的2D特征进行关联；同时，为了兼顾实时性和高性能，还要提取关键帧。

## 2D特征匹配

- 2D特征点：光流跟踪 / ORB描述子匹配
- 2D线段：LBD描述子匹配
- 2D曲线或边缘点（点云）：通过 **距离变换** 进行 **倒角匹配**
- 3D平面：根据两平面参数（法向量和质心），计算法向量之间的夹角以及质心间的欧式距离，作为平面匹配的相似度，据此进行匹配

## 3D路标及其参数化表示

### 3D路标点

$$
P = [X, Y, Z]^T
$$

（1）2D特征点和2D边缘点对应的3D路标点，初始在深度图中提取

（2）因深度图距离限制、空洞（深度缺失）以及3D路标点质量较差被剔除等因素，需要后续补充3D路标点，通过对极几何的极线搜索算法，三角化出3D路标点，如下图所示

<p align="center">
  <img src="/img/post/msf/multi_modal_struct/landmark_3d_point.png" style="width:60%">
</p>

### 3D直线

<p align="center">
  <img src="/img/post/msf/multi_modal_struct/landmark_3d_line.png" style="width:60%">
</p>

3D空间直线有4个自由度，其表示方式主要有两种：普吕克坐标和正交表示。

**普吕克坐标**

$$
\mathcal{L}=\left(\mathbf{n}^{\top}, \mathbf{v}^{\top}\right)^{\top} 
\in \mathbb{R}^6
$$

**正交表示**

$$
(\mathrm{U}, \mathrm{W}) \in S O(3) \times S O(2) 
\longrightarrow
\left[ \boldsymbol{\theta} \; \theta \right] \in \mathbb{R}^4
$$

通常，为方便表示空间直线，我们使用普吕克坐标；但由于其具有6个变量，多于空间直线的4个自由度，在优化时会出现 **过参数化问题**，因此优化中使用其正交表示。

通过在两个视图图像中提取到的2D线段进行匹配，线段匹配对三角化出空间直线（普吕克坐标表示），并与2D线段进行关联。

<p align="center">
  <img src="/img/post/msf/multi_modal_struct/landmark_3d_line_triangle.png" style="width:60%">
</p>

### 3D平面

欧式空间3D平面有3个自由度，在本算法中对其表示方式主要有两种：Hesse形式、球坐标。

**Hesse形式**

<p align="center">
  <img src="/img/post/msf/multi_modal_struct/landmark_3d_plane_hesse.png" style="width:40%">
</p>

$$
\pi=\left(\mathbf{n}^{\top}, d\right)^{\top} \in \mathbb{R}^4
$$

**球坐标**

<p align="center">
  <img src="/img/post/msf/multi_modal_struct/landmark_3d_plane_sphere.png" style="width:40%">
</p>

$$
\tau=(\theta, \phi, d)^{\top}=
q(\pi)=\left(\theta=\arctan \frac{n_{y}}{n_{x}}, \; \phi=\arcsin n_{z}, \; d\right)^{\top}
$$

一般优化中采用球坐标的表示方式

## 关键帧选取

主要根据时空两大方面，对关键帧进行选取。

（1）时间：经过一定数量的帧数

（2）空间：当当前帧距离上一关键帧的位姿变换超过一定阈值

# 状态估计

## 初始位姿估计

利用2D点特征和3D路标点，通过PNP算法，进行初始位姿估计。

## 后端优化

优化变量为当前帧相机位姿

$$
X = 
\left[ t_x, t_y, t_z, \theta_x, \theta_y, \theta_z \right]^T
$$

定义总体目标函数如下

$$
{X}^{*} =
\arg \min_{X}
\left\{
\sum \rho\left(
\left\|\mathbf{r}_{point}\right\|^{2}
\right)
+
\sum \rho 
\left(
\left\|\mathbf{r}_{edge} \right\|^2 \right)
+
\sum \rho 
\left(
\left\|\mathbf{r}_{line} \right\|^2 \right)
+
\sum \rho 
\left(
\left\|\mathbf{r}_{plane} \right\|^2 \right)
\right\}
$$

### 特征点

<p align="center">
  <img src="/img/post/msf/multi_modal_struct/opt_reprojection_error_point.png" style="width:60%">
</p>

（1）通过FAST或者光流法跟踪在参考关键帧图像上得到2D特征点，并得到与其关联的3D路标点。

（2）根据当前帧与参考帧间的位姿，将上述3D路标点投影到当前帧图像上，作为投影预测值。

（3）在当前帧图像上检测到的2D特征点中，找到与参考帧匹配的2D特征点，作为观测值。

（4）计算观测特征点到投影点的欧式距离，即重投影误差，作为观测残差。

残差定义为

$$
r_{point} = u - K \cdot T_{cr} P_r
$$

### 线段

<p align="center">
  <img src="/img/post/msf/multi_modal_struct/opt_reprojection_error_line.png" style="width:60%">
</p>

（1）通过LSD算法在参考关键帧图像上提取到2D线段特征，并得到与其关联的3D空间直线。

（2）根据当前帧与参考帧间的位姿，将上述3D空间直线投影到当前帧图像上，作为投影预测值。

（3）在当前帧图像上检测到的2D线段中，找到与参考帧匹配的2D线段，作为观测值。

（4）计算观测线段两个端点到投影直线的欧式距离，即线重投影误差，作为观测残差。

残差定义为

$$
r_{line} = D(z, L, T_{cw})
$$

### 曲线或边缘点（ICP）

通过Canny边缘检测算法，在参考帧 $F_{ref}$ 中检测到2D边缘点 $\mathbf{p}_{i}^{\mathcal{F}_{\text {ref}}}$ ，根据相机投影模型和深度图像，得到 3D边缘点，如下

$$
\mathcal{S}^{\mathcal{F}_{\text {ref }}}=\left\{\mathbf{s}_{i}^{\mathcal{F}_{\text {ref }}}\right\}=\left\{d_{i}^{\mathcal{F}_{\text {ref }}} \pi^{-1}\left(\mathbf{p}_{i}^{\mathcal{F}_{\text {ref }}}\right)\right\}
$$

根据当前帧与参考帧间的位姿，将上述3D点投影到当前帧 $F_k$，得到2D点 $\mathbf{o}_{i}^{\mathcal{F}_{k}}$，如下

$$
\mathcal{O}^{\mathcal{F}_{k}}=\left\{\mathbf{o}_{i}^{\mathcal{F}_{k}}\right\}=\left\{\pi\left(\mathbf{R}^{\mathrm{T}}\left(\mathbf{s}_{i}^{\mathcal{F}_{\text {ref }}}-\mathbf{t}\right)\right)\right\}
$$

在当前帧检测到的2D点 $P^{F_K}$ 中找到 $\mathbf{o}_{i}^{\mathcal{F}_{k}}$ 的最近邻点，定义公式如下

$$
n\left(\mathbf{o}_{i}^{\mathcal{F}_{k}}\right)=\underset{\mathbf{p}_{j}^{\mathcal{F}_{k}} \in \mathcal{P}^{\mathcal{F}_{k}}}{\operatorname{argmin}}\left\|\mathbf{p}_{j}^{\mathcal{F}_{k}}-\mathbf{o}_{i}^{\mathcal{F}_{k}}\right\|
$$

残差定义为

$$
r_{edge} = \mathbf{o}_{i}^{\mathcal{F}_{k}}-n\left(\mathbf{o}_{i}^{\mathcal{F}_{k}}\right)
$$

定义目标函数

$$
X^*=\underset{X}{\operatorname{argmin}} \sum_{i=1}^{N}\left\|\mathbf{o}_{i}^{\mathcal{F}_{k}}-n\left(\mathbf{o}_{i}^{\mathcal{F}_{k}}\right)\right\|^{2}
$$

### 平面

（1）通过RANSAC算法在参考关键帧提取世界坐标系的3D平面。

（2）根据参考帧的位姿 $T_{rw}$ 以及当前帧与参考帧间的位姿 $T_{cr}$，将上述3D平面投影到当前帧，作为预测值。

（3）在当前帧提取3D平面，找到与参考帧匹配的3D平面，作为观测值。

（4）计算观测值与预测值之间的误差，作为观测残差。

残差定义为

$$
r_{plane} = 
q(\pi_c^w) - 
q(T_{cr}^{-1} T_{rw}^{-1} \pi_r^w)
$$