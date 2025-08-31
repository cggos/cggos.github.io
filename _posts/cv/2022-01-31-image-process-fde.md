---
title: 图像频率域分析之频域谱(FDE)
tags:
  - Computer Vision
  - DIP
  - Frequency Domain
  - Entropy
categories:
  - Computer Vision
key: image-process-fde
abbrlink: f31ca1ba
date: 2022-01-31 00:00:00
---

[TOC]

# Overview

code: https://github.com/cggos/cvkit/blob/master/scripts/cv_py/fft_fde.py

主要用于：

* 图像模糊度计算
* 镜头对焦

# 频域熵(FDE) 

计算图像的 **频域谱**，表示如下

$$
f(i, j)
$$

将其 **幅度谱** 归一化

$$
f_{\text {norm }}(i, j)=\frac{1}{\sum_{(i, j) \in D}|f(i, j)|}|f(i, j)|
$$

计算 **归一化幅度谱** 的 **信息熵**，即最终的 **FDE**

$$
F D E=-\sum_{(i, j) \in D} f_{\text {norm }}(i, j) \cdot \log \left(f_{\text {norm }}(i, j)\right)
$$

# 图像模糊度

通过计算图像模糊度，我们在SLAM算法中可以

* 检测模糊图像
* 动态改变图像特征点的噪声值

## 高斯模糊图像

通过利用高斯模糊算法将图像模糊程度逐渐增大，其对应的FDE值逐渐减小。

* Entropy: 11.368065834 (Origin)
* Entropy: 10.0918264389 ($\sigma = 1$)
* Entropy: 9.30934810638 ($\sigma = 2$)

![](/img/post/fde/lena_00.png)
![](/img/post/fde/lena_01.png)
![](/img/post/fde/lena_02.png)

* Entropy: 8.96276855469 ($\sigma = 3$)
* Entropy: 8.66108512878 ($\sigma = 5$)
* Entropy: 8.41834259033 ($\sigma = 10$)

![](/img/post/fde/lena_03.png)
![](/img/post/fde/lena_05.png)
![](/img/post/fde/lena_10.png)

# References

* Entropy based measure of camera focus
