---
title: Data Association in MSCKF
tags:
  - Visual SLAM
  - VIO
  - MSCKF
  - Data Association
  - DSA
  - Graph
categories:
  - SLAM
key: msckf-data-association
abbrlink: c92888db
date: 2020-10-10 00:00:00
---

[TOC]

## MSCKF

```cpp
typedef
std::map<FeatureIDType, Feature, std::less<int>,
  Eigen::aligned_allocator<std::pair<const FeatureIDType, Feature> > > MapServer;

struct Feature {
  FeatureIDType id;

  Eigen::Vector3d position;

  std::map<StateIDType, Eigen::Vector4d, std::less<StateIDType>,
    Eigen::aligned_allocator<std::pair<const StateIDType, Eigen::Vector4d> > > observations;
};                
```

<p align="center">
  <img src="/img/post/msckf/post_ekf_update.png" style="width:80%;"/>
</p>
