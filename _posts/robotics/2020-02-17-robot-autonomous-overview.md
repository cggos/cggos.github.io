---
layout: article
title:  "机器人自主定位导航 概述"
date:   2020-02-17
tags: Robotics
key: robot-autonomous-localization-navigation-overview
---

<div>{%- include extensions/ted.html id='vijay_kumar_the_future_of_flying_robots' -%}</div>

[TOC]

* [服务机器人究竟是如何实现自主定位导航的？](https://www.slamtec.com/cn/News/Detail/60)

* [在实现自主导航之前，移动机器人都有哪些避障方法？](https://www.leiphone.com/news/201605/tTC7DaH95LwnVIZW.html)

<p align="center">
  <img src="../images/robot/robot_features.png"/>
</p>

## Overview

机器人自主移动的四大核心功能：
* 环境构建
* 自主导航（室内定位、智能避障、路径规划等）
* 智能跟随
* 自主回充

## SLAM & 自主导航

自主导航，从大的方面来讲包括局域导航和全局导航两部分。

* 局域导航是指通过视觉、雷达、超声波等传感器实时获取当前环境信息，提取数据融合后的特征，经智能算法处理后实现当前可通行区域的判断和多目标跟踪
* 全局导航主要指利用GPS提供的全局导航数据进行全局路径规划，并实现全电子地图范围内的路径导航。

**SLAM ≠ 自主定位导航**，不解决运动问题；需要在完成 SLAM 之后，进行一个叫做目标点导航的能力，规划一条从A点到B点的路径出来，然后让机器人移动过去。

SLAM如其名一样，主要解决的是机器人的地图构建和即时定位问题，而自主导航需要解决的是智能移动机器人与环境进行自主交互，尤其是点到点自主移动的问题，这需要更多的技术支持。

**机器人自主定位导航 = SLAM + 路径规划 + 运动控制**。


## 定位

要先知道自己在地图中的位置，才可以进行后续的路径规划。

## 建图

地图构建也是机器人实现自主导航行动的前提。地图一方面可以帮助机器人配合自身的传感器进行实时定位，同时也用于后续展开行动时，导航过程的路径规划。

## 路径规划

运动规划是一个很大的概念，从机械臂的运动、飞行器的飞行，到扫地机的清扫，机器人的移动，其实这些都是属于运动规划的范畴。运动规划主要分为： 全局规划 、 局部规划。

* 全局规划：是最上层的运动规划逻辑，它按照机器人预先记录的环境地图并结合机器人当前位姿以及任务目标点的位置，在地图上找到前往目标点最快捷的路径。

* 局部规划：当环境出现变化或者上层规划的路径不利于机器人实际行走的时候（比如机器人无法按照规划的路径完成特定转弯半径的转向），局部路径规划将做出微调。与全局规划有所区别的是，局部规划可能并不知道机器人最终要去哪，但是对于机器人怎么绕开眼前的障碍物特别在行。

运动规划的过程中还包含静态地图和动态地图两种情况。

* A*（A-Star)算法是一种 静态 路网中求解最短路径最有效的直接搜索方法，也是解决许多搜索问题的有效算法。算法中的距离估算值与实际值越接近，最终搜索速度越快。但是，A* 算法同样也可用于动态路径规划当中，只是当环境发生变化时，需要重新规划路线。

* D* 算法则是一种 动态 启发式路径搜索算法，它事先对环境位置，让机器人在陌生环境中行动自如，在瞬息万变的环境中游刃有余。D* 算法的最大优点是不需要预先探明地图，机器人可以和人一样，即使在未知环境中，也可以展开行动，随着机器人不断探索，路径也会时刻调整。


<p align="center">
  <img src="../images/robot/path_plan.jpg"/>
</p>

### 空间覆盖（space coverage）

扫地机器人所需要的功能跟市面上的机器人有所不同，比如针对折返的工字形清扫，如何有效进行清扫而不重复清扫？如何让扫地机和人一样，理解房间、门、走廊这种概念？

针对这些问题，学术界长久以来有一个专门的研究课题，叫做空间覆盖（space coverage），同时也提出了非常多的算法和理论。其中，比较有名的是Morse Decompositions，扫地机通过它实现对空间进行划分，随后进行清扫。

所以，他要实现的不是尽快实现从A到B的算法，为了家里能尽量扫得干净，要尽量覆盖从A到B点的所有区域，实现扫地机器人“扫地”的这个功能。

## 运动控制
