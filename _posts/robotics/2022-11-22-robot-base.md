---
title: 移动机器人常见底盘及其运动学
categories:
  - Robotics
tags:
  - Kinematics
  - RobotBase
index_img: /img/post/robot_base/2wd.jpg
key: robot-base
abbrlink: '93950854'
date: 2022-11-22 00:00:00
---

# Overview [^1]

通常移动机器人依赖电机驱动车轮实现行走功能。机器人底盘结构不同，其运动学也完全不同。根据不同类型车轮，常见的底盘结构差速运动模型、滑移运动模型、阿克曼运动模型、全向轮运动模型等等。

<p align="center">
  <img src="/img/post/robot_base/base_var.png" style="width:80%"/>
</p>

下图中，（a）双轮差速式机器人，（b）阿克曼式机器人，（c）四轮驱动机器人，（d）双履带式机器人，（e）麦克纳姆轮全向机器人，（f）全向轮全向机器人，（g）四轮驱动四轮转向机器人 [^2]

<p align="center">
  <img src="/img/post/robot_base/base_var0.jpg" style="width:48%"/>
  <img src="/img/post/robot_base/base_var1.jpg" style="width:48%"/>
</p>

ROS 中运动学分析为正解（Forward kinematics）和逆解（Inverse Kinematics）两种。

* **正解 或 正运动学模型（forward kinematic model）**：是将获得的机器人底盘速度指令 /cmd_vel 转化为每个车轮的实际速度。
* **逆解 或 逆运动学模型（inverse kinematic model）**：是根据电机编码器获得的每个车轮速度计算出机器人底盘速度，从而实现航迹推算。

# 两轮差速运动模型 (Differential Drive robot)

双轮差速式机器人的两个动力轮设置在底盘左右两侧，两轮速度可独立控制，通过给定不同速度实现底盘的直线和转向控制。为保持平衡，底盘一般会配有一到两个辅助支撑的万向轮，从而形成三轮或四轮的轮系结构。

ROS自带的DWA路径规划算法比较适合 [^3]，why？

<p align="center">
  <img src="/img/post/robot_base/2wd.jpg" style="width:50%"/>
</p>

<p align="center">
  <img src="/img/post/robot_base/2wd_var.png" style="width:60%"/>
</p>

## Forward Kinematic Model [^3]

一个驱动轮的速度

车体中心的速度

$$
v = \frac{v_l + v_r}{2}
$$

求得小车的近似瞬间速度 $v$ 后，以世界坐标系为原点，对 $v$ 进行积分，即可得到机器人在世界坐标系中的位姿 $P$

$$
\dot P = 
\begin{bmatrix} c & -s & 0 \\ s & c & 0 \\ 0 & 0 & 1 \end{bmatrix}
\begin{bmatrix} v_x \\ v_y \\ v_\theta \end{bmatrix}
$$

$$
P_1 = P_0 + \dot P \cdot \Delta t
$$


```cpp
//compute odometry in a typical way given the velocities of the robot
double dt = (current_time - last_time).toSec();

double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
double delta_th = vth * dt;

x += delta_x;
y += delta_y;
th += delta_th;
```


## Backward Kinematic Model


# 四轮阿克曼底盘 (Four-wheeled Ackerman robot)

阿克曼式机器人为四轮式，它的原理与汽车相似，由两后轮作为驱动轮提供动力，由两前轮作为转向轮控制方向，且两前轮的转角通过阿克曼转向机构关联。由于采用了与汽车相似的构造，阿克曼式机器人操纵性与汽车类似。


# 四轮滑移底盘（Four-wheel sliding robot）

四轮驱动机器人的四个直行轮大小相同、独立驱动且前后、左右对称布置，依靠左右侧直行轮的速度差实现转向。在转弯过程中，四轮驱动机器人是靠滑动摩擦实现的，因此会对直行轮及地面造成一定的磨损。因为存在严重的滑移情况，四轮驱动机器人难以精确控制。


# 四轮驱动四轮转向机器人

四轮驱动四轮转向机器人（4WD-4WS）相当于有8个电机在控制其运动，可轻松实现机器人的全向运动，具有机构简单、行动灵活、效率高等特点，在室外非结构化场景下具有较强的自适应能力。然而，随着电机数量的增加，对控制的精确性、同步性提出了更高的要求，在一定程度上加大了控制难度。


# 全向移动底盘（omnidirectional wheel robot）

<p align="center">
  <img src="/img/post/robot_base/omni_driver.png" style="width:80%"/>
</p>

这类机器人相对比较特殊，车轮采用了 **麦克纳姆轮或全向轮**，按照一定的规律控制车轮转动，则可以实现前、后、左、右四个方向的全向移动，比起非全向移动机器人，其灵活性更好，能够在狭窄的区域运动。但由于受到麦克纳姆轮或全向轮的限制，该类机器人的承载能力不大。另外，全向移动机器人的各个车轮产生的力会相互抵消一部分，因此同样转矩产生的净推力效率较低，综合效率不如差速式机器人。


# 双履带式

双履带式机器人底盘左右两侧各配置一套履带移动机构。每套履带移动机构由轮系、悬挂系统和履带组成。轮系包含若干驱动轮、支重轮、导向轮、托带轮；悬挂系统一般采用克里斯蒂悬挂，以保障越障性能良好；履带一般由强度高、重量轻、模量高、无收缩的复合材料制成。双履带式机器人的越障性能优良，在室外复杂环境中有较多应用。


[^1]: [原理篇：机器人常见底盘运动学分析](https://xxty847.github.io/2020/02/02/%E5%8E%9F%E7%90%86%E7%AF%87%EF%BC%9A%E6%9C%BA%E5%99%A8%E4%BA%BA%E5%B8%B8%E8%A7%81%E5%BA%95%E7%9B%98%E8%BF%90%E5%8A%A8%E5%AD%A6%E5%88%86%E6%9E%90/)

[^2]: [移动机器人技术漫谈（二）：移动装置构型](http://www.robotious.com/kj/llyl/409.html)

[^3]: [分析总结常见的几种移动机器人底盘类型及其运动学](https://blog.csdn.net/qq_24706659/article/details/88342626)
