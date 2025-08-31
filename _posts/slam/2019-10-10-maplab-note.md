---
title: Maplab 学习笔记
tags:
  - Visual SLAM
  - VIO
categories:
  - SLAM
key: slam-maplab-note
abbrlink: 9c7e06
date: 2019-10-10 00:00:00
---

[TOC]

## Overview

* [ethz-asl/maplab](https://github.com/ethz-asl/maplab/): An open visual-inertial mapping framework.

## Maplab Framework

<p align="center">
  <img src="https://cloud.githubusercontent.com/assets/966785/6546069/2a4ce0b2-c5a4-11e4-8613-ddcf5e8b0591.png">
</p>

## Typical workflow

<p align="center">
  <img src="https://github.com/ethz-asl/maplab/wiki/images/diagrams/maplab_dataflow.png">
</p>

### Online ROVIOLI frontend

<p align="center">
  <img src="https://github.com/ethz-asl/maplab/wiki/images/rovioli-overview.png">
</p>

### Offline maplab console

* a convenient **console user interface** and a **map manager** to access the maps,
* a **plug-in architecture** to easily extend it with new commands and algorithms,
* **visual-inertial least-squares optimization** that can be extended with additional sensors,
* robust pose-graph relaxation using switchable constraints,
* BRISK/FREAK-based **loopclosure**,
* **map summarization** for lifelong mapping,
* **dense reconstruction** and an interface to Voxblox.
