---
title: Mesh Texturing in a Nutshell (Let There Be Color)
tags:
  - Mapping
  - 3D Reconstruction
  - Texturing
  - MAP-MRF
  - Dynamic Programming
  - Belief Propagation
  - Graph
categories:
  - Mapping
index_img: /img/post/texturing/mesh_cut_curve.png
og_img: /img/post/texturing/face_mesh_textured.png
key: Mesh-Texturing-in-a-Nutshell
abbrlink: d001e9db
date: 2022-06-12 00:00:00
---

[TOC]

<style>
.graphviz { 
  display: flex; 
  justify-content: center; 
}
</style>

# Overview

<!-- dot {align="center"} -->

```graphviz 
digraph {
  TV [label="TextureView"];
  TVs [label="N TextureViews"];
  Texturing [style=filled, shape=box];
  ColorImg->TV;
  CamK->TV;
  CamTF->TV;
  TV->TVs;
  TVs->Input;
  TriangleMesh->Input;
  Input->Texturing->Output->TexturedMesh;
}
```

* code (forked): https://github.com/cggos/mvs-texturing
* paper: ***Let There Be Color! Large-Scale Texturing of 3D Reconstructions***
* video: https://www.youtube.com/watch?v=Ie-qLJdmlLI


# 1. Texture Views

```cpp
tex::generate_texture_views()
```

```graphviz
digraph {
  TV [label="TextureView"];
  TV->ColorImg;
  TV->CamK;
  TV->CamTF;
}
```


# 2. Mesh --> MeshInfo

```cpp
tex::prepare_mesh()
```

<p align="center">
  <img src="/img/post/texturing/face_mesh.png" style="width:100%">
</p>

## Check Mesh

```cpp
TriangleMesh::ensure_normals()
```

* Ensure face and vertex normals

## Init `MeshInfo`

```cpp
MeshInfo::initialize()
```

### Create `VertexInfo`

Add faces to their three vertices

```graphviz
digraph {
  vertex -> face1
  vertex -> face2
  vertex -> face3
}
```

### Update `VertexInfo`

Classify each vertex and compute adjacenty info

* Build new, temporary adjacent faces representation `AdjacentFaceList adj_temp` for ordering

```graphviz
digraph {
  face_id [color=green];
  front_vid [color=blue];
  back_vid [color=blue];

  AdjFaceTmp->face_id
  AdjFaceTmp->front_vid
  AdjFaceTmp->back_vid
}
```

```graphviz
graph {
  layout=twopi;
  node [shape=circle];

  v0 [color="red"];
  v1 [color="blue"];
  v2 [color="blue"];

  v0--v1 [color=green];
  v0--v2 [color=green];
  v0--v3;
  v0--v4;
  v1--v2 [color=green];
  v3--v2;
  v3--v4;
  v1--v4;

  overlap=false;
}
```

* Sort adjacent faces by chaining them

  ```cpp
  AdjacentFaceList adj_sorted;
  ```

* update `VertexInfo`

```graphviz
digraph {
  vclass;
  verts [color=blue];
  faces [color=green];

  vinfo->vclass;
  vinfo->verts;
  vinfo->faces;
}
```



# 3. Mesh + MeshInfo --> Adjacency Graph (`UniGraph`)

```cpp
tex::build_adjacency_graph()
```

对于每个 face，将mesh中与其每条 edge 邻接的 face 存入 `adj_faces`；将当前 face 与 `adj_faces` 中每个 face 建立 edge，构建 `UniGraph` 。


```graphviz
graph {
  node [shape=circle];

  f0--f1; f1--f2;
  f0--f3; f3--f4;
  f1--f4;

  overlap=false;
}
```


# 4. View Selection --> Best View Label :smile:

```cpp
tex::calculate_data_costs()

tex::view_selection()
```

## Calculate DataCosts

Calculates the data costs for each face and texture view combination, if the face is visible within the texture view.

```cpp
FaceProjectionInfos face_projection_infos(num_faces);
calculate_face_projection_infos(mesh, texture_views, settings, &face_projection_infos);
postprocess_face_infos(settings, &face_projection_infos, data_costs);
```

### Calculate `FaceProjectionInfo`

<p align="center">
  <img src="/img/post/texturing/sfm_cam.png" style="width:60%">
</p>

```cpp
for (std::uint16_t j = 0; j < static_cast<std::uint16_t>(num_views); ++j) {
  TextureView * texture_view = &texture_views->at(j);
  // get view_pos and view_dir
  for (std::size_t i = 0; i < faces.size(); i += 3) {
    // get face_normal and face_center
    // compute and check viewing_angle
    // get face info
  }
}
```

```graphviz
digraph {
  face_info [style=filled];
  nnn [label="..."];
  face_info_n [style=filled];
  rankdir=LR;
  face_id->face_info;
  face_info->view_id;
  face_info->mean_color;
  face_info->quality;
  face_id->nnn;
  face_id->face_info_n;
}
```

### PostProcess Face Infos

create `hist_qualities::Histogram` using `info.quality`, and get the `upper_bound` when percentile=0.995

<p align="center">
  <img src="/img/post/texturing/hist.jpg" style="width:80%">
</p>

compute data cost

* gmi
* area

```cpp
float normalized_quality = std::min(1.0f, info.quality / percentile);
float data_cost = (1.0f - normalized_quality);
data_costs->set_value(i, info.view_id, data_cost);
```

| DataCost | face0 | face1 | ... | faceN |
|:-:|-|-|-|-|
| view0 |
| view1 |
| ... |
| viewN |


## View Selection

### Data Association

#### Graph `mapmap::Graph<cost_t>`

```graphviz
graph {
  rankdir = LR;
  face_id--adj_face_id [label="weight"];
}
```

#### LabelSet `mapmap::LabelSet<cost_t, simd_w>`

| view id | face0 | face1 | ... | faceN |
|:-:|-|-|-|-|
| view0 |
| view1 |
| ... |
| viewN |

#### Unaries

```cpp
using unary_t = mapmap::UnaryTable<cost_t, simd_w>;
std::vector<unary_t> unaries;
```

| | face_id | label_set | costs |
|-|-|-|-|
| unary0 |
| unary1 |
| ... |
| unaryN |


#### Pairwise

```cpp
using pairwise_t = mapmap::PairwisePotts<cost_t, simd_w>;
pairwise_t pairwise(1.0f);
```

### MAP-MRF :triangular_flag_on_post:

```cpp
mapmap::mapMAP<cost_t, simd_w> solver;
solver.set_graph(&mgraph);
solver.set_label_set(&label_set);
for(std::size_t i = 0; i < graph->num_nodes(); ++i)
    solver.set_unary(i, &unaries[i]);
solver.set_pairwise(&pairwise);
solver.set_logging_callback(display);
solver.set_termination_criterion(&terminate);
solver.optimize(solution, ctr);
```

The aim is to find a **labeling** for X that produces the lowest energy.

#### pairwise MRFs

<p align="center">
  <img src="/img/post/texturing/pairwise_mrf.jpg" style="width:50%">
</p>

* the filled-in circles: **the observed nodes $Y_i$** (face)
* the empty circles: **the "hidden" nodes $X_i$** (view label)

#### MAP --> Minimum Energy

energy/cost function:

$$
\text{energy} (Y, X) = 
\sum_{i} \text{DataCost} (y_i, x_i) + 
\sum_{j = \text{neighbours of i}} \text{SmoothnessCost} (x_i, x_j)
$$

##### Tree MRFs via DP

<p align="center">
  <img src="/img/post/texturing/mapmap_mrf.png" style="width:100%">
</p>


##### LBP

by OpenMVS

<p align="center">
  <img src="/img/post/texturing/LBP_message_passing.png">
</p>


# 5. Create Texture Atlases :smile:

```cpp
tex::generate_texture_patches()

tex::global_seam_leveling()
tex::local_seam_leveling()

tex::generate_texture_atlases()
```

## Generate Texture Patches

Generates texture patches using the graph to determine adjacent faces with the same label.

<p align="center">
  <img src="/img/post/texturing/face_texture_patches.png" style="width:80%">
</p>

## Global / Local Seam Levelling :triangular_flag_on_post:

* paper: *Seamless Mosaicing of Image-Based Texture Maps*

<p align="center">
  <img src="/img/post/texturing/seam_levelling.png" style="width:100%">
</p>

without seam levelling

<p align="center">
  <img src="/img/post/texturing/face_mesh_textured_no_seam_levelling.png" style="width:80%">
</p>

## Texture Atlases

generate `TextureAtlas` from all of `TexturePatch`

<p align="center">
  <img src="/img/post/texturing/face_texture.png">
</p>

# 6. Mesh + Texture --> Obj Model

```cpp
tex::build_model()

tex::Model::save()
```

* .obj
* .mtl
* .png

<p align="center">
  <img src="/img/post/texturing/face_mesh_textured.png" style="width:100%">
</p>


# 网格UV展开

上述纹理重建属于 **计算机视觉** 的内容，本节是其逆过程，属于 **计算机图形学** 的内容。

* http://geometryhub.net/notes/uvunfold


<p align="center">
  <img src="/img/post/texturing/mesh_cut_curve.png" style="width:100%">
</p>


# Reference

* [UV的概念及作用](https://www.zdaiot.com/ImageProcessing/UV%E7%9A%84%E6%A6%82%E5%BF%B5%E5%8F%8A%E4%BD%9C%E7%94%A8/)
* [【Let It Be Color！——3D重建之纹理重建】02-基于映射的纹理重建算法（上）](https://bbs.huaweicloud.com/blogs/195742)
* https://github.com/tyluann/3DTexture
* https://zhuanlan.zhihu.com/p/44424934
