<p align="center">
  <img src="docs/graph_core_logo_blue.png?raw=true" alt="Graph Core Logo" width="40%" style="display: block; margin: auto;">
</p>

## Introduction
`graph_core` is an open-source C++ library for sampling-based robot path planning. It provides essential tools for solving path planning problems, includes state-of-the-art algorithms, and streamlines the development of new algorithms.

## Status
[![build check](https://github.com/JRL-CARI-CNR-UNIBS/graph_core/actions/workflows/build_and_install.yaml/badge.svg)](https://github.com/JRL-CARI-CNR-UNIBS/graph_core/actions/workflows/build_and_install.yaml)
[![clang-format check](https://github.com/JRL-CARI-CNR-UNIBS/graph_core/actions/workflows/clang-format.yaml/badge.svg)](https://github.com/JRL-CARI-CNR-UNIBS/graph_core/actions/workflows/clang-format.yaml)
[![Codacy Badge](https://app.codacy.com/project/badge/Grade/1755d91be93e4c86912929a5e9ad04e8)](https://app.codacy.com/gh/JRL-CARI-CNR-UNIBS/graph_core/dashboard?utm_source=gh&utm_medium=referral&utm_content=&utm_campaign=Badge_grade)
![Status](https://img.shields.io/badge/License-BSD3-green)

Developed and tested for Ubuntu 20.04, 22.04 and Ubuntu-latest.


## Tutorials
See [this page](https://github.com/JRL-CARI-CNR-UNIBS/graph_core/blob/master/docs/tutorial/tutorial_intro.md) for tutorials.

## Dependencies
`graph_core` depends on [Eigen3](https://eigen.tuxfamily.org/index.php?title=Main_Page), which can be installed with

```bash
sudo apt update
sudo apt -y install libeigen3-dev
```

Furthermore, it relies on the following packages:

- [cnr_logger](https://github.com/CNR-STIIMA-IRAS/cnr_logger): Logging package.
- [cnr_param](https://github.com/CNR-STIIMA-IRAS/cnr_param): Package to read and set parameters. It depends on [cnr_yaml](https://github.com/CNR-STIIMA-IRAS/cnr_yaml).
- [cnr_class_loader](https://github.com/JRL-CARI-CNR-UNIBS/cnr_class_loader): Provides a way to load classes as plugins.

These packages require the following system dependencies. Install them by running

```bash
sudo apt update
sudo apt -y install libboost-all-dev libyaml-cpp-dev libpoco-dev liblog4cxx-dev libgtest-dev
```

To simplify installation and dependency resolution, `graph_core` uses [CPM](https://github.com/cpm-cmake/CPM.cmake) to automatically download and integrate the required GitHub packages (`cnr_logger`, `cnr_yaml`, `cnr_param`, `cnr_class_loader`) into your build process.
If you'd prefer to install the dependencies manually instead of relying on CPM, you can refer to the [cnr_common](https://github.com/JRL-CARI-CNR-UNIBS/cnr_common) page, or use [vcstool](https://github.com/dirk-thomas/vcstool) with the [`deps.repos` file](https://github.com/JRL-CARI-CNR-UNIBS/graph_core/deps.repos). In these cases, `graph_core` will automatically detect and link against the manually installed dependencies. Manual installation is recommended if other packages in your environment also depend on any of `graph_core`'s dependencies, ensuring consistency and avoiding redundant installations.

## Installation 
Follow these steps to compile and install `graph_core` using CMake.

1. Set the workspace directory path:
    ```bash
    export PATH_TO_WS=path_to_your_ws
    ```

2. Compile and install `graph_core`:
    ```bash
    cd $PATH_TO_WS
    mkdir -p build/graph_core
    cmake -S src/graph_core/graph_core -B build/graph_core -DCMAKE_INSTALL_PREFIX=$PATH_TO_WS/install
    make -C build/graph_core install
    ```

### Environment Configuration
Add the following lines to your `~.bashrc` file:

```bash
export PATH_TO_GRAPH_CORE_WS=path_to_your_ws #replace with the path to your workspace
if [[ ":$PATH:" != *":${PATH_TO_GRAPH_CORE_WS}/install/bin:"* ]]; then
    export PATH="${PATH_TO_GRAPH_CORE_WS}/install/bin:$PATH"
fi
if [[ ":$CMAKE_PREFIX_PATH:" != *":${PATH_TO_GRAPH_CORE_WS}/install:"* ]]; then
    export CMAKE_PREFIX_PATH="${PATH_TO_GRAPH_CORE_WS}/install:$CMAKE_PREFIX_PATH"
fi
``` 

## Installing within a Catkin workspace
To build `graph_core` within a Catkin workspace, ensure you have set `catkin config --install`. You do not need to export the paths as shown above, but you need to source the `install/setup.bash` file.

In your `~/.bashrc`, add `source path_to_your_catkin_ws/install/setup.bash`.

**Note**: If you installed `graph_core` dependencies automatically via CPM and another package in your workspace requires one of those dependencies (e.g., `cnr_param`) but not `graph_core`, you have two options:

- Option 1 [Recommended]: Build and install `graph_core` (and its dependencies) in a non-catkin workspace, then build other packages in a secondary (cascade) catkin workspace.
- Option 2: Download `graph_core` and its dependencies in a catkin workspace (e.g., using vcstool and the .repos file), build using `catkin build`, and source it.

## Final configuration
The `cnr_param` library requires a directory to store its parameters. You can set this directory by adding the following line to your `~/.bashrc` file:

```bash
export CNR_PARAM_ROOT_DIRECTORY="/tmp/cnr_param"
```

---

## Modifications from `master` (`apple_Test` branch)

This repository branch (`apple_Test`) introduces specific enhancements, bug fixes, and performance optimizations designed for high-performance multi-goal planning in robotics applications (specifically tested within the autonomous apple harvesting pipeline).

### Summary of Changes

| File | Change Type | Purpose / Description |
|---|---|---|
| `graph_core/CMakeLists.txt` | Build System / Compatibility | Upgraded C++ standard to C++20 and defined `EIGEN_MAX_ALIGN_BYTES=16` for ROS 2 Jazzy / Eigen alignment safety. |
| `graph_core/include/graph_core/metrics/euclidean_metrics.h` | Code Cleanliness / Optimization | Split configuration difference evaluation into explicit `Eigen::VectorXd diff` before `.norm()` calculation to ensure clean compiler vectorization and avoid template ambiguity. |
| `graph_core/src/graph_core/samplers/informed_sampler.cpp` | Logging / Performance | Downgraded bounded-sample warning from `CNR_WARN` to `CNR_DEBUG` to eliminate high-frequency terminal log spam and I/O latency during constrained informed sampling. |
| `graph_core/src/graph_core/graph/tree.cpp` | Performance Optimization | Optimized ellipsoid pruning checks (`purgeNodeOutsideEllipsoid` and `purgeNodeOutsideEllipsoids`) with short-circuit evaluation, and completely overhauled `cleanTree()` to run in $O(N)$ without repeated KD-Tree rebuilds. |
| `graph_core/tests/path.yaml`, `tree.yaml` | Testing / Datasets | Updated scenario trajectories and tree configuration datasets for apple harvesting verification. |

### Detailed Technical Breakdown

#### 1. C++20 Standard and Eigen Alignment Fix (`graph_core/CMakeLists.txt`)
- **C++ Standard**: Explicitly configured `set(CMAKE_CXX_STANDARD 20)` and `set(CMAKE_CXX_STANDARD_REQUIRED ON)` to match the modern ROS 2 Jazzy and C++20 toolchain.
- **Eigen Alignment**: Added compile definition `add_compile_definitions(EIGEN_MAX_ALIGN_BYTES=16)`. This resolves segmentation faults and undefined behaviors originating from differing memory alignment assumptions between Eigen fixed-size vector types and standard library allocators under GCC/Clang on modern x86_64 architectures.

#### 2. Euclidean Metrics Vectorization (`euclidean_metrics.h`)
- In `EuclideanMetrics::cost()` and `EuclideanMetrics::utopia()`, explicit temporaries are stored:
  ```cpp
  Eigen::VectorXd diff = configuration1 - configuration2;
  return diff.norm();
  ```
  This guarantees standard evaluation order and aids SIMD vectorization across diverse compiler optimization levels.

#### 3. Informed Sampler Log Flooding Prevention (`informed_sampler.cpp`)
- In `InformedSampler::sample()`, when a sample inside the informed ellipsoid falls outside configuration space limits after max attempts, the notice was reduced from `CNR_WARN` to `CNR_DEBUG`.
- In multi-goal planners (e.g. `mIRRT*`), rejecting out-of-joint-limit samples in tightly constrained spaces occurs hundreds of times per second. Suppressing the terminal output completely eliminates console I/O blocking.

#### 4. Tree Pruning & Reset Optimizations (`tree.cpp`)
When planning with large trees (thousands of nodes), tree pruning and resetting operations previously caused noticeable execution freezes. Two critical bottlenecks were resolved:
- **Short-circuiting in `purgeNodeOutsideEllipsoids`**:
  Previously, all samplers were evaluated regardless of whether the node was already confirmed to be inside an admissible set or present in the whitelist:
  ```cpp
  // Old implementation: evaluates all samplers unconditionally
  bool inbound = std::find(white_list.begin(), white_list.end(), node) != white_list.end();
  for (const SamplerPtr& sampler : samplers)
    inbound = inbound || sampler->inBounds(node->getConfiguration());
  ```
  Now, the check exits early as soon as whitelist membership or any sampler confirms admissibility:
  ```cpp
  // Optimized implementation: short-circuits immediately
  bool inbound = false;
  for (const NodePtr& wn : white_list) {
    if (wn == node) { inbound = true; break; }
  }
  if (!inbound) {
    for (const SamplerPtr& sampler : samplers) {
      if (sampler->inBounds(node->getConfiguration())) {
        inbound = true;
        break;
      }
    }
  }
  ```
- **$O(N)$ Tree Clearing in `cleanTree()`**:
  Previously, `Tree::cleanTree()` invoked `purgeFromHere(n, white_list, removed_nodes)` recursively across all children. For each node, it called `deleteNode()`, which repeatedly triggered full KD-Tree rebuilds whenever `deleted_nodes_ > deleted_nodes_threshold_`, leading to severe $O(N \log N \times \text{rebuilds})$ or $O(N^2)$ stalls.
  The new implementation disconnects all nodes in a single $O(N)$ pass and resets the KD-Tree in $O(1)$:
  ```cpp
  void Tree::cleanTree()
  {
    std::vector<NodePtr> white_list = { root_ };
    nodes_->disconnectNodes(white_list);
    root_->disconnectChildConnections();
    root_->disconnectNetChildConnections();
    nodes_->clear();
    nodes_->insert(root_);
  }
  ```
  This eliminates all solver stalls when purging discarded or completed trees.

