<p align="center">
  <picture>
    <source media="(prefers-color-scheme: dark)" srcset="docs/graph_core_logo.png">
    <source media="(prefers-color-scheme: light)" srcset="docs/graph_core_logo_background.png">
    <img alt="Graph Core Logo." src="docs/graph_core_logo_background.png" width="40%" style="display: block; margin: auto;">
  </picture>
</p>


## Introduction
`graph_core` is an open-source C++ library for sampling-based robot path planning. It provides essential tools for solving path planning problems, includes state-of-the-art algorithms, and streamlines the development of new algorithms.

## Status

[![GitHub Action
Status](https://github.com/JRL-CARI-CNR-UNIBS/graph_core/workflows/master/badge.svg)](https://github.com/JRL-CARI-CNR-UNIBS/graph_core)
[![Codacy Badge](https://app.codacy.com/project/badge/Grade/1755d91be93e4c86912929a5e9ad04e8)](https://app.codacy.com/gh/JRL-CARI-CNR-UNIBS/graph_core/dashboard?utm_source=gh&utm_medium=referral&utm_content=&utm_campaign=Badge_grade)

![Status](https://img.shields.io/badge/License-BSD3-green)
![Status](https://img.shields.io/badge/Documentation-Updating-blue?&logo=github)

🚧 Update in Progress! 🚧  
We're currently working on documentation. Expect new changes in the next weeks. Stay tuned!
<!-- <h3 align="center">🚧 Update in Progress! 🚧</h3>
<p align="center">
  <img src="https://img.shields.io/badge/Status-Updating-blue?style=flat-square&logo=github">
</p>
<p align="center" style="font-size: 14px; color: gray;">
  We're currently working on documentation. Expect new changes in the next weeks. Stay tuned!
</p> -->

### Tutorials
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

For detailed instructions on downloading and installing these dependencies, refer to the [cnr_common](https://github.com/JRL-CARI-CNR-UNIBS/cnr_common) page.

## Installation 
Follow these steps to compile and install `graph_core`using CMake: 

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

## Environment Configuration
According to the [cnr_common](https://github.com/JRL-CARI-CNR-UNIBS/cnr_common) instructions, add the following lines to your `~.bashrc` file:

```bash
if [[ ":$PATH:" != *":path_to_your_ws/install/bin:"* ]]; then
    export PATH="path_to_your_ws/install/bin:$PATH"
fi
if [[ ":$LD_LIBRARY_PATH:" != *":path_to_your_ws/install/lib:"* ]]; then
    export LD_LIBRARY_PATH="path_to_your_ws/install/lib:$LD_LIBRARY_PATH"
fi
if [[ ":$CMAKE_PREFIX_PATH:" != *":path_to_your_ws/install:"* ]]; then
    export CMAKE_PREFIX_PATH="path_to_your_ws/install:$CMAKE_PREFIX_PATH"
fi
``` 
Replace `path_to_your_ws/install` with the actual path to your install folder. These settings are necessary to make the installed libraries visible. However, `graph_core` can also be compiled in both ROS1 and ROS2 workspaces. For a ROS1 workspace, ensure you have set `catkin config --install`. In this case, you do not need to export the paths as shown above.