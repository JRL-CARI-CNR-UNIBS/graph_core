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

These packages require the following system dependencies. Install them by running

```bash
sudo apt update
sudo apt -y install libboost-all-dev libyaml-cpp-dev libpoco-dev liblog4cxx-dev libgtest-dev
```

To simplify installation and dependency resolution, `graph_core` uses [CPM](https://github.com/cpm-cmake/CPM.cmake) to automatically download and integrate the required GitHub packages (`cnr_logger`, `cnr_yaml`, `cnr_param`, `cnr_class_loader`) into your build process.
If you'd prefer to install the dependencies manually instead of relying on CPM, you can refer to the [cnr_common](https://github.com/JRL-CARI-CNR-UNIBS/cnr_common) page. In that case, `graph_core` will automatically detect and use the manually installed versions of the dependencies.

## Installation 
Follow these steps to compile and install `graph_core` using CMake.

Note: If you want to automatically install dependencies using CPM and have ROS1 installed on your system, it is recommended not to source ROS1 before building `graph_core`. This avoids potential conflicts between `graph_core` dependencies installed via CPM and catkin.

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

These settings are necessary to make the installed libraries visible. 

## Installing within a Catkin workspace
`graph_core` can also be built within a Catkin workspace. Ensure you have set `catkin config --install`. In this case, you do not need to export the paths as shown above, but you need to source the `install/setup.bash` file.

In your `~/.bashrc`, add `source path_to_your_catkin_ws/install/setup.bash`.

**Note**: If you installed `graph_core` dependencies automatically via CPM and another package in your workspace requires one of those dependencies (e.g., `cnr_param`) but not `graph_core`, you have two options:

- Option 1 [Recommended]: Build and install `graph_core` (and its dependencies) in one workspace, source it, and then build other packages in a secondary (cascade) workspace.
- Option 2: Build everything in the same workspace. Be aware that you may need to build twice. Some packages might fail in the first build but succeed in the second, once dependencies are resolved.

## Final configuration
The `cnr_param` library requires a directory to store its parameters. You can set this directory by adding the following line to your `~/.bashrc` file:

```bash
export CNR_PARAM_ROOT_DIRECTORY="/tmp/cnr_param"
```

<!-- However, `graph_core` can also be compiled in both ROS1 and ROS2 workspaces. For a ROS1 workspace, ensure you have set `catkin config --install`. In this case, you do not need to export the paths as shown above. -->