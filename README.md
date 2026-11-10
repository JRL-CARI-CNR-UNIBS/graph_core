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

It also depends on [yaml-cpp](https://github.com/jbeder/yaml-cpp) (to save and load paths and trees, and to read parameters from YAML files) and on the following package:

- [cnr_class_loader](https://github.com/JRL-CARI-CNR-UNIBS/cnr_class_loader): Provides a way to load classes as plugins.

Install the system dependencies by running

```bash
sudo apt update
sudo apt -y install libboost-all-dev libyaml-cpp-dev libpoco-dev libgtest-dev
```

To simplify installation and dependency resolution, `graph_core` uses [CPM](https://github.com/cpm-cmake/CPM.cmake) to automatically download and integrate `cnr_class_loader` into your build process.
If you'd prefer to install it manually instead of relying on CPM, you can refer to the [cnr_common](https://github.com/JRL-CARI-CNR-UNIBS/cnr_common) page, or use [vcstool](https://github.com/dirk-thomas/vcstool) with the [`deps.repos` file](https://github.com/JRL-CARI-CNR-UNIBS/graph_core/deps.repos). In these cases, `graph_core` will automatically detect and link against the manually installed dependency. Manual installation is recommended if other packages in your environment also depend on `cnr_class_loader`, ensuring consistency and avoiding redundant installations.

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

**Note**: If you installed `graph_core` dependencies automatically via CPM and another package in your workspace requires one of those dependencies (e.g., `cnr_class_loader`) but not `graph_core`, you have two options:

- Option 1 [Recommended]: Build and install `graph_core` (and its dependencies) in a non-catkin workspace, then build other packages in a secondary (cascade) catkin workspace.
- Option 2: Download `graph_core` and its dependencies in a catkin workspace (e.g., using vcstool and the .repos file), build using `catkin build`, and source it.

## Logging and parameters
`graph_core` is a plain C++ library: it does not depend on ROS and does not read parameters from any server. The application that uses it decides where messages go and where parameters come from.

**Logging.** Classes receive a `graph::core::LoggerPtr` (see [`logger.h`](graph_core/include/graph_core/logger.h)) and write through the `GC_TRACE`, `GC_DEBUG`, `GC_INFO`, `GC_WARN`, `GC_ERROR` and `GC_FATAL` macros. `GC_FATAL` only logs, it does not abort. A null logger is allowed and disables logging. `graph::core::ConsoleLogger` writes to the terminal:

```cpp
#include <graph_core/logger.h>

graph::core::LoggerPtr logger = std::make_shared<graph::core::ConsoleLogger>("my_planner", graph::core::LogLevel::Info);
GC_INFO(logger, "max distance = " << max_distance);
```

To use another logging framework, implement `graph::core::Logger`. For example, in a ROS 2 package:

```cpp
#include <rclcpp/rclcpp.hpp>

class RclcppLogger : public graph::core::Logger
{
  rclcpp::Logger logger_;

  static int severity(graph::core::LogLevel level)
  {
    switch (level)
    {
      case graph::core::LogLevel::Trace:
      case graph::core::LogLevel::Debug:
        return RCUTILS_LOG_SEVERITY_DEBUG;
      case graph::core::LogLevel::Info:
        return RCUTILS_LOG_SEVERITY_INFO;
      case graph::core::LogLevel::Warn:
        return RCUTILS_LOG_SEVERITY_WARN;
      case graph::core::LogLevel::Error:
        return RCUTILS_LOG_SEVERITY_ERROR;
      default:
        return RCUTILS_LOG_SEVERITY_FATAL;
    }
  }

public:
  explicit RclcppLogger(const rclcpp::Logger& logger) : logger_(logger)
  {
  }

  bool enabled(graph::core::LogLevel level) const override
  {
    return rcutils_logging_logger_is_enabled_for(logger_.get_name(), severity(level));
  }

  void log(graph::core::LogLevel level, const char* func, int line, const std::string& msg) override
  {
    rcutils_log_location_t location = { func, "graph_core", static_cast<size_t>(line) };
    rcutils_log(&location, severity(level), logger_.get_name(), "%s", msg.c_str());
  }
};

// graph::core::LoggerPtr logger = std::make_shared<RclcppLogger>(node->get_logger());
```

**Parameters.** Solvers, optimizers and plugins are configured with a `graph::core::ParamMap` (see [`params.h`](graph_core/include/graph_core/params.h)): a map from parameter name to value (`bool`, `int64_t`, `double`, `std::string` or arrays of them, as in ROS 2). Missing parameters take their default value.

```cpp
#include <graph_core/params.h>

// From code
graph::core::ParamMap params{ { "max_distance", 0.5 }, { "use_kdtree", true }, { "rewire_factor", 1.1 } };

// From a YAML file (nested maps are flattened as "a/b")
graph::core::ParamMap params = graph::core::paramsFromYAML(YAML::LoadFile("params.yaml")["my_solver"]);

solver->config(params);
```

`graph::core::get_param(logger, params, name, value[, default_value])` reads a single parameter. It accepts an integer for a floating point parameter, checks the range of integers (e.g. a negative value for an unsigned parameter is an error) and accepts an array of numbers for an `Eigen::VectorXd`. A parameter with a wrong type throws `std::invalid_argument`.

In a ROS 2 node, copy the parameters under a prefix into the map, e.g. with `node->list_parameters({prefix}, 0)` followed by `node->get_parameters(...)`, converting each `rclcpp::Parameter` into the matching `ParamValue`.

Paths and trees are data, not parameters: save them with `toYAML()` and load them with `Path::fromYAML()` / `Tree::fromYAML()`.

**Migrating from cnr_logger.** [`graph_core/compat/cnr_logger.h`](graph_core/include/graph_core/compat/cnr_logger.h) maps the `CNR_*` macros, `cnr_logger::TraceLoggerPtr` and the cnr_logger colors to the `graph_core` logger. It is temporary and must not be included together with the real cnr_logger headers.
