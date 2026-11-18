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

## Installing within a Catkin workspace (ROS 1)
To build `graph_core` within a Catkin workspace, ensure you have set `catkin config --install`. You do not need to export the paths as shown above, but you need to source the `install/setup.bash` file.

In your `~/.bashrc`, add `source path_to_your_catkin_ws/install/setup.bash`.

**Note**: If you installed `graph_core` dependencies automatically via CPM and another package in your workspace requires one of those dependencies (e.g., `cnr_class_loader`) but not `graph_core`, you have two options:

- Option 1 [Recommended]: Build and install `graph_core` (and its dependencies) in a non-catkin workspace, then build other packages in a secondary (cascade) catkin workspace.
- Option 2: Download `graph_core` and its dependencies in a catkin workspace (e.g., using vcstool and the .repos file), build using `catkin build`, and source it.

## Installing within a colcon workspace (ROS 2)
`graph_core` is a plain CMake package (`build_type` `cmake` in `package.xml`), so colcon builds it like any other package of the workspace. It has been tested with ROS 2 Jazzy.

```bash
cd path_to_your_ros2_ws/src
git clone https://github.com/JRL-CARI-CNR-UNIBS/graph_core.git
vcs import < graph_core/deps.repos     # cnr_class_loader as a workspace package
cd ..
rosdep install --from-paths src --ignore-src -y
colcon build --packages-up-to graph_core --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

No `PATH` / `CMAKE_PREFIX_PATH` export is needed: `install/setup.bash` does it. If `cnr_class_loader` is in the workspace (or already installed and sourced), `graph_core` uses it; otherwise CPM downloads it during the build. In this case skip the `vcs import` step and add `--skip-keys cnr_class_loader` to `rosdep install`, since `cnr_class_loader` is not a rosdep key. The test executables are installed in `install/graph_core/bin/graph_core/tests/src` (e.g. `run_all_tests`); pass `-DBUILD_TESTS=OFF` to skip them.

The same note of the Catkin section applies: if other packages of the workspace depend on `cnr_class_loader`, put it in the workspace with `deps.repos` instead of relying on CPM.

## Logging and parameters
`graph_core` is a plain C++ library: it does not depend on ROS and does not read parameters from any server. The application that uses it decides where messages go and where parameters come from.

**Logging.** Classes receive a `graph::core::LoggerPtr` (see [`logger.h`](graph_core/include/graph_core/logger.h)) and write through the `GC_TRACE`, `GC_DEBUG`, `GC_INFO`, `GC_WARN`, `GC_ERROR` and `GC_FATAL` macros. `GC_FATAL` only logs, it does not abort. A null logger is allowed and disables logging. `graph::core::ConsoleLogger` writes to the terminal:

```cpp
#include <graph_core/logger.h>

graph::core::LoggerPtr logger = std::make_shared<graph::core::ConsoleLogger>("my_planner", graph::core::LogLevel::Info);
GC_INFO(logger, "max distance = " << max_distance);
```

To use another logging framework, implement `graph::core::Logger` (see [Using graph_core in a ROS 2 package](#using-graph_core-in-a-ros-2-package) for an adapter to the ROS 2 logger). Each `GC_*` macro has a `GC_*_THROTTLE(logger, period, stream)` variant that writes at most once every `period` seconds per call site.

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

In a ROS 2 node, build the map from the node parameters (see [Using graph_core in a ROS 2 package](#using-graph_core-in-a-ros-2-package)).

Paths and trees are data, not parameters: save them with `toYAML()` and load them with `Path::fromYAML()` / `Tree::fromYAML()`.

**Migrating from cnr_logger.** [`graph_core/compat/cnr_logger.h`](graph_core/include/graph_core/compat/cnr_logger.h) maps the `CNR_*` macros, `cnr_logger::TraceLoggerPtr` and the cnr_logger colors to the `graph_core` logger. It is temporary and must not be included together with the real cnr_logger headers.

## Using graph_core in a ROS 2 package
`graph_core` does not depend on ROS: a ROS 2 package links it as a normal CMake library and connects it to the node through two small adapters, one for the logger and one for the parameters.

### package.xml and CMakeLists.txt

```xml
<depend>graph_core</depend>
<depend>rclcpp</depend>
```

```cmake
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(graph_core REQUIRED)

add_executable(my_planner_node src/my_planner_node.cpp)
target_link_libraries(my_planner_node PUBLIC graph_core::graph_core rclcpp::rclcpp)
install(TARGETS my_planner_node DESTINATION lib/${PROJECT_NAME})

ament_package()
```

A library that exposes `graph_core` types in its headers should also call `ament_export_dependencies(graph_core)`.

### Logger adapter
Forward the `graph_core` messages to the node logger, so that they are printed, sent to `/rosout` and filtered with `--ros-args --log-level` like the other ROS 2 messages:

```cpp
#include <graph_core/logger.h>
#include <rclcpp/rclcpp.hpp>

class RclcppLogger : public graph::core::Logger
{
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

private:
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

  rclcpp::Logger logger_;
};
```

`graph::core::LogLevel::Trace` has no ROS 2 counterpart and is mapped to `DEBUG`. Using a child logger (`node->get_logger().get_child("graph_core")`) allows to change the level of the planner messages alone, e.g. `--ros-args --log-level my_planner_node.graph_core:=debug`.

### Parameters from the node
`ParamValue` has the same types as `rclcpp::Parameter` (except byte arrays), so the conversion is one-to-one. The function below returns the parameters under `prefix` with the prefix removed from the names (`my_solver.max_distance` -> `max_distance`). Parameters passed with `--params-file` are declared on the fly, so the node does not need to know their names in advance; alternatively, create the node with `rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)`.

```cpp
#include <graph_core/params.h>
#include <rclcpp/rclcpp.hpp>

graph::core::ParamMap paramsFromNode(rclcpp::Node& node, const std::string& prefix)
{
  const std::string full_prefix = prefix.empty() ? "" : prefix + ".";

  // Declare the parameters given to the node under the prefix
  for (const auto& [name, value] : node.get_node_parameters_interface()->get_parameter_overrides())
    if (name.compare(0, full_prefix.size(), full_prefix) == 0 && !node.has_parameter(name))
      node.declare_parameter(name, value);

  graph::core::ParamMap params;
  const auto list = node.list_parameters(prefix.empty() ? std::vector<std::string>{} : std::vector<std::string>{ prefix },
                                          rcl_interfaces::srv::ListParameters::Request::DEPTH_RECURSIVE);
  for (const std::string& name : list.names)
  {
    if (name.compare(0, full_prefix.size(), full_prefix) != 0)
      continue;
    const std::string key = name.substr(full_prefix.size());
    const rclcpp::Parameter p = node.get_parameter(name);
    switch (p.get_type())
    {
      case rclcpp::ParameterType::PARAMETER_BOOL:
        params[key] = p.as_bool();
        break;
      case rclcpp::ParameterType::PARAMETER_INTEGER:
        params[key] = static_cast<int64_t>(p.as_int());
        break;
      case rclcpp::ParameterType::PARAMETER_DOUBLE:
        params[key] = p.as_double();
        break;
      case rclcpp::ParameterType::PARAMETER_STRING:
        params[key] = p.as_string();
        break;
      case rclcpp::ParameterType::PARAMETER_BOOL_ARRAY:
        params[key] = p.as_bool_array();
        break;
      case rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY:
      {
        const auto& v = p.as_integer_array();
        params[key] = std::vector<int64_t>(v.begin(), v.end());
        break;
      }
      case rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY:
        params[key] = p.as_double_array();
        break;
      case rclcpp::ParameterType::PARAMETER_STRING_ARRAY:
        params[key] = p.as_string_array();
        break;
      default:  // not set or byte array
        break;
    }
  }
  return params;
}
```

Nested ROS 2 names keep the `.` separator (`a.b.c` -> `b.c` with prefix `a`), while `paramsFromYAML()` joins nested keys with `/` by default: use `graph::core::subParams(params, "b", ".")` to extract a deeper level, or `paramsFromYAML(yaml, ".")` to get the same keys from a YAML file. The `graph_core` solvers and plugins only read first-level names (`max_distance`, `use_kdtree`, ...).

Call `paramsFromNode()` before each `computePath()` / `config()` to pick up the values changed with `ros2 param set`.

### Example node

```cpp
#include <graph_core/solvers/rrt_star.h>
#include <graph_core/samplers/uniform_sampler.h>
#include <graph_core/metrics/euclidean_metrics.h>
#include <graph_core/collision_checkers/cube_3d_collision_checker.h>
#include <rclcpp/rclcpp.hpp>
// + RclcppLogger and paramsFromNode() defined above

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("my_planner_node");

  graph::core::LoggerPtr logger = std::make_shared<RclcppLogger>(node->get_logger().get_child("graph_core"));
  graph::core::ParamMap params = paramsFromNode(*node, "my_solver");

  double min_cc_distance;
  graph::core::get_param(logger, params, "min_cc_distance", min_cc_distance, 0.01);

  auto collision_checker = std::make_shared<graph::core::Cube3dCollisionChecker>(logger, 1.0, min_cc_distance);
  auto metrics = std::make_shared<graph::core::EuclideanMetrics>(logger);

  Eigen::VectorXd lb = Eigen::VectorXd::Constant(3, -2.5);
  Eigen::VectorXd ub = Eigen::VectorXd::Constant(3, 2.5);
  auto sampler = std::make_shared<graph::core::UniformSampler>(lb, ub, logger);
  auto solver = std::make_shared<graph::core::RRTStar>(metrics, collision_checker, sampler, logger);

  Eigen::VectorXd start, goal;
  graph::core::get_param(logger, params, "start_configuration", start);
  graph::core::get_param(logger, params, "goal_configuration", goal);
  auto start_node = std::make_shared<graph::core::Node>(start, logger);
  auto goal_node = std::make_shared<graph::core::Node>(goal, logger);

  double max_time;
  graph::core::get_param(logger, params, "max_time", max_time, 5.0);

  graph::core::PathPtr solution;
  if (solver->computePath(start_node, goal_node, params, solution, max_time, 1000000))
    GC_INFO(logger, "Path found, cost " << solution->cost());
  else
    GC_WARN(logger, "Path not found");

  rclcpp::shutdown();
  return 0;
}
```

with the parameter file `config/my_planner.yaml`

```yaml
my_planner_node:
  ros__parameters:
    my_solver:
      max_distance: 0.5
      use_kdtree: true
      rewire_factor: 1.1
      min_cc_distance: 0.01
      max_time: 5.0
      start_configuration: [-1.5, -1.5, -1.5]
      goal_configuration: [1.5, 1.5, 1.5]
```

```bash
ros2 run my_package my_planner_node --ros-args --params-file config/my_planner.yaml
```

In ROS 2 YAML files an array of numbers that looks like integers (`[1, 2, 3]`) is an integer array: `get_param` accepts it for an `Eigen::VectorXd` or a `double` vector anyway. In a launch file, pass the same file with `parameters=[config_file]`.

For collision checking against a MoveIt planning scene see [moveit_collision_checker](https://github.com/JRL-CARI-CNR-UNIBS/moveit_collision_checker), and for visualizing paths and trees in RViz see [graph_display](https://github.com/JRL-CARI-CNR-UNIBS/graph_display).

## Changelog
See [CHANGELOG.md](CHANGELOG.md).
