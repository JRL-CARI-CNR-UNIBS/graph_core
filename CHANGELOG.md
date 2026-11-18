# Changelog
All notable changes to `graph_core` are documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/).

## [Unreleased]

### Removed
- Dependencies on `cnr_logger`, `cnr_param` and `cnr_yaml`: `graph_core` is now a plain C++ library that does not depend on ROS and does not read parameters from any server. Only `cnr_class_loader` is still fetched (by CPM or with `deps.repos`).
- `get_param` overloads for `Node`, `Tree` and `Path`: they are data, not parameters, and are loaded with `fromYAML()`.
- `-Ofast` from the Release flags: it enables `-ffinite-math-only`, which breaks the comparisons with `std::numeric_limits<double>::infinity()` used for the costs.

### Added
- Logger interface `graph::core::Logger` / `graph::core::LoggerPtr` (`graph_core/logger.h`) replacing `cnr_logger::TraceLoggerPtr`, with:
  - `GC_TRACE`, `GC_DEBUG`, `GC_INFO`, `GC_WARN`, `GC_ERROR`, `GC_FATAL` macros (same `(logger, stream)` form as `CNR_*`); the message is built only if the level is enabled, a null logger disables logging, `GC_FATAL` only logs;
  - throttled variants `GC_*_THROTTLE(logger, period, stream)`, writing at most once every `period` seconds per call site;
  - `graph::core::ConsoleLogger`, writing to the terminal with a level threshold;
  - ANSI colors in `graph::core::color`.
- Parameters (`graph_core/params.h`): `graph::core::ParamMap` (name -> `bool`, `int64_t`, `double`, `std::string` or arrays of them, as in ROS 2), `get_param(logger, params, name, value[, default])` with integer -> floating point, range-checked integer and array -> `Eigen` conversions, `paramsFromYAML()` and `subParams()`.
- Temporary compatibility header `graph_core/compat/cnr_logger.h`, mapping the `CNR_*` macros and the `cnr_logger` names to the new logger for downstream packages.
- Unit tests for the logger, `ParamMap` / `get_param` / `paramsFromYAML`, and `Tree::near` against a brute-force search.
- Documentation of the use in ROS 2: colcon build, `package.xml` / `CMakeLists.txt`, adapters to the `rclcpp` logger and to the node parameters, example node and parameter file (README and tutorial).
- This changelog.

### Changed
- `config()`, plugin `init()` and `computePath()` take a `graph::core::ParamMap` instead of a parameter namespace.
- printf-style `CNR_*` calls converted to streams.
- `yaml-cpp` is now an explicit dependency.
- Tests read the parameters from a YAML file instead of `cnr_param_server`, take the log level as argument and write their outputs to the temporary directory; they fail when no path is found.
- CI no longer builds `cnr_logger`, `cnr_yaml`, `cnr_param` nor installs `liblog4cxx-dev`.

### Fixed
- `package.xml` had an invalid XML declaration (`version="2.0"`), which made colcon ignore its dependencies.
- `paramsFromYAML()`: quoted scalars stay strings, integers follow YAML 1.2 (`010` is decimal), arrays of booleans are supported.
- Logger: lock-free `enabled()`; the throttled macros check the level first, saturate the period and let only one thread write per period.
- `graph_coreConfig.cmake`: `set_and_check` was called with several paths.

### Performance
- `Tree::toYAML()` is O(n) instead of O(n^2).
- `costToNode()` locks the parent connection once per step.
- `KdNode::near()` compares squared distances and computes the square root only for the nodes inside the ball.
