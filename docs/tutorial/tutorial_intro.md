
## Tutorials
This tutorial provides a concise guide on computing paths using `graph_core`. While `graph_core` alone does not handle scene management, it is often combined with ROS and MoveIt to enable collision checking and planning scene management. For practical examples of integrating `graph_core` with MoveIt for these purposes, you can refer to [these test examples](https://github.com/JRL-CARI-CNR-UNIBS/graph_ros_tests). Additionally, [this package](https://github.com/JRL-CARI-CNR-UNIBS/graph_display) offers tools to visualize paths, nodes, and trees within Rviz, enhancing your ability to work with and debug complex algorithms.

### Compute a path
[This tutorial](https://github.com/JRL-CARI-CNR-UNIBS/graph_core/tree/master/graph_core/tests/src/compute_path_test.cpp) provides a quick overview of how to use `graph_core` to compute a path. As mentioned earlier, `graph_core` does not handle collision checking, so this example uses foo collision checker. Below are explanations of the main steps involved.

1. Include the headers. The [structure](https://github.com/JRL-CARI-CNR-UNIBS/graph_core/tree/master/graph_core/include/graph_core) of `graph_core` is quite simple and intuitive. Here we need to include a path planning algoritm (e.g., RRT) and all the other things it needs: a sampler, a cost function and the foo collision checker.

```cpp
// Graph core required headers
#include <graph_core/solvers/rrt_star.h>
#include <graph_core/samplers/uniform_sampler.h>
#include <graph_core/metrics/euclidean_metrics.h>
#include <graph_core/collision_checkers/cube_3d_collision_checker.h>
```

2. Create a logger object for logging purposes.
   `graph_core` writes its messages through a `graph::core::LoggerPtr` (see [logger.h](https://github.com/JRL-CARI-CNR-UNIBS/graph_core/blob/master/graph_core/include/graph_core/logger.h)). Here we use `graph::core::ConsoleLogger`, which prints to the terminal the messages at or above a given level. In a ROS application you can pass instead an adapter to the ROS logger (see the [README](https://github.com/JRL-CARI-CNR-UNIBS/graph_core/blob/master/README.md#logging-and-parameters)). A null logger disables logging.

```cpp
std::string path_to_config_folder = std::string(TEST_DIR);

graph::core::LoggerPtr logger =
    std::make_shared<graph::core::ConsoleLogger>("compute_path_test", graph::core::LogLevel::Debug);
```
3. Define the foo collision checker, a cost function, a sampler, and finally the solver.
   `graph_core` does not read parameters by itself: the application fills a `graph::core::ParamMap` (a map from parameter name to value, see [params.h](https://github.com/JRL-CARI-CNR-UNIBS/graph_core/blob/master/graph_core/include/graph_core/params.h)) and passes it to the objects. Here the parameters are read from a YAML file with `graph::core::paramsFromYAML()`; in a ROS node you would copy them from the node parameters. `graph::core::get_param()` reads a single parameter from the map, with an optional default value.

```cpp
// Load the parameters defined under "graph_core_tests" in the YAML file
std::string param_ns_base = "graph_core_tests";
graph::core::ParamMap params = graph::core::paramsFromYAML(
    YAML::LoadFile(path_to_config_folder + "/graph_core_tests_params.yaml")[param_ns_base]);

// Define the collision checker (foo collision checker)
double min_cc_distance;  // distance between two configurations to be checked along a connection
double default_min_cc_distance = 0.01;
graph::core::get_param(logger, params, "min_cc_distance", min_cc_distance,
                        default_min_cc_distance);

double abs_joint_threshold;  // parameter specific for this foo collision checker: if a joint absolute value is less
                              // than this threshold, the configuration is considered not valid
double default_abs_joint_threshold = 1.0;
graph::core::get_param(logger, params, "abs_joint_threshold", abs_joint_threshold, default_abs_joint_threshold);

if (min_cc_distance <= 0)
  min_cc_distance = default_min_cc_distance;

if (abs_joint_threshold <= 0)
  abs_joint_threshold = default_abs_joint_threshold;

graph::core::CollisionCheckerPtr collision_checker =
    std::make_shared<graph::core::Cube3dCollisionChecker>(logger, abs_joint_threshold, min_cc_distance);

// Define a cost function (Euclidean metrics)
graph::core::MetricsPtr metrics = std::make_shared<graph::core::EuclideanMetrics>(logger);

// Define lower/upper bounds
size_t dof = 3;
Eigen::VectorXd lb(dof);
lb.setConstant(-2.5);
Eigen::VectorXd ub(dof);
ub.setConstant(2.5);

// Define a sampler (uniform sampler)
graph::core::SamplerPtr sampler = std::make_shared<graph::core::UniformSampler>(lb, ub, logger);

// Define the solver (RRT*)
graph::core::TreeSolverPtr solver =
    std::make_shared<graph::core::RRTStar>(metrics, collision_checker, sampler, logger);
```

4. Define the start and goal nodes for the path planning process. You can specify these configurations directly, or use the `graph::core::get_param(..)` function to read the start and goal configurations from the parameters. This approach allows you to manage configurations more flexibly, especially when working with complex or varying setups.

```cpp
// Define start and goal nodes
Eigen::VectorXd start_configuration(dof);
start_configuration << -1.5, -1.5, -1.5;

Eigen::VectorXd goal_configuration(dof);
goal_configuration << 1.5, 1.5, 1.5;

// Optionally, you can load these directly from params
// graph::core::get_param(logger, params, "start_configuration", start_configuration);
// graph::core::get_param(logger, params, "goal_configuration", goal_configuration);

graph::core::NodePtr start_node = std::make_shared<graph::core::Node>(start_configuration, logger);
graph::core::NodePtr goal_node = std::make_shared<graph::core::Node>(goal_configuration, logger);
```

5. With the solver set up, compute the path from the start to the goal node. `computePath()` configures the solver with `params` (e.g. `max_distance`, `use_kdtree`, `rewire_factor`) before solving.

```cpp
// Compute a path
double max_time;  // seconds
double default_max_time = 5.0;
graph::core::get_param(logger, params, "max_time", max_time, default_max_time);

size_t max_iter = 1000000;
graph::core::PathPtr solution;

graph::core::graph_time_point tic = graph::core::graph_time::now();
bool found = solver->computePath(start_node, goal_node, params, solution, max_time, max_iter);
graph::core::graph_time_point toc = graph::core::graph_time::now();
```

6. After computing the path, you can optionally print out the solution details and save them to a file for further analysis or record-keeping.

```cpp
// Print out the solution and save it to a file
double elapsed_time = graph::core::toSeconds(toc, tic);
GC_INFO(logger, "Elapsed time: " << elapsed_time << " seconds");

if (found)
  GC_INFO(logger, graph::core::color::BOLDGREEN() << "Path found!\n" << *solution << graph::core::color::RESET());
else
  GC_INFO(logger, graph::core::color::BOLDRED() << "Path not found!" << graph::core::color::RESET());

YAML::Node yaml_path, yaml_tree, yaml_path_ns, yaml_tree_ns;
yaml_path["path"] = solution->toYAML();
yaml_tree["tree"] = solution->getTree()->toYAML();

yaml_path_ns[param_ns_base] = yaml_path;
yaml_tree_ns[param_ns_base] = yaml_tree;

std::ofstream fout_path(path_to_config_folder + "/path.yaml");
std::ofstream fout_tree(path_to_config_folder + "/tree.yaml");

if (fout_path.is_open())
{
  fout_path << yaml_path_ns;
  fout_path.close();
}
else
  GC_ERROR(logger, "Error opening 'path.yaml' for writing.");

if (fout_tree.is_open())
{
  fout_tree << yaml_tree_ns;
  fout_tree.close();
}
else
  GC_ERROR(logger, "Error opening 'tree.yaml' for writing.");

```

### Path post processing
[This tutorial](https://github.com/JRL-CARI-CNR-UNIBS/graph_core/tree/master/graph_core/tests/src/path_post_processing_test.cpp) builds on the results of the previous tutorial and demonstrates how to apply post-processing to the computed path. Post-processing can refine the path, optimize it, or prepare it for further use.

1. Let's include a new header for path post processing.

```cpp
#include <graph_core/solvers/path_optimizers/path_local_optimizer.h>
```

2. Load and print out the path and tree computed in the previous tutorial. Paths and trees are data, not parameters: load them from the YAML files with `fromYAML()`.

```cpp
// Load the path and tree computed in the first tutorial
std::string param_ns = "graph_core_tests";
YAML::Node yaml_path = YAML::LoadFile(path_to_config_folder + "/path.yaml")[param_ns]["path"];
YAML::Node yaml_tree = YAML::LoadFile(path_to_config_folder + "/tree.yaml")[param_ns]["tree"];

graph::core::PathPtr path = graph::core::Path::fromYAML(yaml_path, metrics, collision_checker, logger);
graph::core::TreePtr tree = graph::core::Tree::fromYAML(yaml_tree, metrics, collision_checker, logger);

// Set the tree as the tree associated to the path
path->setTree(tree);

GC_INFO(logger, "Loaded path\n " << *path);
GC_INFO(logger, "Loaded tree\n " << *tree);
```

3. Clone the path. Since we will apply some post processing to the path but not to the tree, we will work on an exact clone of the original path to keep integrity between it and its tree. The clone has the same waypoints but stored within different nodes instances and does not have any tree associated. It shares the same metrics and collision checker with the original path. However, you can clone also them if you want to work on different instances. We will not do it here.

```cpp
graph::core::PathPtr cloned_path = path->clone();
```

E.g., flip the path:

```cpp
cloned_path->flip();
```

4. [This class](https://github.com/JRL-CARI-CNR-UNIBS/graph_core/blob/master/graph_core/include/graph_core/solvers/path_optimizers/path_optimizer_base.h) is designed for local optimizations.

```cpp
// Apply some post processing on the path (warp and simplify)
graph::core::PathLocalOptimizerPtr path_opt =
    std::make_shared<graph::core::PathLocalOptimizer>(collision_checker, metrics, logger);

path_opt->setPath(cloned_path);

double warp_min_conn_length;
double default_warp_min_conn_length = 0.1;
graph::core::get_param(logger, params, "warp_min_conn_length", warp_min_conn_length, default_warp_min_conn_length);

double warp_min_step_size;
double default_warp_min_step_size = 0.05;
graph::core::get_param(logger, params, "warp_min_step_size", warp_min_step_size, default_warp_min_step_size);

path_opt->warp(warp_min_conn_length, warp_min_step_size);
GC_INFO(logger, "Path after warp \n " << *cloned_path);

double simplify_max_conn_length;
double default_simplify_max_conn_length = 0.05;
graph::core::get_param(logger, params, "simplify_max_conn_length", simplify_max_conn_length,
                        default_simplify_max_conn_length);

path_opt->simplify(simplify_max_conn_length);
GC_INFO(logger, "Path after simplify \n " << *cloned_path);
```
