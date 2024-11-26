
## Tutorials
This tutorial provides a concise guide on computing paths using `graph_core`. While `graph_core` alone does not handle scene management, it is often combined with ROS and MoveIt to enable collision checking and planning scene management. For practical examples of integrating `graph_core` with MoveIt for these purposes, you can refer to [these test examples](https://github.com/JRL-CARI-CNR-UNIBS/graph_ros_tests). Additionally, [this package](https://github.com/JRL-CARI-CNR-UNIBS/graph_display) offers tools to visualize paths, nodes, and trees within Rviz, enhancing your ability to work with and debug complex algorithms.

### Compute a path
[This tutorial](https://github.com/JRL-CARI-CNR-UNIBS/graph_core/blob/master/docs/tutorial/tutorial1.cpp) provides a quick overview of how to use `graph_core` to compute a path. As mentioned earlier, `graph_core` does not handle collision checking, so this example uses foo collision checker. Below are explanations of the main steps involved.

1. Include the headers. The [structure](https://github.com/JRL-CARI-CNR-UNIBS/graph_core/tree/master/graph_core/include/graph_core) of `graph_core` is quite simple and intuitive. Here we need to include a path planning algoritm (e.g., RRT) and all the other things it needs: a sampler, a cost function and the foo collision checker.

```cpp
// Graph core required headers
#include <graph_core/solvers/rrt.h>
#include <graph_core/samplers/uniform_sampler.h>
#include <graph_core/metrics/euclidean_metrics.h>
#include <graph_core/collision_checkers/cube_3d_collision_checker.h>
```

2. Create a logger object for logging purposes. 
   For logging purposes, create a logger object using the [cnr_logger](https://github.com/CNR-STIIMA-IRAS/cnr_logger) library. This logger requires a configuration file that specifies where to print the logging information, along with other settings. For further details, refer to the [cnr_logger repository](https://github.com/CNR-STIIMA-IRAS/cnr_logger).

```cpp
// Load the logger's cofiguration
std::string path_to_logger_folder = "path/to/logger/folder";
std::string logger_file = path_to_logger_folder+"/logger_param.yaml";
cnr_logger::TraceLoggerPtr logger = std::make_shared<cnr_logger::TraceLogger>("graph_core_tutorial_loggers",logger_file);
```

3. Define the foo collision checker, a cost function, a sampler, and finally the solver.
   To handle parameters, you can utilize [cnr_param](https://github.com/CNR-STIIMA-IRAS/cnr_param), which leverages [yaml-cpp](https://github.com/jbeder/yaml-cpp) to read parameter values from YAML files. `cnr_param` also integrates with the ROS and ROS2 parameter ecosystems, making it versatile for different applications. See the `cnr_param` repository for further details. Additionally, `graph_core` provides a wrapper function to simplify parameter retrieval, helping to streamline the setup process. You can find the main utility functions offered by `graph_core` [here](https://github.com/JRL-CARI-CNR-UNIBS/graph_core/blob/master/graph_core/include/graph_core/util.h).

```cpp
// Define namespace for parameters retrieving
std::string param_ns = "/graph_core_tutorial";  // must begin with "/"

// Define the collision checker (foo collision checker)
double min_cc_distance;
double default_min_cc_distance = 0.01;
graph::core::get_param(logger,param_ns,"min_cc_distance",min_cc_distance,min_cc_distance); //wrapper to cnr_param functions
  
if(min_cc_distance<=0)
  min_cc_distance = default_min_cc_distance;
 
  graph::collision_check::CollisionCheckerPtr collision_checker = 
  std::make_shared<graph::collision_check::Cube3dCollisionChecker>(logger, min_cc_distance);

// Define a cost function (Euclidean metrics)
graph::core::MetricsPtr metrics = std::make_shared<graph::core::EuclideanMetrics>(logger);

// Define lower/upper bounds
size_t dof = 3; 
Eigen::VectorXd lb(dof); lb.setConstant(-2.5);
Eigen::VectorXd ub(dof); ub.setConstant( 2.5);

// Define a sampler (uniform sampler)
graph::core::SamplerPtr sampler = std::make_shared<graph::core::UniformSampler>(lb,ub,logger);

// Define the solver (RRT)
graph::core::TreeSolverPtr solver =
std::make_shared<graph::core::RRT>(metrics,collision_checker,sampler,logger);
```

4. Define the start and goal nodes for the path planning process. You can specify these configurations directly, or use the `graph::core::get_param(..)` function to read the start and goal configurations from parameter files. This approach allows you to manage configurations more flexibly, especially when working with complex or varying setups.

```cpp
// Define start and goal nodes
Eigen::VectorXd start_configuration(dof);
start_configuration << -1.5,-1.5,-1.5;

Eigen::VectorXd goal_configuration(dof);
goal_configuration <<  1.5, 1.5, 1.5;

// Optionally, you can load these directly from params
// graph::core::get_param(logger,param_ns,"start_configuration",start_configuration);
// graph::core::get_param(logger,param_ns,"goal_configuration",goal_configuration)

graph::core::NodePtr start_node = std::make_shared<graph::core::Node>(start_configuration,logger);
graph::core::NodePtr goal_node  = std::make_shared<graph::core::Node>(goal_configuration, logger);
```

5. With the solver set up, compute the path from the start to the goal node.

```cpp
// Compute a path
double max_time = 10.0; //seconds
size_t max_iter = 1000000;
graph::core::PathPtr solution;
  
graph::core::graph_time_point tic = graph::core::graph_time::now();
bool found = solver->computePath(start_node,goal_node,param_ns,solution,max_time,max_iter);
graph::core::graph_time_point toc = graph::core::graph_time::now(); 
```

6. After computing the path, you can optionally print out the solution details and save them to a file for further analysis or record-keeping.

```cpp
// Print out the solution and save it to a file
double elapsed_time = graph::core::toSeconds(toc,tic);
CNR_INFO(logger, "Elapsed time: "<<elapsed_time<<" seconds");

if(found)
    CNR_INFO(logger, cnr_logger::BOLDGREEN() << "Path found!\n"<<*solution<<cnr_logger::RESET());
else
    CNR_INFO(logger, cnr_logger::BOLDRED() << "Path not found!"<<cnr_logger::RESET());

YAML::Node yaml_path, yaml_tree, yaml_path_ns, yaml_tree_ns;
yaml_path["path"] = solution->toYAML();
yaml_tree["tree"] = solution->getTree()->toYAML();

yaml_path_ns[param_ns] = yaml_path;
yaml_tree_ns[param_ns] = yaml_tree;

std::ofstream fout_path(path_to_config_folder+"/path.yaml");
std::ofstream fout_tree(path_to_config_folder+"/tree.yaml");

if (fout_path.is_open())
{
  fout_path << yaml_path;
  fout_path.close();
}
else
  CNR_ERROR(logger,"Error opening 'path.yaml' for writing.");

if (fout_tree.is_open())
{
  fout_tree << yaml_tree;
  fout_tree.close();
}
else
  CNR_ERROR(logger,"Error opening 'tree.yaml' for writing.");
```

### Path post processing
[This tutorial](https://github.com/JRL-CARI-CNR-UNIBS/graph_core/blob/master/docs/tutorial/tutorial2.cpp) builds on the results of the previous tutorial and demonstrates how to apply post-processing to the computed path. Post-processing can refine the path, optimize it, or prepare it for further use.

1. Let's include a new header for path post processing.

```cpp
#include <graph_core/solvers/path_optimizers/path_local_optimizer.h>
```

2. Load and print out the path and tree computed in the previous tutorial.

```cpp
// Load the path and tree computed in the first tutorial
graph::core::PathPtr path;
graph::core::get_param(logger,param_ns,"path",path,metrics,checker);

graph::core::TreePtr tree;
graph::core::get_param(logger,param_ns,"tree",tree,metrics,checker);

// Set the tree as the tree associated to the path
path->setTree(tree);

CNR_INFO(logger,"Loaded path\n "<<*path);
CNR_INFO(logger,"Loaded tree\n "<<*tree);
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
graph::core::PathOptimizerPtr path_opt = 
  std::make_shared<graph::core::PathLocalOptimizer>(checker,metrics,logger);

path_opt->setPath(cloned_path);

double warp_min_conn_length;
graph::core::get_param(logger,param_ns,"warp_min_conn_length",warp_min_conn_length,0.1);

double warp_min_step_size;
graph::core::get_param(logger,param_ns,"warp_min_step_size",warp_min_step_size,0.1);

path_opt->warp(warp_min_conn_length,warp_min_step_size);
CNR_INFO(logger,"Path after warp \n "<<*cloned_path);

double simplify_max_conn_length;
graph::core::get_param(logger,param_ns,"simplify_max_conn_length",simplify_max_conn_length,0.1);

path_opt->simplify(simplify_max_conn_length);
CNR_INFO(logger,"Path after simplify \n "<<*cloned_path);
```
