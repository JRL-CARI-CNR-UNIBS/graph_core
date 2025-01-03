// Graph core required headers
#include <graph_core/solvers/rrt.h>
#include <graph_core/samplers/uniform_sampler.h>
#include <graph_core/metrics/euclidean_metrics.h>
#include <graph_core/collision_checkers/cube_3d_collision_checker.h>
#include <graph_core/solvers/path_optimizers/path_local_optimizer.h>

int main(int argc, char** argv)
{
  // Load the logger's configuration
  std::string logger_file = std::string(TEST_DIR) + "/logger_param.yaml";
  ;
  cnr_logger::TraceLoggerPtr logger = std::make_shared<cnr_logger::TraceLogger>("graph_core_tests_logger", logger_file);

  // Load params
  std::string command = "cnr_param_server --path-to-file " + std::string(TEST_DIR) + "/graph_core_tests_params.yaml";
  CNR_INFO(logger, "Executing command: " << command);
  int ret_code = std::system(command.c_str());
  if (ret_code != 0)
  {
    CNR_ERROR(logger, "Error: command " << command << " failed");
    return ret_code;
  }

  command = "cnr_param_server --path-to-file " + std::string(TEST_DIR) + "/path.yaml";
  CNR_INFO(logger, "Executing command: " << command);
  ret_code = std::system(command.c_str());
  if (ret_code != 0)
  {
    CNR_ERROR(logger, "Error: command " << command << " failed");
    return ret_code;
  }

  command = "cnr_param_server --path-to-file " + std::string(TEST_DIR) + "/tree.yaml";
  CNR_INFO(logger, "Executing command: " << command);
  ret_code = std::system(command.c_str());
  if (ret_code != 0)
  {
    CNR_ERROR(logger, "Error: command " << command << " failed");
    return ret_code;
  }

  // Define namespace for parameters retrieving
  std::string param_ns = "/graph_core_tests";  // must begin with "/"

  // Define the collision checker (foo collision checker)
  double min_cc_distance;
  double default_min_cc_distance = 0.01;
  graph::core::get_param(logger, param_ns, "min_cc_distance", min_cc_distance,
                         min_cc_distance);  // wrapper to cnr_param functions

  if (min_cc_distance <= 0)
    min_cc_distance = default_min_cc_distance;

  graph::core::CollisionCheckerPtr collision_checker =
      std::make_shared<graph::core::Cube3dCollisionChecker>(logger, min_cc_distance);

  // Define a cost function (Euclidean metrics)
  graph::core::MetricsPtr metrics = std::make_shared<graph::core::EuclideanMetrics>(logger);

  // Load the path and tree computed in the first tutorial
  graph::core::PathPtr path;
  graph::core::get_param(logger, param_ns, "path", path, metrics, collision_checker);

  graph::core::TreePtr tree;
  graph::core::get_param(logger, param_ns, "tree", tree, metrics, collision_checker);

  // Set the tree as the tree associated to the path
  path->setTree(tree);

  CNR_INFO(logger, "Loaded path\n " << *path);
  CNR_INFO(logger, "Loaded tree\n " << *tree);

  // Do some example operations on the path, but to keep integrity of the original path with the tree
  // work on a clone of the path. The clone has the same waypoints but stored within different nodes instances
  // and does not have any tree associated. It shares the same metrics and collision checker with the original path.
  // However, you can clone also them if you want to work on different instances. We will not do it here.
  graph::core::PathPtr cloned_path = path->clone();

  // Flip the path
  cloned_path->flip();
  CNR_INFO(logger, "Flipped path\n " << *cloned_path);

  // Apply some post processing on the path (warp and simplify)
  graph::core::PathLocalOptimizerPtr path_opt =
      std::make_shared<graph::core::PathLocalOptimizer>(collision_checker, metrics, logger);

  path_opt->setPath(cloned_path);

  double warp_min_conn_length;
  double default_warp_min_conn_length = 0.1;
  graph::core::get_param(logger, param_ns, "warp_min_conn_length", warp_min_conn_length, default_warp_min_conn_length);

  double warp_min_step_size;
  double default_warp_min_step_size = 0.05;
  graph::core::get_param(logger, param_ns, "warp_min_step_size", warp_min_step_size, default_warp_min_step_size);

  path_opt->warp(warp_min_conn_length, warp_min_step_size);
  CNR_INFO(logger, "Path after warp \n " << *cloned_path);

  double simplify_max_conn_length;
  double default_simplify_max_conn_length = 0.05;
  graph::core::get_param(logger, param_ns, "simplify_max_conn_length", simplify_max_conn_length,
                         default_simplify_max_conn_length);

  path_opt->simplify(simplify_max_conn_length);
  CNR_INFO(logger, "Path after simplify \n " << *cloned_path);

  return 0;
}
