#include "test_utils.h"
// Graph core required headers
#include <graph_core/solvers/rrt.h>
#include <graph_core/samplers/uniform_sampler.h>
#include <graph_core/metrics/euclidean_metrics.h>
#include <graph_core/collision_checkers/cube_3d_collision_checker.h>
#include <graph_core/solvers/path_optimizers/path_local_optimizer.h>

int main(int argc, char** argv)
{
  std::string path_to_config_folder = std::string(TEST_DIR);

  // Logger: the log level can be passed as first argument
  graph::core::LoggerPtr logger = graph_core_tests::makeLogger("path_post_processing_test", argc, argv);
  if (not logger)
    return 1;

  // Load the parameters
  std::string param_ns = "graph_core_tests";
  graph::core::ParamMap params =
      graph_core_tests::loadParams(path_to_config_folder + "/graph_core_tests_params.yaml", param_ns);

  // Define the collision checker (foo collision checker)
  double min_cc_distance;
  double default_min_cc_distance = 0.01;
  graph::core::get_param(logger, params, "min_cc_distance", min_cc_distance, default_min_cc_distance);

  if (min_cc_distance <= 0)
    min_cc_distance = default_min_cc_distance;

  graph::core::CollisionCheckerPtr collision_checker =
      std::make_shared<graph::core::Cube3dCollisionChecker>(logger, min_cc_distance);

  // Define a cost function (Euclidean metrics)
  graph::core::MetricsPtr metrics = std::make_shared<graph::core::EuclideanMetrics>(logger);

  // Load the path and tree computed in the first tutorial
  // Path and tree are data, not parameters: load them from YAML with fromYAML()
  YAML::Node yaml_path = YAML::LoadFile(path_to_config_folder + "/path.yaml")[param_ns]["path"];
  YAML::Node yaml_tree = YAML::LoadFile(path_to_config_folder + "/tree.yaml")[param_ns]["tree"];

  graph::core::PathPtr path = graph::core::Path::fromYAML(yaml_path, metrics, collision_checker, logger);
  graph::core::TreePtr tree = graph::core::Tree::fromYAML(yaml_tree, metrics, collision_checker, logger);
  if (not path || not tree)
  {
    GC_ERROR(logger, "Cannot load path.yaml / tree.yaml (run compute_path_test first)");
    return 1;
  }

  // Set the tree as the tree associated to the path
  path->setTree(tree);

  GC_INFO(logger, "Loaded path\n " << *path);
  GC_INFO(logger, "Loaded tree\n " << *tree);

  // Do some example operations on the path, but to keep integrity of the original path with the tree
  // work on a clone of the path. The clone has the same waypoints but stored within different nodes instances
  // and does not have any tree associated. It shares the same metrics and collision checker with the original path.
  // However, you can clone also them if you want to work on different instances. We will not do it here.
  graph::core::PathPtr cloned_path = path->clone();

  // Flip the path
  cloned_path->flip();
  GC_INFO(logger, "Flipped path\n " << *cloned_path);

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

  return 0;
}
