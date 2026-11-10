#include "test_utils.h"
// Graph core required headers
#include <graph_core/solvers/rrt_star.h>
#include <graph_core/samplers/uniform_sampler.h>
#include <graph_core/metrics/euclidean_metrics.h>
#include <graph_core/collision_checkers/cube_3d_collision_checker.h>

int main(int argc, char** argv)
{
  std::string path_to_config_folder = std::string(TEST_DIR);

  // Logger: the log level can be passed as first argument
  graph::core::LoggerPtr logger = graph_core_tests::makeLogger("compute_path_test", argc, argv);
  if (not logger)
    return 1;

  // Load the parameters. graph_core does not read parameters by itself: the application fills a ParamMap
  // (here from a YAML file, in a ROS node from the node parameters) and passes it to the objects.
  std::string param_ns_base = "graph_core_tests";
  graph::core::ParamMap params =
      graph_core_tests::loadParams(path_to_config_folder + "/graph_core_tests_params.yaml", param_ns_base);

  // Define the collision checker (foo collision checker)
  double min_cc_distance;  // distance between two configurations to be checked along a connection
  double default_min_cc_distance = 0.01;
  graph::core::get_param(logger, params, "min_cc_distance", min_cc_distance, default_min_cc_distance);

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

  // Compute a path
  double max_time;  // seconds
  double default_max_time = 5.0;
  graph::core::get_param(logger, params, "max_time", max_time, default_max_time);

  size_t max_iter = 1000000;
  graph::core::PathPtr solution;

  graph::core::graph_time_point tic = graph::core::graph_time::now();
  bool found = solver->computePath(start_node, goal_node, params, solution, max_time, max_iter);
  graph::core::graph_time_point toc = graph::core::graph_time::now();

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

  return 0;
}
