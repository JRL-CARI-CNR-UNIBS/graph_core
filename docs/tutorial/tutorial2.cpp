// Graph core required headers
#include <graph_core/solvers/rrt.h>
#include <graph_core/samplers/uniform_sampler.h>
#include <graph_core/metrics/euclidean_metrics.h>
#include <graph_core/collision_checkers/cube_3d_collision_checker.h>
#include <graph_core/solvers/path_optimizers/path_local_optimizer.h>

int main(int argc, char **argv)
{
  // Load the logger's configuration
  std::string path_to_config_folder = "path/to/config/folder"
  std::string logger_file = path_to_config_folder+"/logger_param.yaml";
  cnr_logger::TraceLoggerPtr logger = std::make_shared<cnr_logger::TraceLogger>("graph_core_tutorial_loggers",logger_file);

  // Define namespace for parameters retrieving
  std::string param_ns = "/graph_core_tutorial"  // must begin with "/"

  // Define the collision checker (foo collision checker)
  double min_cc_distance;
  double default_min_cc_distance = 0.01,
  graph::core::get_param(logger,param_ns,"min_cc_distance",min_cc_distance,min_cc_distance); //wrapper to cnr_param functions
  
  if(min_cc_distance<=0)
    min_cc_distance = default_min_cc_distance;

   graph::collision_check::CollisionCheckerPtr collision_checker = 
    std::make_shared<graph::collision_check::Cube3dCollisionChecker>(logger, min_cc_distance);

  // Define a cost function (Euclidean metrics)
  graph::core::MetricsPtr metrics = std::make_shared<graph::core::EuclideanMetrics>(logger);

  // Load the path and tree computed in the first tutorial
  graph::core::PathPtr path;
  graph::core::get_param(logger,param_ns,"path",path,metrics,checker);

  graph::core::TreePtr tree;
  graph::core::get_param(logger,param_ns,"tree",tree,metrics,checker);

  // Set the tree as the tree associated to the path
  path->setTree(tree);

  CNR_INFO(logger,"Loaded path\n "<<*path);
  CNR_INFO(logger,"Loaded tree\n "<<*tree);

  // Do some example operations on the path, but to keep integrity of the original path with the tree
  // work on a clone of the path. The clone has the same waypoints but stored within different nodes instances
  // and does not have any tree associated. It shares the same metrics and collision checker with the original path.
  // However, you can clone also them if you want to work on different instances. We will not do it here.
  graph::core::PathPtr cloned_path = path->clone();

  // Flip the path
  cloned_path->flip();
  CNR_INFO(logger,"Flipped path\n "<<*cloned_path);

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

  return 0;
}

