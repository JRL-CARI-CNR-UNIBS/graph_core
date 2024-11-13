// Graph core required headers
#include <graph_core/solvers/rrt.h>
#include <graph_core/samplers/uniform_sampler.h>
#include <graph_core/metrics/euclidean_metrics.h>
#include <graph_core/collision_checkers/cube_3d_collision_checker.h>

int main(int argc, char **argv)
{
  // Load the logger's configuration
  std::string path_to_config_folder = "path/to/config/folder";
  std::string logger_file = path_to_config_folder+"/logger_param.yaml";
  cnr_logger::TraceLoggerPtr logger = std::make_shared<cnr_logger::TraceLogger>("graph_core_tutorial_loggers",logger_file);

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

  // Compute a path
  double max_time = 10.0; //seconds
  size_t max_iter = 1000000;
  graph::core::PathPtr solution;
  
  graph::core::graph_time_point tic = graph::core::graph_time::now();
  bool found = solver->computePath(start_node,goal_node,param_ns,solution,max_time,max_iter);
  graph::core::graph_time_point toc = graph::core::graph_time::now();

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

  return 0;
}

