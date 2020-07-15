#include <ros/ros.h>
#include <graph_core/metrics.h>
#include <graph_core/graph/node.h>
#include <graph_core/graph/path.h>
#include <graph_core/graph/tree.h>
#include <graph_core/solvers/rrt_connect.h>
#include <graph_core/solvers/birrt.h>
#include <graph_core/solvers/rrt_star.h>
#include <graph_core/solvers/path_solver.h>
#include <graph_core/tube_informed_sampler.h>
int main(int argc, char **argv)
{
  ros::init(argc, argv, "node");
  ros::NodeHandle nh;

  Eigen::VectorXd start_conf(3);
  start_conf.setConstant(0);
  start_conf(0) = 1.1;
  start_conf(1) = 0;
  start_conf(2) = 0;

  pathplan::NodePtr start_node = std::make_shared<pathplan::Node>(start_conf);
  pathplan::MetricsPtr metrics = std::make_shared<pathplan::Metrics>();
  pathplan::CollisionCheckerPtr checker = std::make_shared<pathplan::Cube3dCollisionChecker>();

  Eigen::VectorXd goal_conf(3);
  goal_conf.setConstant(0);
  goal_conf(0) = -0.6;
  goal_conf(1) = 1.1;
  goal_conf(2) = 0;
  pathplan::NodePtr goal_node = std::make_shared<pathplan::Node>(goal_conf);

  Eigen::VectorXd lb(3);
  Eigen::VectorXd ub(3);
  lb.setConstant(-M_PI);
  ub.setConstant(M_PI);
  pathplan::SamplerPtr sampler = std::make_shared<pathplan::InformedSampler>(start_conf, goal_conf, lb, ub);


  ROS_INFO("Test solver");
  pathplan::BiRRT solver(metrics, checker, sampler);
  solver.config(nh);
  solver.addGoal(goal_node);
  solver.addStart(start_node);


  pathplan::PathPtr solution;
  if (!solver.solve(solution, 1000))
  {
    ROS_INFO("No solutions found");
    return 0;
  }
  ROS_INFO_STREAM("Cost = "<<solution->cost());

  double radius=0.3;//*solution->cost();
  pathplan::TubeInformedSamplerPtr tube_sampler = std::make_shared<pathplan::TubeInformedSampler>(start_conf, goal_conf, lb, ub);
  tube_sampler->setPath(solution);
  tube_sampler->setRadius(radius);
  tube_sampler->setCost(solution->cost());

  ROS_INFO("Improving solution with RRT*");
  pathplan::RRTStar opt_solver(metrics, checker, tube_sampler);
//  pathplan::RRTStar opt_solver(metrics, checker, sampler);
  opt_solver.addStartTree(solver.getStartTree());
  opt_solver.addGoal(goal_node);
  opt_solver.config(nh);


  std::vector<pathplan::NodePtr> white_list;
  white_list.push_back(goal_node);


  for (size_t idx=0;idx<1000;idx++)
  {
    if (!ros::ok())
      break;

    if (opt_solver.solve(solution,10))
    {
      tube_sampler->setPath(solution);
      tube_sampler->setCost(solution->cost());
      sampler->setCost(solution->cost());
      ROS_INFO_STREAM("Cost improved to = "<<solution->cost() << " during iteration "<< idx << ". removed nodes: " << opt_solver.getStartTree()->purgeNodes(sampler, white_list, true));
    }
  }
  ROS_INFO("End of tube sampling test node");
  ROS_INFO_STREAM("path\n"<<*solution);
  return 0;
}
