#include <ros/ros.h>
#include <graph_core/metrics.h>
#include <graph_core/graph/node.h>
#include <graph_core/graph/path.h>
#include <graph_core/graph/tree.h>
#include <graph_core/solvers/rrt_connect.h>
#include <graph_core/solvers/birrt.h>
#include <graph_core/solvers/rrt_star.h>
#include <graph_core/solvers/path_solver.h>

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
//  pathplan::CollisionCheckerPtr checker=std::make_shared<pathplan::CollisionChecker>();

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
//  pathplan::RRTConnect solver(metrics,checker,sampler);
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

  double rrt_cost = solution->cost();

  pathplan::PathLocalOptimizer path_solver(checker, metrics);
  path_solver.config(nh);

  ROS_INFO_STREAM("found a solution with cost = " << solution->cost());
  path_solver.setPath(solution);
  path_solver.solve(solution);
  ROS_INFO_STREAM("improve the solution to cost = " << solution->cost());


  sampler->setCost(solution->cost());
  solver.getStartTree()->addBranch(solution->getConnections());
  ROS_INFO("Improving solution with RRT*");
  pathplan::RRTStar opt_solver(metrics, checker, sampler);
  opt_solver.addStartTree(solver.getStartTree());
  opt_solver.addGoal(goal_node);
  opt_solver.config(nh);

  std::vector<pathplan::NodePtr> white_list;
  white_list.push_back(goal_node);


  for (unsigned int idx = 0; idx < 5000; idx++)
  {
    if (opt_solver.update(solution))
    {
      ROS_INFO_STREAM("Iteration = " << idx << " , rrt* reduce cost to = " << solution->cost());
      path_solver.setPath(solution);
      path_solver.solve(solution);
      ROS_INFO_STREAM("local solver improve the solution to cost = " << solution->cost());

      sampler->setCost(solution->cost());
      opt_solver.getStartTree()->purgeNodes(sampler, white_list, true);
    }
    else
      opt_solver.getStartTree()->purgeNodes(sampler, white_list, false);

    if (idx % 100 == 0)
      ROS_INFO("iter=%u,tree with %u nodes", idx, opt_solver.getStartTree()->getNumberOfNodes());
  }

  double opt_cost = solution->cost();
  ROS_INFO_STREAM("solution cost = " << solution->cost());
  ROS_INFO("tree with %u nodes", opt_solver.getStartTree()->getNumberOfNodes());

  ROS_INFO_STREAM("solution\n" << *solution);

  return 0;
}
