#include <ros/ros.h>
#include <graph_core/metrics.h>
#include <graph_core/graph/node.h>
#include <graph_core/graph/path.h>
#include <graph_core/graph/tree.h>
#include <graph_core/solvers/rrt_connect.h>
#include <graph_core/solvers/multigoal.h>
#include <graph_core/solvers/rrt_star.h>
#include <graph_core/solvers/path_solver.h>
#include <graph_core/tube_informed_sampler.h>
#include <graph_core/boxes_checker.h>

#include <iostream>
#include <fstream>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "multigoal");
  ros::NodeHandle nh("~");

  std::mt19937 m_gen(time(0));
  std::uniform_real_distribution<double> m_ud;

  int dof=6;
  double box_size=0.9;
  double boxes_distance=1.0;
  double bound=5.0;

  Eigen::VectorXd lb(dof);
  Eigen::VectorXd ub(dof);
  lb.setConstant(-bound);
  ub.setConstant(bound);


  pathplan::CollisionCheckerPtr checker = std::make_shared<pathplan::BoxesChecker>(box_size,boxes_distance);
  pathplan::MetricsPtr metrics = std::make_shared<pathplan::Metrics>();

  Eigen::VectorXd start_conf(dof);
  for (int it=0;it<100;it++)
  {
    start_conf.setRandom();
    start_conf*=bound;
    if (checker->check(start_conf))
      break;
  }
  pathplan::NodePtr start_node = std::make_shared<pathplan::Node>(start_conf);


  pathplan::SamplerPtr sampler = std::make_shared<pathplan::InformedSampler>(lb, ub, lb, ub);


  pathplan::TreeSolverPtr solver=std::make_shared<pathplan::MultigoalSolver>(metrics, checker, sampler);
  solver->config(nh);
  solver->addStart(start_node);

  Eigen::VectorXd goal_conf(dof);
  for (int it=0;it<250;it++)
  {
    goal_conf.setRandom();
    goal_conf*=bound;
    if ((goal_conf-start_conf).norm()<boxes_distance*3.0)
      continue;

    pathplan::NodePtr goal_node = std::make_shared<pathplan::Node>(goal_conf);

    solver->addGoal(goal_node);

  }


  ros::Rate lp(100);
  pathplan::PathPtr solution;
  ros::WallTime t0=ros::WallTime::now();
  while (ros::ok())
  {
    solver->update(solution);

    ROS_INFO_STREAM_THROTTLE(3,"time = "<< (ros::WallTime::now()-t0).toSec() << ".\nSolver: " << *solver);
    if (solver->completed())
    {
      ROS_INFO_STREAM("time = "<< (ros::WallTime::now()-t0).toSec() << ".\nSolver: " << *solver);
      return 0;
    }
    lp.sleep();
  }

  return 0;
}
