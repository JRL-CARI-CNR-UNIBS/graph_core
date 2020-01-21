#include <ros/ros.h>
#include <graph_core/metrics.h>
#include <graph_core/graph/node.h>
#include <graph_core/graph/path.h>
#include <graph_core/graph/tree.h>
#include <graph_core/solvers/rrt_connect.h>
#include <graph_core/solvers/birrt.h>
#include <graph_core/solvers/rrt_star.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "node");
  ros::NodeHandle nh;

  Eigen::VectorXd start_conf(3);
  start_conf.setConstant(0);
  start_conf(0)=1.1;
  start_conf(1)=0;
  start_conf(2)=0;

  pathplan::NodePtr start_node=std::make_shared<pathplan::Node>(start_conf);

  pathplan::MetricsPtr metrics=std::make_shared<pathplan::Metrics>();
  pathplan::CollisionCheckerPtr checker=std::make_shared<pathplan::Cube3dCollisionChecker>();
//  pathplan::CollisionCheckerPtr checker=std::make_shared<pathplan::CollisionChecker>();

  Eigen::VectorXd goal_conf(3);
  goal_conf.setConstant(0);
  goal_conf(0)=-0.6;
  goal_conf(1)=1.1;
  goal_conf(2)=0;
  pathplan::NodePtr goal_node=std::make_shared<pathplan::Node>(goal_conf);

  Eigen::VectorXd lb(3);
  Eigen::VectorXd ub(3);
  lb.setConstant(-M_PI);
  ub.setConstant(M_PI);
  pathplan::SamplerPtr sampler=std::make_shared<pathplan::InformedSampler>(start_conf,goal_conf,lb,ub);


  ROS_INFO("Test solver");
//  pathplan::RRTConnect solver(metrics,checker,sampler);
  pathplan::BiRRT solver(metrics,checker,sampler);
  solver.config(nh);
  solver.addGoal(goal_node);
  solver.addStart(start_node);

  pathplan::PathPtr solution;
  if (!solver.solve(solution,1000))
  {
    ROS_INFO("No solutions found");
    return 0;
  }

  ROS_INFO_STREAM("found a solution with cost = "<<solution->cost());
  double rrt_cost=solution->cost();

  for (unsigned int ipath=0;ipath<10;ipath++)
  {
    bool improve_warp=solution->warp();
    ROS_INFO_STREAM("warp improve to cost = "<<solution->cost());
    bool improve_child=solution->slipChild();
    ROS_INFO_STREAM("slipChild improve to cost = "<<solution->cost());
    bool improve_parent=solution->slipParent();
    ROS_INFO_STREAM("slipParent improve to cost = "<<solution->cost());
    if (!(improve_child || improve_parent || improve_warp))
      break;

  }

  sampler->setCost(solution->cost());

  ROS_INFO("Improving solution with RRT*");
  pathplan::RRTStar opt_solver(metrics,checker,sampler);
  opt_solver.addStartTree(solver.getStartTree());
  opt_solver.addGoal(goal_node);
  opt_solver.config(nh);

  for (unsigned int idx=0;idx<1000;idx++)
  {
    if (opt_solver.update(solution))
    {
      for (unsigned int ipath=0;ipath<100;ipath++)
      {
        bool improve_warp=solution->warp();
        bool improve_child=solution->slipChild();
        bool improve_parent=solution->slipParent();

        if (!(improve_child || improve_parent || improve_warp))
          break;

      }

      sampler->setCost(solution->cost());

    }
  }

  double opt_cost=solution->cost();
  ROS_INFO_STREAM("improve solution to cost = "<<solution->cost());

  ROS_INFO_STREAM("solution\n"<<*solution);
  return 0;
}
