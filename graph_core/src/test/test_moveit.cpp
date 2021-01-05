#include <ros/ros.h>
#include <graph_core/metrics.h>
#include <graph_core/occupancy_metrics.h>
#include <graph_core/avoidance_metrics.h>
#include <graph_core/graph/node.h>
#include <graph_core/graph/path.h>
#include <graph_core/graph/tree.h>
#include <graph_core/solvers/rrt_connect.h>
#include <graph_core/solvers/multigoal.h>
#include <graph_core/solvers/rrt_star.h>
#include <graph_core/solvers/path_solver.h>
#include <graph_core/moveit_collision_checker.h>
#include <graph_core/local_informed_sampler.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <graph_core/graph/graph_display.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "node");
  ros::NodeHandle nh;

  ros::AsyncSpinner aspin(4);

  std::string group_name = "ur5_on_guide";
  moveit::planning_interface::MoveGroupInterface move_group(group_name);
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr       kinematic_model = robot_model_loader.getModel();
  planning_scene::PlanningScenePtr planning_scene = std::make_shared<planning_scene::PlanningScene>(kinematic_model);

  robot_state::RobotState state = planning_scene->getCurrentState();

  pathplan::Display display_tree(planning_scene,group_name,"ur5_tool0");

//  pathplan::MetricsPtr metrics=std::make_shared<pathplan::Metrics>();
  pathplan::AvoidanceMetricsPtr metrics=std::make_shared<pathplan::AvoidanceMetrics>(nh);
  Eigen::Vector3d human; human << 0.703, 0.396, 1.245;
  metrics->addPoint(human);
  human << 1.000, 0.945, 1.381;
  metrics->addPoint(human);
  human << 0.628, 0.771, 1.351;
  metrics->addPoint(human);


  //  pathplan::MetricsPtr metrics = std::make_shared<pathplan::OccupancyMetrics>(nh);
  pathplan::CollisionCheckerPtr checker = std::make_shared<pathplan::MoveitCollisionChecker>(planning_scene, group_name);

  std::vector<std::string> joint_names = kinematic_model->getJointModelGroup(group_name)->getActiveJointModelNames();

  unsigned int dof = joint_names.size();
  Eigen::VectorXd lb(dof);
  Eigen::VectorXd ub(dof);

  for (unsigned int idx = 0; idx < dof; idx++)
  {
    const robot_model::VariableBounds& bounds = kinematic_model->getVariableBounds(joint_names.at(idx));
    if (bounds.position_bounded_)
    {
      lb(idx) = bounds.min_position_;
      ub(idx) = bounds.max_position_;
    }
  }
  pathplan::SamplerPtr sampler = std::make_shared<pathplan::InformedSampler>(lb, ub, lb, ub);
  pathplan::TreeSolverPtr solver=std::make_shared<pathplan::MultigoalSolver>(metrics, checker, sampler);
  solver->config(nh);

  Eigen::VectorXd start_conf(7);
  start_conf << 1.32, -1.9815160293343739, -1.9785185066912643, -1.3669110234783275, -1.9496418040874854, 1.4566744068999329, -1.1948766522077063;
  pathplan::NodePtr start_node = std::make_shared<pathplan::Node>(start_conf);
  solver->addStart(start_node);
  display_tree.displayNode(start_node,"start_node",{0,0,1,1});

  Eigen::Isometry3d T_b_goal;
  Eigen::Quaterniond q(0.938, -0.322, 0.061, -0.109);
  q.normalize();
  T_b_goal=q;
  T_b_goal.translation() << 0.097, 0.225, 1.35;

  Eigen::VectorXd goal_conf(7);

  ROS_INFO("Computing ik");
  for (unsigned int idx=0;idx<400;idx++)
  {
    state.setToRandomPositions();
    if (!state.setFromIK(kinematic_model->getJointModelGroup(group_name),T_b_goal))
      continue;
    state.copyJointGroupPositions(group_name,goal_conf);
    pathplan::NodePtr goal_node = std::make_shared<pathplan::Node>(goal_conf);
    if (solver->addGoal(goal_node))
      display_tree.displayNode(goal_node,"goal_nodes",{0,1,1,1});
  }

  ROS_INFO("Computing solution");
  pathplan::PathPtr solution;

  ros::WallTime t0=ros::WallTime::now();
  ros::WallTime tl=ros::WallTime::now()-ros::WallDuration(1000);
  ros::Rate lp(100);
  bool solved=false;
  int path_id=-1;
  while (ros::ok())
  {
    solver->update(solution);

    ROS_INFO_STREAM_THROTTLE(3,"time = "<< (ros::WallTime::now()-t0).toSec() << ".\nSolver: " << *solver);
    if (solver->solved())
    {
      solved=true;
    }
    if ((ros::WallTime::now()-tl)>ros::WallDuration(2))
    {
      ROS_INFO("display");
      if (solved)
      {
        if (path_id>=0)
          display_tree.clearMarker(path_id,"solution");
        path_id=display_tree.displayPath(solution,"solution",{0,1,0,1});
      }

      tl=ros::WallTime::now();
    }

    if (solver->completed())
    {
      ROS_INFO_STREAM("time = "<< (ros::WallTime::now()-t0).toSec() << ".\nSolver: " << *solver);
      solved=true;
      break;
    }

    lp.sleep();
  }

  if (!solved)
  {
    ROS_ERROR("unable to solve problem");
    return 0;
  }

  ROS_INFO_STREAM("solution\n" << *solution);

  //std::vector<double> color{1,0,0,0.5};
  display_tree.clearMarkers();
  display_tree.displayPath(solution,{0,1,0,1});
  display_tree.displayTree(solver->getStartTree(),"tree",{0,0,1,1});

  //ros::spin();
  return 0;
}
