#include <ros/ros.h>
#include <graph_core/graph/subtree.h>
#include <graph_core/moveit_collision_checker.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <graph_core/graph/graph_display.h>
#include <graph_core/solvers/anytime_rrt.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_AnytimeRRT");
  ros::NodeHandle nh;

  ros::AsyncSpinner aspin(4);

  std::string group_name = "cartesian_arm";
  moveit::planning_interface::MoveGroupInterface move_group(group_name);
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr       kinematic_model = robot_model_loader.getModel();
  planning_scene::PlanningScenePtr planning_scene = std::make_shared<planning_scene::PlanningScene>(kinematic_model);

  pathplan::CollisionCheckerPtr checker = std::make_shared<pathplan::MoveitCollisionChecker>(planning_scene, group_name);
  pathplan::MetricsPtr metrics=std::make_shared<pathplan::Metrics>();

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
  pathplan::Display display(planning_scene,group_name);

  pathplan::AnytimeRRTPtr solver = std::make_shared<pathplan::AnytimeRRT>(metrics,checker,sampler);

  Eigen::Vector3d start_conf;
  Eigen::Vector3d goal_conf;
  start_conf = {0.0,0.0,0.0};
  goal_conf = {0.8,0.8,0.8};

  pathplan::NodePtr start_node = std::make_shared<pathplan::Node>(start_conf);
  pathplan::NodePtr goal_node  = std::make_shared<pathplan::Node>(goal_conf);

  pathplan::PathPtr solution;

  bool success = solver->computePath(start_node, goal_node, nh, solution);

  if(success)
    display.displayPath(solution,100);

  return 0;
}
