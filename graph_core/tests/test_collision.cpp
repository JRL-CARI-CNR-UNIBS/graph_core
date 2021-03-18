#include <ros/ros.h>

#include <graph_core/moveit_collision_checker.h>
#include <graph_core/parallel_moveit_collision_checker.h>
#include <graph_core/sampler.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "node");
  ros::NodeHandle nh;

  ros::AsyncSpinner aspin(4);

  std::string group_name = "manipulator";
  moveit::planning_interface::MoveGroupInterface move_group(group_name);
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr       kinematic_model = robot_model_loader.getModel();
  planning_scene::PlanningScenePtr planning_scene = std::make_shared<planning_scene::PlanningScene>(kinematic_model);

  robot_state::RobotState state = planning_scene->getCurrentState();

  //pathplan::CollisionCheckerPtr checker = std::make_shared<pathplan::MoveitCollisionChecker>(planning_scene, group_name);
  pathplan::CollisionCheckerPtr checker = std::make_shared<pathplan::ParallelMoveitCollisionChecker>(planning_scene, group_name,50);

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

  int iters=100000;
  double time=0;
  for (int idx=0;idx<iters;idx++)
  {
    Eigen::VectorXd q1=sampler->sample();
    Eigen::VectorXd q2=sampler->sample();
    ros::WallTime t0=ros::WallTime::now();
    checker->checkPath(q1,q2);
    ros::WallTime t1=ros::WallTime::now();
    time+=(t1-t0).toSec();
  }
  ROS_INFO("Average time = %f ms on %d attempts",1e3*time/(double)iters,iters);
  return 0;
}
