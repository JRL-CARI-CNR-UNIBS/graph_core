#include <ros/ros.h>
#include <graph_core/graph/subtree.h>
#include <graph_core/moveit_collision_checker.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <graph_core/graph/graph_display.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_precomputing_tree");
  ros::NodeHandle nh;

  ros::AsyncSpinner aspin(4);

  std::string group_name = "manipulator";
  moveit::planning_interface::MoveGroupInterface move_group(group_name);
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr       kinematic_model = robot_model_loader.getModel();
  planning_scene::PlanningScenePtr planning_scene = std::make_shared<planning_scene::PlanningScene>(kinematic_model);


  double steps=nh.param("steps",0.01);
  pathplan::CollisionCheckerPtr checker = std::make_shared<pathplan::MoveitCollisionChecker>(planning_scene, group_name,steps);
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

  for (int itree=0;itree<10;itree++)
  {
    Eigen::VectorXd q;
    while (ros::ok())
    {
      q=sampler->sample();
      if (checker->check(q))
        break;
    }
    pathplan::NodePtr root = std::make_shared<pathplan::Node>(q);
    pathplan::TreePtr tree=std::make_shared<pathplan::Tree>(root,pathplan::Direction::Forward,1,checker,metrics);
    pathplan::NodePtr new_node;
    for (int idx=0;idx<10000;idx++)
    {
      tree->rewire(sampler->sample(),5.0);
    }
    display.clearMarkers();
    display.displayTree(tree,"pathplan",{1,0,0,0.3});
    ROS_INFO_STREAM("tree  = " << *tree);
    nh.setParam("precomputed/tree_"+std::to_string(itree),tree->toXmlRpcValue());
  }
  return 0;
}
