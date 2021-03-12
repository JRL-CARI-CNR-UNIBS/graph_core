#include <ros/ros.h>
#include <graph_core/graph/node.h>
#include <graph_core/graph/graph_display.h>
#include <graph_core/moveit_collision_checker.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "node_collision_check");
  ros::AsyncSpinner spinner(4);
  spinner.start();

  ros::NodeHandle nh;

  // //////////////////////////////////////// GETTING ROS PARAM ///////////////////////////////////////////////

  double checker_resolution;
  if (!nh.getParam("checker_resolution",checker_resolution))
  {
    ROS_ERROR("checker_resolution not set, set equal to 0.05");
    checker_resolution = 0.05;
  }

  std::string group_name;
  if (!nh.getParam("group_name",group_name))
  {
    ROS_ERROR("group_name not set, exit");
    return 0;
  }

  std::string base_link;
  if (!nh.getParam("base_link",base_link))
  {
    ROS_ERROR("base_link not set, exit");
    return 0;
  }

  std::string last_link;
  if (!nh.getParam("last_link",last_link))
  {
    ROS_ERROR("last_link not set, exit");
    return 0;
  }

  std::string test;
  if (!nh.getParam("test",test))
  {
    ROS_ERROR("test not set, set "" ");
    test = "";
  }

  std::vector<double> start_configuration;
  if (!nh.getParam("start_configuration",start_configuration))
  {
    ROS_ERROR("start_configuration not set");
    return 0;
  }

  std::vector<double> goal_configuration;
  if (!nh.getParam("goal_configuration",goal_configuration))
  {
    ROS_ERROR("goal_configuration not set");
    return 0;
  }

  // /////////////////////////////////UPLOADING THE ROBOT ARM/////////////////////////////////////////////////////////////

  moveit::planning_interface::MoveGroupInterface move_group(group_name);
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
  planning_scene::PlanningScenePtr planning_scene = std::make_shared<planning_scene::PlanningScene>(kinematic_model);

  const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(group_name);
  std::vector<std::string> joint_names = joint_model_group->getActiveJointModelNames();

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

  // ///////////////////////////////////UPDATING THE PLANNING STATIC SCENE////////////////////////////////////
  ros::ServiceClient ps_client=nh.serviceClient<moveit_msgs::GetPlanningScene>("/get_planning_scene");

  if (!ps_client.waitForExistence(ros::Duration(10)))
  {
    ROS_ERROR("unable to connect to /get_planning_scene");
    return 1;
  }

  moveit_msgs::GetPlanningScene ps_srv;

  if (!ps_client.call(ps_srv))
  {
    ROS_ERROR("call to srv not ok");
    return 1;
  }

  if (!planning_scene->setPlanningSceneMsg(ps_srv.response.scene))
  {
    ROS_ERROR("unable to update planning scene");
    return 1;
  }

  // //////////////////////////////////////////PATH PLAN & VISUALIZATION////////////////////////////////////////////////////////

  pathplan::CollisionCheckerPtr checker = std::make_shared<pathplan::MoveitCollisionChecker>(planning_scene, group_name, checker_resolution);

  Eigen::VectorXd start = Eigen::Map<Eigen::VectorXd>(start_configuration.data(), start_configuration.size());
  Eigen::VectorXd goal = Eigen::Map<Eigen::VectorXd>(goal_configuration.data(), goal_configuration.size());

  pathplan::SamplerPtr sampler = std::make_shared<pathplan::InformedSampler>(start, goal, lb, ub);
  sampler->setCost(std::numeric_limits<double>::infinity()); //to sample the whole joints space

  pathplan::Display disp = pathplan::Display(planning_scene,group_name,last_link);
  ros::Duration(2).sleep();
  std::vector<double> marker_color = {1.0,0.0,0.0,1.0};
  std::vector<double> marker_size = {0.05,0.05,0.05};
  disp.changeNodeSize(marker_size);
  int id = 0;
  disp.displayNode(std::make_shared<pathplan::Node>(start),id,"pathplan",marker_color);
  marker_color = {0.0,1.0,0.0,1.0};
  id++;
  disp.displayNode(std::make_shared<pathplan::Node>(goal),id,"pathplan",marker_color);
  disp.defaultNodeSize();

  bool success_in;
  ros::WallTime tic_in = ros::WallTime::now();
  success_in = checker->check(start);
  ros::WallTime toc_in = ros::WallTime::now();

  ROS_INFO_STREAM("TIME start: "<<(toc_in-tic_in).toSec()<<" success: "<<success_in);

  tic_in = ros::WallTime::now();
  success_in = checker->check(goal);
  toc_in = ros::WallTime::now();

  ROS_INFO_STREAM("TIME goal: "<<(toc_in-tic_in).toSec()<<" success: "<<success_in);

  marker_color = {0.0,0.0,1.0,1.0};
  int n_succ = 0;
  int n_fail = 0;
  int count = 0;
  std::vector<double> time_vec;
  double max_time =0;

  for(unsigned int i=0;i<1000;i++)
  {
    Eigen::VectorXd sample = sampler->sample();

    bool success;
    ros::WallTime tic = ros::WallTime::now();
    success = checker->check(sample);
    ros::WallTime toc = ros::WallTime::now();

    double time = (toc-tic).toSec();
    if(time>max_time) max_time = time;
    if(time >0.001)count++;
    time_vec.push_back(time);
    if(success)n_succ++;
    else n_fail++;

    id++;
    disp.displayNode(std::make_shared<pathplan::Node>(sample),id,"pathplan",marker_color);
    //ROS_INFO_STREAM("iter: "<<i<<" success:: "<<success<<" time: "<<time);
    ros::Duration(0.001).sleep();
  }

  double time_avg = 0;
  for(const double &time:time_vec) time_avg +=time;
  time_avg = time_avg/(double) time_vec.size();

  ROS_INFO_STREAM("time avg: "<<time_avg<<" max_time: "<<max_time<<" count>0.001: "<<count<<" free: "<<n_succ<<" coll: "<<n_fail);

  bool success;
  ros::WallTime tic = ros::WallTime::now();
  success = checker->checkPath(start,goal);
  ros::WallTime toc = ros::WallTime::now();

  ROS_INFO_STREAM("checker resol: "<<checker_resolution);
  ROS_INFO_STREAM("time path: "<<(toc-tic).toSec()<<" cost: "<<(goal-start).norm()<<" success: "<<success);

  return 0;
}

