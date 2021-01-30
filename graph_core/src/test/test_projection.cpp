#include <ros/ros.h>
#include <graph_core/metrics.h>
#include <graph_core/occupancy_metrics.h>
#include <graph_core/graph/node.h>
#include <graph_core/graph/path.h>
#include <graph_core/graph/tree.h>
#include<graph_core/graph/graph_display.h>
#include<graph_core/graph/trajectory.h>
#include <graph_core/solvers/rrt_connect.h>
#include <graph_core/solvers/birrt.h>
#include <graph_core/solvers/rrt_star.h>
#include <graph_core/solvers/path_solver.h>
#include <graph_core/moveit_collision_checker.h>
#include <graph_core/local_informed_sampler.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <rviz_visual_tools/rviz_visual_tools.h>
#include <eigen_conversions/eigen_msg.h>
#include <moveit_planning_helper/spline_interpolator.h>
#include <thread>
#include <mutex>

Eigen::VectorXd current_configuration;
planning_scene::PlanningScenePtr planning_scn;
pathplan::TrajectoryPtr trajectory;
pathplan::DisplayPtr disp;
moveit_msgs::RobotTrajectory trj_msg;
double t=0;
double dt=0.01;
double main_frequency = 1/dt;

trajectory_processing::SplineInterpolator interpolator;
trajectory_msgs::JointTrajectoryPoint pnt;
pathplan::MetricsPtr metrics = std::make_shared<pathplan::Metrics>();
pathplan::CollisionCheckerPtr checker;

pathplan::PathPtr current_path;

int n_conn = 0;


int main(int argc, char **argv)
{
  std::string test = "cartesian";

  ros::init(argc, argv, "node_prj");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::NodeHandle nh;
  ros::Publisher target_pub=nh.advertise<sensor_msgs::JointState>("/joint_target",1);
  sensor_msgs::JointState jt_msg;

  std::string group_name;
  std::string base_link;
  std::string last_link;

  if(test == "panda")
  {
    group_name = "panda_arm";
    base_link = "panda_link0";
    last_link = "panda_link8";
  }

  if(test == "cartesian")
  {
    group_name = "cartesian_arm";
    base_link = "world";
    last_link = "end_effector";
  }

  moveit::planning_interface::MoveGroupInterface move_group(group_name);
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
  planning_scn = std::make_shared<planning_scene::PlanningScene>(kinematic_model);

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

  Eigen::VectorXd start_conf(dof);
  if(test == "panda") start_conf << 0.0,0.0,0.0,-1.5,0.0,1.5,-1.0;
  if(test == "cartesian") start_conf << 0.0,0.0,0.0;

  Eigen::VectorXd goal_conf(dof);
  if(test == "panda") goal_conf <<  1.5,0.5,0.0,-1.0,0.0,2.0,-1.0;
  if(test == "cartesian") goal_conf <<  0.8,0.8,0.8;

  pathplan::NodePtr start_node = std::make_shared<pathplan::Node>(start_conf);

  checker = std::make_shared<pathplan::MoveitCollisionChecker>(planning_scn, group_name);

  // ///////////////////////////PATH PLAN & VISUALIZATION//////////////////////////////////////////////////////////////////////////

  disp = std::make_shared<pathplan::Display>(planning_scn,group_name,last_link);
  pathplan::PathPtr path = NULL;
  trajectory = std::make_shared<pathplan::Trajectory>(path,nh,planning_scn,group_name,base_link,last_link);

  int current_node_id = -1;
  int trj_node_id = -2;

  pathplan::NodePtr goal_node = std::make_shared<pathplan::Node>(goal_conf);

  pathplan::PathPtr path1 = trajectory->computeBiRRTPath(start_node, goal_node, lb, ub, metrics, checker, 1);
  pathplan::PathPtr path2 = trajectory->computeBiRRTPath(goal_node, start_node, lb, ub, metrics, checker, 1);

  // ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  std::vector<pathplan::ConnectionPtr> conn = path1->getConnections();
  conn.insert(conn.end(),path2->getConnectionsConst().begin(),path2->getConnectionsConst().end()-1);
  current_path = std::make_shared<pathplan::Path>(conn,metrics,checker);

  std::vector<double> marker_color;
  marker_color = {0.5,0.5,0.0,1.0};
  disp->displayPathAndWaypoints(current_path,"pathplan",marker_color);

  pathplan::SamplerPtr samp = std::make_shared<pathplan::InformedSampler>(start_conf, goal_conf, lb, ub);
  pathplan::BiRRTPtr solver = std::make_shared<pathplan::BiRRT>(metrics, checker, samp);
  solver->config(nh);

  trajectory->setPath(current_path);
  robot_trajectory::RobotTrajectoryPtr trj= trajectory->fromPath2Trj();
  moveit_msgs::RobotTrajectory tmp_trj_msg;
  trj->getRobotTrajectoryMsg(tmp_trj_msg);

  double MAX_TIME = trj->getDuration();

  interpolator.setTrajectory(tmp_trj_msg);
  interpolator.setSplineOrder(1);

  Eigen::VectorXd point2project(dof);

  interpolator.interpolate(ros::Duration(t),pnt);
  current_configuration = start_conf;
  Eigen::VectorXd past_configuration = current_configuration;

  for(unsigned int i=0;i<current_path->getConnections().size()-1;i++)
  {
    if(current_path->getConnections().at(i)->getChild()->getConfiguration() != current_path->getConnections().at(i+1)->getParent()->getConfiguration())
    {
      ROS_ERROR("Current_path disconnesso!");
      assert(0);
    }
  }

  double stop = false;
  ros::Rate lp(main_frequency);
  while (ros::ok() && !stop)
  {
    t+=dt;

    interpolator.interpolate(ros::Duration(t),pnt);
    for(unsigned int i=0; i<pnt.positions.size();i++) point2project[i] = pnt.positions.at(i);

    jt_msg.velocity=pnt.velocities;
    jt_msg.header.stamp=ros::Time::now();
    target_pub.publish(jt_msg);

    past_configuration = current_configuration;
    current_configuration = current_path->projectOnClosestConnectionKeepingCurvilinearAbscissa(point2project,past_configuration,n_conn);
    //current_configuration = current_path->projectOnClosestConnectionKeepingPastPrj(point2project,past_configuration,n_conn);
    //current_configuration = current_path->projectOnClosestConnection(point2project);

    if(t>=MAX_TIME) stop = true;
    else
    {
      std::vector<double> marker_scale_sphere_actual(3,0.02);
      std::vector<double> marker_color_sphere_actual = {1.0,0.0,1.0,1.0};

      disp->changeNodeSize(marker_scale_sphere_actual);
      disp->displayNode(std::make_shared<pathplan::Node>(current_configuration),current_node_id,"pathplan",marker_color_sphere_actual);

      marker_color_sphere_actual = {0.0,1.0,0.0,1.0};
      disp->displayNode(std::make_shared<pathplan::Node>(point2project),trj_node_id,"pathplan",marker_color_sphere_actual);
      disp->defaultNodeSize();
    }

    lp.sleep();
  }

  return 0;
}

