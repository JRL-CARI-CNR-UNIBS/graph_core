#include <ros/ros.h>
#include <graph_core/metrics.h>
#include <graph_core/occupancy_metrics.h>
#include <graph_core/graph/node.h>
#include <graph_core/graph/path.h>
#include <graph_core/graph/tree.h>
#include <graph_core/solvers/rrt_connect.h>
#include <graph_core/solvers/birrt.h>
#include <graph_core/solvers/rrt_star.h>
#include <graph_core/solvers/path_solver.h>
#include <graph_core/moveit_collision_checker.h>
#include <graph_core/local_informed_sampler.h>
#include <graph_core/graph/trajectory.h>
#include <graph_core/graph/graph_display.h>
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
#include <graph_core/replanner.h>
#include <moveit_planning_helper/spline_interpolator.h>

//#define COMMENT(...) ROS_LOG(::ros::console::levels::Debug, ROSCONSOLE_DEFAULT_NAME, __VA_ARGS__)

int main(int argc, char **argv)
{
  //std::string test = "sharework";
  std::string test = "panda";

  unsigned int n_paths = 3;

  std::vector<rviz_visual_tools::colors> colors {rviz_visual_tools::GREEN, rviz_visual_tools::BLUE, rviz_visual_tools::RED};
  uint32_t shape = visualization_msgs::Marker::LINE_STRIP;

  ros::init(argc, argv, "node_replanner");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::NodeHandle nh;

  ros::Publisher display_publisher = nh.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
  ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("/marker_visualization_topic", 1);

  std::string group_name;
  std::string base_link;
  std::string last_link;

  if(test == "sharework")
  {
    group_name = "manipulator";
    base_link = "base_link";
    last_link = "open_tip";
  }
  else
  {
    group_name = "panda_arm";
    base_link = "panda_link0";
    last_link = "panda_link8";
  }

  moveit::planning_interface::MoveGroupInterface move_group(group_name);
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
  planning_scene::PlanningScenePtr planning_scene = std::make_shared<planning_scene::PlanningScene>(kinematic_model);

  const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(group_name); //kinematic_model->getJointModelGroup(group_name);
  std::vector<std::string> joint_names = joint_model_group->getActiveJointModelNames();

  unsigned int dof = joint_names.size();
  Eigen::VectorXd lb(dof);
  Eigen::VectorXd ub(dof);

  for (unsigned int idx = 0; idx < dof; idx++)
  {
    const robot_model::VariableBounds& bounds = kinematic_model->getVariableBounds(joint_names.at(idx));  //bounds dei joints definito in urdf e file joints limit
    if (bounds.position_bounded_)
    {
      lb(idx) = bounds.min_position_;
      ub(idx) = bounds.max_position_;
    }
  }

  Eigen::VectorXd start_conf(dof);
  if(test == "sharework")
  {
    start_conf << 0.0,0.0,0.0,0.0,0.0,0.0; //sharework
  }
  else
  {
    start_conf << 0.0,0.0,0.0,-1.5,0.0,1.5,0.50; //panda
  }

  Eigen::VectorXd goal_conf(dof);
  if(test == "sharework")
  {
    goal_conf << -2.4568206461416158, -3.2888890061467357, 0.5022868201292561, -0.3550610439856255, 2.4569204968195355, 3.1415111410868053; //sharework
  }
  else
  {
    goal_conf <<  1.5, 0.5, 0.0, -1.0, 0.0, 2.0, 1.0; //panda
  }

  pathplan::NodePtr start_node = std::make_shared<pathplan::Node>(start_conf);
  //pathplan::NodePtr goal_node = std::make_shared<pathplan::Node>(goal_conf);

  pathplan::MetricsPtr metrics = std::make_shared<pathplan::Metrics>();
  pathplan::CollisionCheckerPtr checker = std::make_shared<pathplan::MoveitCollisionChecker>(planning_scene, group_name);

  // ////////////////////////////////////////////////////////////////////////PATH PLAN & VISUALIZATION/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  pathplan::Display disp = pathplan::Display(planning_scene,group_name,last_link);
  pathplan::PathPtr path = NULL;
  pathplan::Trajectory trajectory = pathplan::Trajectory(path,nh,planning_scene,group_name,base_link,last_link);

  for(unsigned int j=0; j<1;j++)
  {
    std::vector<pathplan::PathPtr> path_vector;

    for (unsigned int i =0; i<n_paths; i++)
    {
      pathplan::NodePtr goal_node = std::make_shared<pathplan::Node>(goal_conf);

      pathplan::PathPtr solution = trajectory.computeBiRRTPath(start_node, goal_node, lb, ub, metrics, checker, 1);
      path_vector.push_back(solution);
      ros::Duration(0.1).sleep();

      std::vector<double> marker_color;
      if(i==0) marker_color = {0.5,0.5,0.0,1.0};
      if(i==1) marker_color = {0.0f,0.0f,1.0,1.0};
      if(i==2) marker_color = {1.0,0.0f,0.0f,1.0};

      disp.displayPathAndWaypoints(solution,"pathplan",marker_color);
    }

    pathplan::PathPtr current_path = path_vector.front();
    int idx = 4; // current_path->getConnections().size()-1;

    std::vector<pathplan::ConnectionPtr> conn_v = current_path->getConnections();
    /*if(conn_v.size()-3 > 0)
    {
      conn_v.at(conn_v.size()-3)->setCost(std::numeric_limits<double>::infinity());
    }
    else
    {
      conn_v.at(conn_v.size()-2)->setCost(std::numeric_limits<double>::infinity());
    }*/

    conn_v.back()->setCost(std::numeric_limits<double>::infinity());
    //conn_v.at(idx)->setCost(std::numeric_limits<double>::infinity());
    current_path->setConnections(conn_v);

    /*conn_v = path_vector.at(1)->getConnections();
    conn_v.back()->setCost(std::numeric_limits<double>::infinity());
    path_vector.at(1)->setConnections(conn_v);

    conn_v = path_vector.at(2)->getConnections();
    conn_v.back()->setCost(std::numeric_limits<double>::infinity());
    path_vector.at(2)->setConnections(conn_v);*/

    std::vector<pathplan::PathPtr> other_paths = {path_vector.at(1),path_vector.at(2)};
    pathplan::SamplerPtr samp = std::make_shared<pathplan::InformedSampler>(start_conf, goal_conf, lb, ub);
    pathplan::BiRRTPtr solver = std::make_shared<pathplan::BiRRT>(metrics, checker, samp);
    solver->config(nh);

    //Eigen::VectorXd current_configuration = (current_path->getConnections().at(idx)->getChild()->getConfiguration() + current_path->getConnections().at(idx)->getParent()->getConfiguration())/2.0;
    pathplan::NodePtr node = current_path->getConnections().at(idx)->getParent();
    Eigen::VectorXd current_configuration = node->getConfiguration();


    pathplan::PathPtr new_path;
    pathplan::PathPtr subpath_from_path2;
    int connected2path_number;
    bool success;
    bool succ_node = 1;
    int informed = 2;

    // ///////////////////// Visualization current node /////////////////////
    std::vector<double> marker_color_sphere_actual = {1.0,0.0,1.0,1.0};
    disp.displayNode(std::make_shared<pathplan::Node>(current_configuration),"pathplan",marker_color_sphere_actual);
    // /////////////////////////////////////////////////////////////////////*/

    //disp.nextButton("Press \"next\" to start");

    pathplan::Replanner replanner = pathplan::Replanner(current_configuration, current_path, other_paths, solver, metrics, checker, lb, ub);

    for(unsigned int x=0; x<1;x++)
    {
      double time = 0.5;
      ros::WallTime tic = ros::WallTime::now();
      success =  replanner.informedOnlineReplanning(informed,succ_node,0.90*time);
      ros::WallTime toc = ros::WallTime::now();
      ROS_INFO_STREAM("DURATION: "<<(toc-tic).toSec()<<" success: "<<success<< " n sol: "<<replanner.getReplannedPathVector().size());
      if((toc-tic).toSec()>time) ROS_ERROR("TIME OUT");
      ros::Duration(0.01).sleep();
    }
    //success =  replanner.informedOnlineReplanning(informed,succ_node,disp);
    //success = replanner.pathSwitch(current_path,node, succ_node, new_path, subpath_from_path2, connected2path_number, disp);
    //success = replanner.connect2goal(current_path,node, new_path,disp);


    //if(success)ROS_INFO_STREAM("j: "<<j<<" success: "<<success<<" cost: "<<replanner.getReplannedPath()->cost());
    //else ROS_INFO_STREAM("j: "<<j<<" success: "<<success);

    /*//////////////////////////Visualization////////////////////////////////////
    std::vector<int> marker_id; marker_id.push_back(-101);
    std::vector<double> marker_color;
    marker_color = {1.0,1.0,0.0,1.0};

    std::vector<double> marker_scale(3,0.01);
    disp.changeConnectionSize(marker_scale);
    disp.displayPath(replanner.getReplannedPath(),"pathplan",marker_color);
    /////////////////////////////////////////////////////////////////////////*/

    /*trajectory.setPath(replanner.getReplannedPath());
    robot_trajectory::RobotTrajectoryPtr trj= trajectory.fromPath2Trj();
    moveit_msgs::RobotTrajectory trj_msg;
    trj->getRobotTrajectoryMsg(trj_msg);

    trajectory_processing::SplineInterpolator interpolator;
    interpolator.setTrajectory(trj_msg);
    interpolator.setSplineOrder(1);
    interpolator.resampleTrajectory(0.5, trj_msg);
    trajectory.displayTrj();*/

    ros::Duration(0.1).sleep();
  }
  return 0;
}

