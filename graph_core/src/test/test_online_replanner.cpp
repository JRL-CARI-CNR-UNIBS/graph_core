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
#include <graph_core/replanner.h>
#include <moveit_planning_helper/spline_interpolator.h>
#include <object_loader_msgs/addObjects.h>
#include <thread>
#include <mutex>

volatile bool stop=false;

pathplan::ReplannerPtr replanner;
Eigen::VectorXd current_configuration;
Eigen::VectorXd configuration_replan;
planning_scene::PlanningScenePtr planning_scn;
std::mutex planning_mtx;
std::mutex trj_mtx;
std::mutex checker_mtx;
std::mutex display_mtx;
pathplan::TrajectoryPtr trajectory;
pathplan::DisplayPtr disp;

moveit_msgs::RobotTrajectory trj_msg;
double t=0;
double dt=0.1;//0.01;
double dt_replan=dt;
double t_replan=t+dt_replan;
trajectory_processing::SplineInterpolator interpolator;
trajectory_msgs::JointTrajectoryPoint pnt;
int n_conn = 0;
int n_conn_replan = 0;

bool first_replan = true;

void replanning_fcn()
{
  ros::Rate lp(1/dt);
  bool replan = true;
  int replanned_path_id;
  int node_id;
  Eigen::VectorXd goal;
  while (!stop)
  {
    trj_mtx.lock();
    if(replanner->getCurrentPath()->curvilinearAbscissaOfPoint(configuration_replan)<replanner->getCurrentPath()->curvilinearAbscissaOfPoint(current_configuration)) ROS_ERROR("REPL: replan prima di current");

    goal = replanner->getCurrentPath()->getConnections().back()->getChild()->getConfiguration();
    replan = ((current_configuration-goal).norm()>1e-06 && (configuration_replan-goal).norm()>1e-06);
    if(replan) replanner->setCurrentConf(configuration_replan);
    //else stop = true;
    trj_mtx.unlock();

    if(replan)
    {
      planning_mtx.lock();
      bool success =  replanner->informedOnlineReplanning(2,1);
      planning_mtx.unlock();

      //ROS_INFO_STREAM("success: "<<success);
      if (success && !stop)
      {
        std::vector<double> marker_scale_sphere_actual(3,0.02);
        std::vector<double> marker_color_sphere_actual = {0.0,0.0,0.0,1.0};
        display_mtx.lock();
        disp->clearMarker(node_id);
        disp->changeNodeSize(marker_scale_sphere_actual);
        node_id = disp->displayNode(std::make_shared<pathplan::Node>(replanner->getCurrentConf()),"pathplan",marker_color_sphere_actual);
        disp->defaultNodeSize();
        display_mtx.unlock();

        if(first_replan)
        {
          first_replan = false;
          replanner->addOtherPath(replanner->getCurrentPath());
        }

        trj_mtx.lock();
        replanner->startReplannedPathFromNewCurrentConf(current_configuration);
        t=0;
        t_replan = 0;

        ROS_INFO("---------------------------------------");

        checker_mtx.lock();
        replanner->setCurrentPath(replanner->getReplannedPath());
        n_conn = 0;
        n_conn_replan = 0;
        checker_mtx.unlock();

        moveit_msgs::RobotTrajectory tmp_trj_msg;
        trajectory->setPath(replanner->getCurrentPath());
        robot_trajectory::RobotTrajectoryPtr trj= trajectory->fromPath2Trj(pnt);
        trj->getRobotTrajectoryMsg(tmp_trj_msg);

        interpolator.setTrajectory(tmp_trj_msg);
        interpolator.setSplineOrder(1);
        trj_mtx.unlock();

        std::vector<double> marker_scale(3,0.01);
        std::vector<double> marker_color = {1.0,1.0,0.0,1.0};

        display_mtx.lock();
        disp->clearMarker(replanned_path_id);
        disp->changeConnectionSize(marker_scale);
        replanned_path_id = disp->displayPath(replanner->getReplannedPath(),"pathplan",marker_color);
        disp->defaultConnectionSize();
        display_mtx.unlock();
      }
    }

    lp.sleep();
  }
}

void collision_check_fcn()
{
   ros::NodeHandle nh;
  ros::ServiceClient ps_client=nh.serviceClient<moveit_msgs::GetPlanningScene>("/get_planning_scene");

  if (!ps_client.waitForExistence(ros::Duration(10)))
  {
    ROS_ERROR("unable to connect to /get_planning_scene");
  }

  moveit_msgs::GetPlanningScene ps_srv;

  ros::Rate lp(30);
  bool object_spawned = false;
  bool second_object_spawned = false;
  while (!stop)
  {
    // ////////////////////////////////////////////SPAWNING THE OBJECT/////////////////////////////////////////////

    ros::ServiceClient add_obj;
    if(t>=0.15 && !second_object_spawned)
    {
      second_object_spawned = true;
      object_spawned = false;
    }

    if(!object_spawned && t>=0.05)
    {
      object_spawned = true;

      add_obj=nh.serviceClient<object_loader_msgs::addObjects>("add_object_to_scene");

      if (!add_obj.waitForExistence(ros::Duration(10)))
      {
        ROS_FATAL("srv not found");
      }

      object_loader_msgs::addObjects srv;
      object_loader_msgs::object obj;
      obj.object_type="scatola";

      int obj_conn_pos = replanner->getCurrentPath()->getConnections().size()-2;
      pathplan::ConnectionPtr obj_conn = replanner->getCurrentPath()->getConnections().at(obj_conn_pos);
      pathplan::NodePtr obj_parent = obj_conn->getParent();
      pathplan::NodePtr obj_child = obj_conn->getChild();
      Eigen::VectorXd obj_pos = (obj_child->getConfiguration()+obj_parent->getConfiguration())/2;
      //Eigen::VectorXd obj_pos = obj_parent->getConfiguration();

      moveit::core::RobotState obj_pos_state = trajectory->fromWaypoints2State(obj_pos);
      tf::poseEigenToMsg(obj_pos_state.getGlobalLinkTransform("panda_link8"),obj.pose.pose);
      obj.pose.header.frame_id="world";

      srv.request.objects.push_back(obj);
      if (!add_obj.call(srv))
      {
        ROS_ERROR("call to srv not ok");
      }
      if (!srv.response.success)
      {
        ROS_ERROR("srv error");
      }

      if (!planning_scn->setPlanningSceneMsg(ps_srv.response.scene))
      {
        ROS_ERROR("unable to update planning scene");
      }
    }

    // ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    if (!ps_client.call(ps_srv))
    {
      ROS_ERROR("call to srv not ok");
    }

    planning_mtx.lock();
    if (!planning_scn->setPlanningSceneMsg(ps_srv.response.scene))
    {
      ROS_ERROR("unable to update planning scene");
    }
    checker_mtx.lock();
    replanner->checkPathValidity();
    checker_mtx.unlock();
    planning_mtx.unlock();
    lp.sleep();
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "node_replanner");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::NodeHandle nh;
  ros::Publisher target_pub=nh.advertise<sensor_msgs::JointState>("/joint_target",1);
  ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("/marker_visualization_topic", 1);
  ros::Publisher display_publisher = nh.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);

  std::string test = "panda";

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

  pathplan::MetricsPtr metrics = std::make_shared<pathplan::Metrics>();
  pathplan::CollisionCheckerPtr checker = std::make_shared<pathplan::MoveitCollisionChecker>(planning_scn, group_name);

  // ///////////////////////////PATH PLAN & VISUALIZATION//////////////////////////////////////////////////////////////////////////

  disp = std::make_shared<pathplan::Display>(planning_scn,group_name,base_link,last_link);
  pathplan::PathPtr path = NULL;
  trajectory = std::make_shared<pathplan::Trajectory>(path,nh,planning_scn,group_name,base_link,last_link);

  int current_node_id;
  int trj_node_id;

  std::vector<pathplan::PathPtr> path_vector;

  for (unsigned int i =0; i<3; i++)
  {
    pathplan::NodePtr goal_node = std::make_shared<pathplan::Node>(goal_conf);

    pathplan::PathPtr solution = trajectory->computeBiRRTPath(start_node, goal_node, lb, ub, metrics, checker, 1);
    path_vector.push_back(solution);

    std::vector<double> marker_color;
    if(i==0) marker_color = {0.5,0.5,0.0,1.0};
    if(i==1) marker_color = {0.0f,0.0f,1.0,1.0};
    if(i==2) marker_color = {1.0,0.0f,0.0f,1.0};

    disp->displayPathAndWaypoints(solution,"pathplan",marker_color);
  }

  // ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  pathplan::PathPtr current_path = path_vector.front();
  std::vector<pathplan::PathPtr> other_paths = {current_path,path_vector.at(1),path_vector.at(2)};

  pathplan::SamplerPtr samp = std::make_shared<pathplan::InformedSampler>(start_conf, goal_conf, lb, ub);
  pathplan::BiRRTPtr solver = std::make_shared<pathplan::BiRRT>(metrics, checker, samp);
  solver->config(nh);

  trajectory->setPath(current_path);
  robot_trajectory::RobotTrajectoryPtr trj= trajectory->fromPath2Trj();
  moveit_msgs::RobotTrajectory tmp_trj_msg;
  trj->getRobotTrajectoryMsg(tmp_trj_msg);

  interpolator.setTrajectory(tmp_trj_msg);
  interpolator.setSplineOrder(1);

  trajectory_msgs::JointTrajectoryPoint pnt_replan;

  Eigen::VectorXd point2project(dof);
  interpolator.interpolate(ros::Duration(t_replan),pnt_replan);
  for(unsigned int i=0; i<pnt_replan.positions.size();i++) point2project[i] = pnt_replan.positions.at(i);
  configuration_replan = current_path->projectOnClosestConnection(point2project);
  Eigen::VectorXd past_configuration_replan = configuration_replan;
  n_conn_replan = 0;

  replanner = std::make_shared<pathplan::Replanner>(configuration_replan, current_path, other_paths, solver, metrics, checker, lb, ub);

  interpolator.interpolate(ros::Duration(t),pnt);
  current_configuration = start_conf;
  Eigen::VectorXd past_current_configuration = current_configuration;

  sensor_msgs::JointState joint_state;
  joint_state.position = pnt.positions;
  joint_state.velocity = pnt.velocities;
  joint_state.name = joint_names;
  joint_state.header.frame_id = kinematic_model->getModelFrame();

  target_pub.publish(joint_state);

  ROS_INFO_STREAM("t_REPL: "<<t_replan);
  ROS_INFO_STREAM("t: "<<t);

  stop=false;
  std::thread replanning_thread=std::thread(&replanning_fcn);
  std::thread col_check_thread=std::thread(&collision_check_fcn);

  ros::Rate lp(1/dt);
  while (ros::ok() && !stop)
  {
    trj_mtx.lock();

    t+=dt;
    t_replan=t+dt_replan;

    interpolator.interpolate(ros::Duration(t_replan),pnt_replan);
    for(unsigned int i=0; i<pnt_replan.positions.size();i++) point2project[i] = pnt_replan.positions.at(i);
    configuration_replan = replanner->getCurrentPath()->projectOnClosestConnection(point2project,past_configuration_replan,n_conn_replan);
    past_configuration_replan = configuration_replan;

    interpolator.interpolate(ros::Duration(t),pnt);
    for(unsigned int i=0; i<pnt.positions.size();i++) point2project[i] = pnt.positions.at(i);
    current_configuration = replanner->getCurrentPath()->projectOnClosestConnection(point2project,past_current_configuration,n_conn);
    past_current_configuration = current_configuration;

    ROS_INFO_STREAM("t_REPL: "<<t_replan);
    ROS_INFO_STREAM("t: "<<t);

    trj_mtx.unlock();

    if((current_configuration-goal_conf).norm()<1e-6) stop = true;
    else
    {
      std::vector<double> marker_scale_sphere_actual(3,0.02);
      std::vector<double> marker_color_sphere_actual = {1.0,0.0,1.0,1.0};

      display_mtx.lock();
      disp->clearMarker(current_node_id);
      disp->changeNodeSize(marker_scale_sphere_actual);
      current_node_id = disp->displayNode(std::make_shared<pathplan::Node>(current_configuration),"pathplan",marker_color_sphere_actual);

      disp->clearMarker(trj_node_id);
      marker_color_sphere_actual = {0.0,1.0,0.0,1.0};
      trj_node_id = disp->displayNode(std::make_shared<pathplan::Node>(point2project),"pathplan",marker_color_sphere_actual);
      disp->defaultNodeSize();
      display_mtx.unlock();

      joint_state.position = pnt.positions;
      joint_state.velocity = pnt.velocities;
      joint_state.name = joint_names;
      joint_state.header.frame_id = kinematic_model->getModelFrame();

      target_pub.publish(joint_state);
    }

    if(replanner->getCurrentPath()->curvilinearAbscissaOfPoint(configuration_replan)<replanner->getCurrentPath()->curvilinearAbscissaOfPoint(current_configuration)) ROS_ERROR("replan prima di current");

    lp.sleep();
  }

  ROS_ERROR("STOP");
  stop=true;
  if (replanning_thread.joinable())
    replanning_thread.join();
  if (col_check_thread.joinable())
    col_check_thread.join();

  return 0;
}

