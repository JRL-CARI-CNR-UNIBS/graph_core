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
double dt=0.01;
double dt_replan = 0.050;
double dt_replan_no_obstruction = 0.10;
double main_frequency = 1/dt;
double replan_frequency = 1/dt_replan;
double replan_offset=(dt_replan-dt)*2;
double t_replan=t+replan_offset;
trajectory_processing::SplineInterpolator interpolator;
trajectory_msgs::JointTrajectoryPoint pnt;
trajectory_msgs::JointTrajectoryPoint pnt_replan;
Eigen::VectorXd past_configuration_replan;
int n_conn = 0;
int n_conn_replan = 0;
bool first_replan = true;
bool path_obstructed;
bool computing_avoiding_path = false;
double checker_frequency = 30;

std::string test;

void replanning_fcn()
{
  ros::Rate lp(replan_frequency);

  bool replan = true;
  bool success = 0;
  int replanned_path_id = -1;
  int node_id = -2;
  Eigen::VectorXd goal;
  Eigen::VectorXd point2project(pnt_replan.positions.size());
  double past_abscissa;
  double abscissa;
  bool old_path_obstructed = path_obstructed;

  while (!stop)
  {
    ros::WallTime tic_tot=ros::WallTime::now();

    trj_mtx.lock();

    double time_informedOnlineRepl;
    std::string string_dt;
    if(path_obstructed)
    {
      replan_offset=(dt_replan-dt)*1.2;
      time_informedOnlineRepl = 0.90*dt_replan;
      string_dt = " dt ridotto";
      computing_avoiding_path = true;
    }
    else
    {
      replan_offset=(dt_replan_no_obstruction-dt)*1.2;
      time_informedOnlineRepl = 0.90*dt_replan_no_obstruction;
      string_dt = " dt rilassato";
    }

    if(old_path_obstructed != path_obstructed)
    {
      abscissa = 0;
      past_abscissa = 0;
    }
    old_path_obstructed = path_obstructed;

    t_replan=t+replan_offset;

    interpolator.interpolate(ros::Duration(t_replan),pnt_replan);
    for(unsigned int i=0; i<pnt_replan.positions.size();i++) point2project[i] = pnt_replan.positions.at(i);

    past_configuration_replan = configuration_replan;
    past_abscissa = abscissa;
    configuration_replan = replanner->getCurrentPath()->projectOnClosestConnectionKeepingCurvilinearAbscissa(point2project,past_configuration_replan,abscissa,past_abscissa,n_conn_replan);

    trj_mtx.unlock();
    std::vector<double> marker_scale_sphere_actual(3,0.02);
    std::vector<double> marker_color_sphere_actual = {0.0,0.0,0.0,1.0};
    display_mtx.lock();
    disp->changeNodeSize(marker_scale_sphere_actual);
    disp->displayNode(std::make_shared<pathplan::Node>(configuration_replan),node_id,"pathplan",marker_color_sphere_actual);
    disp->defaultNodeSize();
    display_mtx.unlock();

    marker_color_sphere_actual = {0.5,0.5,0.5,1.0};
    display_mtx.lock();
    disp->changeNodeSize(marker_scale_sphere_actual);
    disp->displayNode(std::make_shared<pathplan::Node>(point2project),node_id-8,"pathplan",marker_color_sphere_actual);
    disp->defaultNodeSize();
    display_mtx.unlock();
    trj_mtx.lock();
    /*if(replanner->getCurrentPath()->curvilinearAbscissaOfPointGivenConnection(configuration_replan,n_conn_replan)<=replanner->getCurrentPath()->curvilinearAbscissaOfPointGivenConnection(current_configuration,n_conn) & current_configuration != replanner->getCurrentPath()->getConnections().at(0)->getParent()->getConfiguration())
    {
      ROS_INFO_STREAM("shifting the replan config");
      do
      {
        t_replan+=replan_offset;

        interpolator.interpolate(ros::Duration(t_replan),pnt_replan);
        for(unsigned int i=0; i<pnt_replan.positions.size();i++) point2project[i] = pnt_replan.positions.at(i);

        past_configuration_replan = configuration_replan;
        past_abscissa = abscissa;
        configuration_replan = replanner->getCurrentPath()->projectOnClosestConnectionKeepingCurvilinearAbscissa(point2project,past_configuration_replan,abscissa,past_abscissa,n_conn_replan);
      }
      while(replanner->getCurrentPath()->curvilinearAbscissaOfPointGivenConnection(configuration_replan,n_conn_replan)<=replanner->getCurrentPath()->curvilinearAbscissaOfPointGivenConnection(current_configuration,n_conn));
    }*/

    goal = replanner->getCurrentPath()->getConnections().back()->getChild()->getConfiguration();
    replan = ((current_configuration-goal).norm()>1e-03 && (configuration_replan-goal).norm()>1e-03);
    if(replan) replanner->setCurrentConf(configuration_replan);
    //else stop = true;
    trj_mtx.unlock();

    /*std::vector<double> marker_scale_sphere_actual(3,0.02);
    std::vector<double> marker_color_sphere_actual = {0.0,0.0,0.0,1.0};
    display_mtx.lock();
    disp->changeNodeSize(marker_scale_sphere_actual);
    disp->displayNode(std::make_shared<pathplan::Node>(replanner->getCurrentConf()),node_id,"pathplan",marker_color_sphere_actual);
    disp->defaultNodeSize();
    display_mtx.unlock();

    marker_color_sphere_actual = {0.5,0.5,0.5,1.0};
    display_mtx.lock();
    disp->changeNodeSize(marker_scale_sphere_actual);
    disp->displayNode(std::make_shared<pathplan::Node>(point2project),node_id-8,"pathplan",marker_color_sphere_actual);
    disp->defaultNodeSize();
    display_mtx.unlock();*/

    if(replan)
    {
      planning_mtx.lock();
      ros::WallTime tic_rep=ros::WallTime::now();
      success =  replanner->informedOnlineReplanning(2,1,time_informedOnlineRepl);
      ros::WallTime toc_rep=ros::WallTime::now();
      planning_mtx.unlock();

      if((toc_rep-tic_rep).toSec()>=time_informedOnlineRepl/0.9) ROS_WARN("informed duration: %f",(toc_rep-tic_rep).toSec());

      ROS_INFO_STREAM("success: "<<success<< string_dt<<": "<<(toc_rep-tic_rep).toSec());
      ros::WallTime tic_trj;
      ros::WallTime toc_trj;

      if(success && !stop)
      {
        if(first_replan)
        {
          first_replan = false;
          replanner->addOtherPath(replanner->getCurrentPath());
        }

        checker_mtx.lock();
        trj_mtx.lock();
        replanner->startReplannedPathFromNewCurrentConf(current_configuration);
        replanner->simplifyReplannedPath(0.05);
        replanner->setCurrentPath(replanner->getReplannedPath());

        std::vector<double> marker_scale(3,0.01);
        std::vector<double> marker_color = {1.0,1.0,0.0,1.0};
        display_mtx.lock();
        disp->changeConnectionSize(marker_scale);
        disp->displayPathAndWaypoints(replanner->getReplannedPath(),replanned_path_id,-12,"pathplan",marker_color);
        disp->defaultConnectionSize();
        display_mtx.unlock();

        path_obstructed = false;
        computing_avoiding_path = false;
        n_conn = 0;
        n_conn_replan = 0;
        abscissa = 0;
        past_abscissa = 0;
        trj_mtx.unlock();
        checker_mtx.unlock();

        trj_mtx.lock();
        tic_trj=ros::WallTime::now();
        moveit_msgs::RobotTrajectory tmp_trj_msg;
        trajectory->setPath(replanner->getCurrentPath());
        robot_trajectory::RobotTrajectoryPtr trj= trajectory->fromPath2Trj(pnt);
        trj->getRobotTrajectoryMsg(tmp_trj_msg);
        toc_trj=ros::WallTime::now();

        interpolator.setTrajectory(tmp_trj_msg);
        interpolator.setSplineOrder(1);

        t=0;
        t_replan=0;

        trj_mtx.unlock();
      }
      ros::WallTime toc_tot=ros::WallTime::now();
      double duration = (toc_tot-tic_tot).toSec();

      if(duration>(time_informedOnlineRepl/0.9))
      {
        ROS_WARN("CYCLE TIME SUPERATO: duration-> %f",duration);
        ROS_WARN("rep-> %f",(toc_rep-tic_rep).toSec());
        if(success) ROS_WARN("trj-> %f",(toc_trj-tic_trj).toSec());
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

  ros::Rate lp(checker_frequency);
  bool object_spawned = false;
  bool second_object_spawned = false;
  bool third_object_spawned = false;
  bool fourth_object_spawned = false;
  while (!stop)
  {
    // ////////////////////////////////////////////SPAWNING THE OBJECT/////////////////////////////////////////////

    ros::ServiceClient add_obj;

    if(t>=0.40 && !fourth_object_spawned)
    {
      fourth_object_spawned = true;
      object_spawned = false;
    }

    if(t>=0.30 && !third_object_spawned)
    {
      third_object_spawned = true;
      object_spawned = false;
    }

    if(t>=0.20 && !second_object_spawned)
    {
      second_object_spawned = true;
      object_spawned = false;
    }

    if(!object_spawned && t>=0.10)
    {
      object_spawned = true;
      ROS_WARN("OGGETTO SPOWNATO");

      add_obj=nh.serviceClient<object_loader_msgs::addObjects>("add_object_to_scene");

      if (!add_obj.waitForExistence(ros::Duration(10)))
      {
        ROS_FATAL("srv not found");
      }

      object_loader_msgs::addObjects srv;
      object_loader_msgs::object obj;
      obj.object_type="scatola";

      int obj_conn_pos;
      if(!second_object_spawned) obj_conn_pos = replanner->getCurrentPath()->getConnections().size()-2;
      else obj_conn_pos = replanner->getCurrentPath()->getConnections().size()-1;
      pathplan::ConnectionPtr obj_conn = replanner->getCurrentPath()->getConnections().at(obj_conn_pos);
      pathplan::NodePtr obj_parent = obj_conn->getParent();
      pathplan::NodePtr obj_child = obj_conn->getChild();
      Eigen::VectorXd obj_pos = (obj_child->getConfiguration()+obj_parent->getConfiguration())/2;
      //Eigen::VectorXd obj_pos = obj_parent->getConfiguration();

      moveit::core::RobotState obj_pos_state = trajectory->fromWaypoints2State(obj_pos);
      if(test == "panda") tf::poseEigenToMsg(obj_pos_state.getGlobalLinkTransform("panda_link8"),obj.pose.pose);
      if(test == "cartesian") tf::poseEigenToMsg(obj_pos_state.getGlobalLinkTransform("end_effector"),obj.pose.pose);
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
    }

    // ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    if (!ps_client.call(ps_srv))
    {
      ROS_ERROR("call to srv not ok");
      continue;
    }

    planning_mtx.lock();
    if (!planning_scn->setPlanningSceneMsg(ps_srv.response.scene))
    {
      ROS_ERROR("unable to update planning scene");
    }
    std::vector<moveit_msgs::CollisionObject> collision_objs;
    planning_scn->getCollisionObjectMsgs(collision_objs);

    checker_mtx.lock();
    replanner->checkPathValidity();
    trj_mtx.lock();
    if(!computing_avoiding_path)
    {
      path_obstructed = !(replanner->getCurrentPath()->isValidFromConf(current_configuration));
      if(path_obstructed)
      {
        double costo = replanner->getCurrentPath()->cost();
        ROS_WARN("COST: %f",costo);
        /*if(costo != std::numeric_limits<double>::infinity())
        {
          while(true)
          {ros::Duration(1).sleep();}
        }*/
      }
    }
    trj_mtx.unlock();
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

  ros::ServiceClient ps_client=nh.serviceClient<moveit_msgs::GetPlanningScene>("/get_planning_scene");

  if (!ps_client.waitForExistence(ros::Duration(10)))
  {
    ROS_ERROR("unable to connect to /get_planning_scene");
  }

  ros::Duration(5).sleep();

  std::string group_name;
  std::string base_link;
  std::string last_link;

  if (!nh.getParam("group_name",test))
  {
    ROS_INFO("group_name not set, use panda");
    test="panda";
  }

  if(test == "sharework")
  {
    group_name = "manipulator";
    base_link = "base_link";
    last_link = "open_tip";
  }
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

  // UPDATING PLANNING SCENE
  ps_client=nh.serviceClient<moveit_msgs::GetPlanningScene>("/get_planning_scene");

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

  if (!planning_scn->setPlanningSceneMsg(ps_srv.response.scene))
  {
    ROS_ERROR("unable to update planning scene");
    return 1;
  }

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
    start_conf << 0.0,0.0,0.0,0.0,0.0,0.0;
  }
  if(test == "panda")
  {
    start_conf << 0.0,0.0,0.0,-1.5,0.0,1.5,-1.0;
  }
  if(test == "cartesian")
  {
    start_conf << 0.0,0.0,0.0;
  }

  Eigen::VectorXd goal_conf(dof);
  if(test == "sharework")
  {
    goal_conf << -2.4568206461416158, -3.2888890061467357, 0.5022868201292561, -0.3550610439856255, 2.4569204968195355, 3.1415111410868053;
  }
  if(test == "panda")
  {
    goal_conf << 1.5,0.5,0.0,-1.0,0.0,2.0,-1.0;
  }
  if(test == "cartesian")
  {
    goal_conf << 0.8,0.8,0.8;
  }

  pathplan::NodePtr start_node = std::make_shared<pathplan::Node>(start_conf);

  pathplan::MetricsPtr metrics = std::make_shared<pathplan::Metrics>();
  pathplan::CollisionCheckerPtr checker = std::make_shared<pathplan::MoveitCollisionChecker>(planning_scn, group_name);

  // ///////////////////////////PATH PLAN & VISUALIZATION//////////////////////////////////////////////////////////////////////////

  disp = std::make_shared<pathplan::Display>(planning_scn,group_name,last_link);
  pathplan::PathPtr path = NULL;
  trajectory = std::make_shared<pathplan::Trajectory>(path,nh,planning_scn,group_name,base_link,last_link);

  int current_node_id = -3;
  int trj_node_id = -4;

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

  path_obstructed = false;

  pathplan::SamplerPtr samp = std::make_shared<pathplan::InformedSampler>(start_conf, goal_conf, lb, ub);
  pathplan::BiRRTPtr solver = std::make_shared<pathplan::BiRRT>(metrics, checker, samp);
  solver->config(nh);

  trajectory->setPath(current_path);
  robot_trajectory::RobotTrajectoryPtr trj= trajectory->fromPath2Trj();
  moveit_msgs::RobotTrajectory tmp_trj_msg;
  trj->getRobotTrajectoryMsg(tmp_trj_msg);

  interpolator.setTrajectory(tmp_trj_msg);
  interpolator.setSplineOrder(1);

  Eigen::VectorXd point2project(dof);
  interpolator.interpolate(ros::Duration(t_replan),pnt_replan);
  for(unsigned int i=0; i<pnt_replan.positions.size();i++) point2project[i] = pnt_replan.positions.at(i);
  configuration_replan = current_path->projectOnClosestConnection(point2project);
  past_configuration_replan = configuration_replan;
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

  stop=false;
  std::thread replanning_thread=std::thread(&replanning_fcn);
  std::thread col_check_thread=std::thread(&collision_check_fcn);

  ros::Rate lp(main_frequency);
  while (ros::ok() && !stop)
  {
    trj_mtx.lock();

    t+=dt;

    interpolator.interpolate(ros::Duration(t),pnt);
    for(unsigned int i=0; i<pnt.positions.size();i++) point2project[i] = pnt.positions.at(i);
    past_current_configuration = current_configuration;
    current_configuration = replanner->getCurrentPath()->projectOnClosestConnectionKeepingPastPrj(point2project,past_current_configuration,n_conn);

    trj_mtx.unlock();

    //if((current_configuration-goal_conf).norm()<1e-3 || (configuration_replan-goal_conf).norm()<1e-3) stop = true;
    if((current_configuration-goal_conf).norm()<1e-3) stop = true;
    else
    {
      std::vector<double> marker_scale_sphere_actual(3,0.02);
      std::vector<double> marker_color_sphere_actual = {1.0,0.0,1.0,1.0};

      display_mtx.lock();
      disp->changeNodeSize(marker_scale_sphere_actual);
      disp->displayNode(std::make_shared<pathplan::Node>(current_configuration),current_node_id,"pathplan",marker_color_sphere_actual);

      marker_color_sphere_actual = {0.0,1.0,0.0,1.0};
      disp->displayNode(std::make_shared<pathplan::Node>(point2project),trj_node_id,"pathplan",marker_color_sphere_actual);
      disp->defaultNodeSize();
      display_mtx.unlock();

      joint_state.position = pnt.positions;
      joint_state.velocity = pnt.velocities;
      joint_state.name = joint_names;
      joint_state.header.frame_id = kinematic_model->getModelFrame();

      target_pub.publish(joint_state);
    }
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

