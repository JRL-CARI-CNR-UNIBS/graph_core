#include "graph_replanning/replanner_manager.h"

namespace pathplan
{
ReplannerManager::ReplannerManager(PathPtr &current_path,
                                   std::vector<PathPtr> &other_paths,
                                   ros::NodeHandle &nh)
{
  current_path_ = current_path;
  other_paths_ = other_paths;
  nh_ = nh;

  subscribeTopicsAndServices();
  fromParam();
  attributeInitialization();
}


// /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void ReplannerManager::fromParam()
{
  if(!nh_.getParam("trj_execution_thread_frequency",trj_execution_thread_frequency_)) throw  std::invalid_argument("trj_execution_thread_frequency not set");
  if(!nh_.getParam("collision_checker_thread_frequency",collision_checker_thread_frequency_)) throw  std::invalid_argument("collision_checker_thread_frequency not set");
  if(!nh_.getParam("dt_replan_restricted",dt_replan_restricted_)) throw  std::invalid_argument("dt_replan_restricted not set");
  if(!nh_.getParam("dt_replan_relaxed",dt_replan_relaxed_)) throw  std::invalid_argument("dt_replan_relaxed not set");
  if(!nh_.getParam("group_name",group_name_)) throw  std::invalid_argument("group_name not set");
  if(!nh_.getParam("base_link",base_link_)) throw  std::invalid_argument("base_link not set");
  if(!nh_.getParam("last_link",last_link_)) throw  std::invalid_argument("last_link not set");
  if(!nh_.getParam("spawn_objs",spawn_objs_)) spawn_objs_ = false;
  if(!nh_.getParam("scaling",scaling_)) scaling_ = 1.0;
  if(!nh_.getParam("checker_resolution",checker_resol_)) checker_resol_ = 0.05;
  if(!nh_.getParam("display_timing_warning",display_timing_warning_)) display_timing_warning_ = false;
  if(!nh_.getParam("display_replanning_success",display_replanning_success_)) display_replanning_success_ = false;
}

void ReplannerManager::attributeInitialization()
{
  stop_ = false;
  replanning_ = false;
  replanning_thread_frequency_ = 1/dt_replan_restricted_;
  real_time_ = 0.0;
  t_ = 0.0;
  dt_ = 1/trj_execution_thread_frequency_;
  replan_offset_ = (dt_replan_restricted_-dt_)*K_OFFSET;
  t_replan_ = t_+replan_offset_;
  n_conn_ = 0;
  first_replan_ = true;
  path_obstructed_ = false;
  computing_avoiding_path_ = false;
  pos_closest_obs_from_goal_check_ = -1;
  pos_closest_obs_from_goal_repl_ = pos_closest_obs_from_goal_check_;

  moveit::planning_interface::MoveGroupInterface move_group(group_name_);
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
  planning_scn_ = std::make_shared<planning_scene::PlanningScene>(kinematic_model);
  planning_scn_replanning_ = std::make_shared<planning_scene::PlanningScene>(kinematic_model);

  moveit_msgs::GetPlanningScene ps_srv;
  if (!plannning_scene_client_.call(ps_srv))
  {
    ROS_ERROR("call to srv not ok");
  }

  if (!planning_scn_->setPlanningSceneMsg(ps_srv.response.scene))
  {
    ROS_ERROR("unable to update planning scene");
  }

  if (!planning_scn_replanning_->setPlanningSceneMsg(ps_srv.response.scene))
  {
    ROS_ERROR("unable to update planning scene");
  }

  const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(group_name_);
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

  pathplan::MetricsPtr metrics = std::make_shared<pathplan::Metrics>();
  checker_thread_cc_ = std::make_shared<pathplan::MoveitCollisionChecker>(planning_scn_, group_name_, checker_resol_);
  checker_ = std::make_shared<pathplan::MoveitCollisionChecker>(planning_scn_replanning_, group_name_, checker_resol_);

  current_path_->setChecker(checker_);  //To synchronize path checker with the related one to the planning scene that will be used by the replanner, which will be different from the planning scene used by the collision checking thread
  for(const PathPtr &path:other_paths_) path->setChecker(checker_);

  pathplan::SamplerPtr samp = std::make_shared<pathplan::InformedSampler>(current_path_->getWaypoints().front(), current_path_->getWaypoints().back(), lb, ub);
  pathplan::BiRRTPtr solver = std::make_shared<pathplan::BiRRT>(metrics, checker_, samp);
  solver->config(nh_);

  trajectory_ = std::make_shared<pathplan::Trajectory>(current_path_,nh_,planning_scn_replanning_,group_name_,base_link_,last_link_);
  robot_trajectory::RobotTrajectoryPtr trj= trajectory_->fromPath2Trj();
  moveit_msgs::RobotTrajectory tmp_trj_msg;
  trj->getRobotTrajectoryMsg(tmp_trj_msg);
  interpolator_.setTrajectory(tmp_trj_msg);
  interpolator_.setSplineOrder(1);

  Eigen::VectorXd point2project(dof);
  interpolator_.interpolate(ros::Duration(t_replan_),pnt_replan_);
  for(unsigned int i=0; i<pnt_replan_.positions.size();i++) point2project[i] = pnt_replan_.positions.at(i);
  configuration_replan_ = current_path_->projectOnClosestConnection(point2project);
  current_configuration_ = current_path_->getWaypoints().front();
  n_conn_ = 0;

  replanner_ = std::make_shared<pathplan::Replanner>(configuration_replan_, current_path_, other_paths_, solver, metrics, checker_, lb, ub);

  interpolator_.interpolate(ros::Duration(t_),pnt_);
  joint_state_.position = pnt_.positions;
  joint_state_.velocity = pnt_.velocities;
  joint_state_.name = joint_names;
  joint_state_.header.frame_id = kinematic_model->getModelFrame();
  joint_state_.header.stamp=ros::Time::now();
  //target_pub_.publish(joint_state_);
}

void ReplannerManager::subscribeTopicsAndServices()
{
  target_pub_= nh_.advertise<sensor_msgs::JointState>("/joint_target",1);
  plannning_scene_client_ = nh_.serviceClient<moveit_msgs::GetPlanningScene>("/get_planning_scene");

  if (!plannning_scene_client_.waitForExistence(ros::Duration(10)))
  {
    ROS_ERROR("unable to connect to /get_planning_scene");
  }

  time_pub_= nh_.advertise<std_msgs::Float64>("/time_topic",1);
  current_norm_pub_ = nh_.advertise<std_msgs::Float64>("/current_norm_topic", 1000);
  new_norm_pub_ = nh_.advertise<std_msgs::Float64>("/new_norm_topic", 1000);
  time_replanning_pub_ = nh_.advertise<std_msgs::Float64>("/time_replanning_topic", 1000);
  obs_current_norm_pub_ = nh_.advertise<std_msgs::Float64>("/obs_current_norm_topic", 1000);
  obs_new_norm_pub_ = nh_.advertise<std_msgs::Float64>("/obs_new_norm_topic", 1000);
  obs_time_replanning_pub_ = nh_.advertise<std_msgs::Float64>("/obs_time_replanning_topic", 1000);
  start_log_ = nh_.serviceClient<std_srvs::Empty> ("/start_log");
  stop_log_  = nh_.serviceClient<std_srvs::Empty> ("/stop_log");
  add_obj_ = nh_.serviceClient<object_loader_msgs::addObjects>("add_object_to_scene");
  remove_obj_ = nh_.serviceClient<object_loader_msgs::removeObjects>("/remove_object_from_scene");

  if (!add_obj_.waitForExistence(ros::Duration(10)))
  {
    ROS_ERROR("unable to connect to /add_object_to_scene");
  }
  if (!remove_obj_.waitForExistence(ros::Duration(10)))
  {
    ROS_ERROR("unable to connect to /remove_object_to_scene");
  }
}

void ReplannerManager::replanningThread()
{
  ros::Rate lp(replanning_thread_frequency_);

  bool replan = true;
  bool success = 0;
  Eigen::VectorXd goal = replanner_->getCurrentPath()->getConnections().back()->getChild()->getConfiguration();
  Eigen::VectorXd point2project(pnt_replan_.positions.size());
  Eigen::VectorXd past_configuration_replan = configuration_replan_;
  double past_abscissa;
  double abscissa;
  int n_conn_replan = 0;
  bool old_path_obstructed = path_obstructed_;

  while (!stop_)
  {
    ros::WallTime tic_tot=ros::WallTime::now();

    trj_mtx_.lock();
    t_replan_ = t_+replan_offset_;

    interpolator_.interpolate(ros::Duration(t_replan_),pnt_replan_);
    for(unsigned int i=0; i<pnt_replan_.positions.size();i++) point2project[i] = pnt_replan_.positions.at(i);

    past_configuration_replan = configuration_replan_;
    past_abscissa = abscissa;
    configuration_replan_ = replanner_->getCurrentPath()->projectOnClosestConnectionKeepingCurvilinearAbscissa(point2project,past_configuration_replan,abscissa,past_abscissa,n_conn_replan);

    replan = ((current_configuration_-goal).norm()>1e-03 && (configuration_replan_-goal).norm()>1e-03);

    trj_mtx_.unlock();

    if(replan)
    {
      double time_informedOnlineRepl;
      std::string string_dt;

      checker_mtx_.lock();
      scene_mtx_.lock();
      moveit_msgs::PlanningScene scn;
      planning_scn_->getPlanningSceneMsg(scn);
      planning_scn_replanning_->setPlanningSceneMsg(scn);
      scene_mtx_.unlock();

      pathplan::PathPtr current_path_copy = current_path_->clone();
      std::vector<pathplan::PathPtr> other_paths_copy;
      for(const pathplan::PathPtr path:other_paths_) other_paths_copy.push_back(path->clone());

      pos_closest_obs_from_goal_repl_ = pos_closest_obs_from_goal_check_;

      replanner_mtx_.lock();
      if(current_path_copy->findConnection(configuration_replan_) == NULL)
      {
        trj_mtx_.lock();
        configuration_replan_ = current_configuration_;  //può essere che non venga trovata la repl conf e rimanga ferma per qualche ciclo, intanto la current conf va avanti, la supera e il replanned path partirà da questa e non includera la repl conf
        trj_mtx_.unlock();
      }
      replanner_->setCurrentConf(configuration_replan_);
      replanner_->setCurrentPath(current_path_copy);
      replanner_->setOtherPaths(other_paths_copy);
      replanner_mtx_.unlock();

      if(path_obstructed_)
      {
        replan_offset_ = (dt_replan_restricted_-dt_)*K_OFFSET;
        time_informedOnlineRepl = 0.90*dt_replan_restricted_;
        string_dt = " reduced dt";
        computing_avoiding_path_ = true;
      }
      else
      {
        replan_offset_ = (dt_replan_relaxed_-dt_)*K_OFFSET;
        time_informedOnlineRepl = 0.90*dt_replan_relaxed_;
        string_dt = " relaxed dt";
      }

      if(old_path_obstructed != path_obstructed_)
      {
        abscissa = 0;
        past_abscissa = 0;
      }
      old_path_obstructed = path_obstructed_;
      checker_mtx_.unlock();

      planning_mtx_.lock();
      ros::WallTime tic_rep=ros::WallTime::now();
      replanning_ = true;
      success =  replanner_->informedOnlineReplanning(2,1,time_informedOnlineRepl);
      replanning_ = false;
      ros::WallTime toc_rep=ros::WallTime::now();
      planning_mtx_.unlock();

      if((toc_rep-tic_rep).toSec()>=time_informedOnlineRepl/0.9 && display_timing_warning_) ROS_WARN("replanning duration: %f",(toc_rep-tic_rep).toSec());
      if(display_replanning_success_) ROS_INFO_STREAM("success: "<< success << string_dt << ": " << (toc_rep-tic_rep).toSec());

      ros::WallTime tic_trj;
      ros::WallTime toc_trj;

      if(success && !stop_)
      {
        if(first_replan_)
        {
          first_replan_ = false;

          replanner_mtx_.lock();
          replanner_->addOtherPath(replanner_->getCurrentPath());
          replanner_mtx_.unlock();

          checker_mtx_.lock();
          other_paths_.push_back(current_path_);
          checker_mtx_.unlock();
        }

        std_msgs::Float64 current_norm;
        std_msgs::Float64 new_norm;
        std_msgs::Float64 time_replanning;

        checker_mtx_.lock();
        replanner_mtx_.lock();
        trj_mtx_.lock();
        replanner_->startReplannedPathFromNewCurrentConf(current_configuration_);
        replanner_->simplifyReplannedPath(0.1);  //usa var locale e poi assegna a curr path

        current_norm.data = replanner_->getCurrentPath()->getNormFromConf(current_configuration_);
        new_norm.data = replanner_->getReplannedPath()->cost();
        time_replanning.data = (toc_rep-tic_rep).toSec();

        replanner_->setCurrentPath(replanner_->getReplannedPath());
        current_path_ = replanner_->getReplannedPath();

        tic_trj=ros::WallTime::now();
        moveit_msgs::RobotTrajectory tmp_trj_msg;
        trajectory_->setPath(current_path_);
        robot_trajectory::RobotTrajectoryPtr trj= trajectory_->fromPath2Trj(pnt_);
        trj->getRobotTrajectoryMsg(tmp_trj_msg);
        toc_trj=ros::WallTime::now();

        interpolator_.setTrajectory(tmp_trj_msg);
        interpolator_.setSplineOrder(1);

        t_=0;
        t_replan_=0;
        n_conn_ = 0;
        n_conn_replan = 0;
        abscissa = 0;
        past_abscissa = 0;
        trj_mtx_.unlock();
        replanner_mtx_.unlock();

        if(computing_avoiding_path_)
        {
          obs_current_norm_pub_.publish(current_norm);
          obs_new_norm_pub_.publish(new_norm);
          obs_time_replanning_pub_.publish(time_replanning);
        }
        else
        {
          current_norm_pub_.publish(current_norm);
          new_norm_pub_.publish(new_norm);
          time_replanning_pub_.publish(time_replanning);
        }

        path_obstructed_ = false;
        computing_avoiding_path_ = false;
        checker_mtx_.unlock();
      }
      ros::WallTime toc_tot=ros::WallTime::now();
      double duration = (toc_tot-tic_tot).toSec();

      if(duration>(time_informedOnlineRepl/0.9) && display_timing_warning_)
      {
        ROS_WARN("Replanning thread time expired: duration-> %f",duration);
        ROS_WARN("replanning time-> %f",(toc_rep-tic_rep).toSec());
        if(success) ROS_WARN("trj computation time-> %f",(toc_trj-tic_trj).toSec());
      }
    }
    lp.sleep();
  }
}

void ReplannerManager::collisionCheckThread()
{
  ros::Rate lp(collision_checker_thread_frequency_);

  moveit_msgs::GetPlanningScene ps_srv;

  while (!stop_)
  {
    ros::WallTime tic_tot = ros::WallTime::now();

    scene_mtx_.lock();
    ros::WallTime tic_pln_call = ros::WallTime::now();
    if (!plannning_scene_client_.call(ps_srv))
    {
      ROS_ERROR("call to srv not ok");
    }
    ros::WallTime toc_pln_call = ros::WallTime::now();

    if (!planning_scn_->setPlanningSceneMsg(ps_srv.response.scene))
    {
      ROS_ERROR("unable to update planning scene");
    }
    scene_mtx_.unlock();

    ros::WallTime tic_mtx = ros::WallTime::now();
    checker_mtx_.lock();
    ros::WallTime toc_mtx = ros::WallTime::now();
    ros::WallTime tic_check = ros::WallTime::now();
    //current_path_->isValid(checker_thread_cc_);
    for(const pathplan::PathPtr& path: other_paths_) path->isValid(checker_thread_cc_);

    trj_mtx_.lock();
    pos_closest_obs_from_goal_check_ = -1;
    bool path_obstructed = !(current_path_->isValidFromConf(current_configuration_,pos_closest_obs_from_goal_check_,checker_thread_cc_));
    trj_mtx_.unlock();
    ros::WallTime toc_check = ros::WallTime::now();
    if(!computing_avoiding_path_) path_obstructed_ = path_obstructed;

    if(replanning_ && path_obstructed)
    {
      bool replan_relaxed = (replan_offset_ == (dt_replan_relaxed_-dt_)*K_OFFSET);
      if(replan_relaxed || (!replan_relaxed && (pos_closest_obs_from_goal_check_>pos_closest_obs_from_goal_repl_)))  //replanning relaxed or not relaxed but a new closer obstacle is spawned..
      {
        replanner_->setEmergencyStop();
        ROS_WARN("EMERGENCY STOP!");
      }
    }
    checker_mtx_.unlock();

    ros::WallTime toc_tot=ros::WallTime::now();
    double duration = (toc_tot-tic_tot).toSec();

    if(duration>(1/collision_checker_thread_frequency_) && display_timing_warning_)
    {
      ROS_WARN("Collision checking thread time expired: duration-> %f",duration);
      ROS_WARN("t mtx %f, t scn call %f, t check %f",(toc_mtx-tic_mtx).toSec(),(toc_pln_call-tic_pln_call).toSec(),(toc_check-tic_check).toSec());
    }

    lp.sleep();
  }
}

bool ReplannerManager::trajectoryExecutionThread()
{
  std_srvs::Empty srv_log;
  start_log_.call(srv_log);

  target_pub_.publish(joint_state_);

  Eigen::VectorXd past_current_configuration = current_configuration_;
  Eigen::VectorXd goal_conf = current_path_->getWaypoints().back();

  ROS_WARN("Launching threads..");
  std::thread display_thread = std::thread(&ReplannerManager::displayThread, this);  //it must be the first one launched, otherwise the first paths will be not displayed in time
  std::thread spawn_obj_thread;
  if(spawn_objs_) spawn_obj_thread = std::thread(&ReplannerManager::spawnObjects, this);
  ros::Duration(1).sleep();
  std::thread replanning_thread = std::thread(&ReplannerManager::replanningThread, this);
  std::thread col_check_thread = std::thread(&ReplannerManager::collisionCheckThread, this);

  ros::Rate lp(trj_execution_thread_frequency_);

  while (ros::ok() && !stop_)
  {
    ros::WallTime tic_tot = ros::WallTime::now();

    replanner_mtx_.lock();
    trj_mtx_.lock();
    real_time_ += dt_;
    t_+= scaling_*dt_;

    interpolator_.interpolate(ros::Duration(t_),pnt_);
    Eigen::VectorXd point2project(pnt_.positions.size());
    for(unsigned int i=0; i<pnt_.positions.size();i++) point2project[i] = pnt_.positions.at(i);
    past_current_configuration = current_configuration_;
    current_configuration_ = replanner_->getCurrentPath()->projectOnClosestConnectionKeepingPastPrj(point2project,past_current_configuration,n_conn_);

    trj_mtx_.unlock();
    replanner_mtx_.unlock();

    joint_state_.position = pnt_.positions;
    joint_state_.velocity = pnt_.velocities;
    joint_state_.header.stamp=ros::Time::now();
    target_pub_.publish(joint_state_);

    std_msgs::Float64 time_msg;
    time_msg.data = real_time_;
    time_pub_.publish(time_msg);

    if((current_configuration_-goal_conf).norm()<1e-3) stop_ = true;

    ros::WallTime toc_tot = ros::WallTime::now();
    double duration = (toc_tot-tic_tot).toSec();

    if(duration>(1/trj_execution_thread_frequency_) && display_timing_warning_)
    {
      ROS_WARN("Trj execution thread time expired: duration-> %f",duration);
    }

    lp.sleep();
  }

  ROS_ERROR("STOP");
  stop_ = true;

  if(replanning_thread.joinable()) replanning_thread.join();
  if(col_check_thread.joinable()) col_check_thread.join();
  if(display_thread.joinable()) display_thread.join();
  if(spawn_objs_)
  {
    if(spawn_obj_thread.joinable()) spawn_obj_thread.join();
  }

  // BINARY LOGGER SALVA FINO A I-1 ESIMO DATO PUBBLICATO, QUESTO AIUTA A SALVARLI TUTTI
  std_msgs::Float64 fake_data;
  sensor_msgs::JointState joint_fake;
  joint_fake.position = pnt_.positions;
  joint_fake.velocity = pnt_.velocities;
  joint_fake.name = joint_state_.name;
  joint_fake.header.frame_id = joint_state_.header.frame_id;
  joint_fake.header.stamp=ros::Time::now();
  fake_data.data = 0.0;
  obs_current_norm_pub_.publish(fake_data);
  obs_new_norm_pub_.publish(fake_data);
  obs_time_replanning_pub_.publish(fake_data);
  current_norm_pub_.publish(fake_data);
  new_norm_pub_.publish(fake_data);
  time_replanning_pub_.publish(fake_data);
  time_pub_.publish(fake_data);
  target_pub_.publish(joint_fake);

  stop_log_.call(srv_log);

  return 1;
}

void ReplannerManager::displayThread()
{
  //scene_mtx_.lock();
  pathplan::DisplayPtr disp = std::make_shared<pathplan::Display>(planning_scn_,group_name_,last_link_);
  //scene_mtx_.unlock();
  ros::Duration(0.5).sleep();

  std::vector<double> marker_color;
  std::vector<double> marker_scale;
  std::vector<double> marker_scale_sphere(3,0.02);

  disp->clearMarkers();

  double display_thread_frequency = 0.75*trj_execution_thread_frequency_;
  ros::Rate lp(display_thread_frequency);

  while(!stop_)
  {
    //checker_mtx_.lock();
    replanner_mtx_.lock();
    trj_mtx_.lock();
    pathplan::PathPtr current_path = current_path_->clone();
    std::vector<pathplan::PathPtr> other_paths;
    for(const pathplan::PathPtr path:other_paths_) other_paths.push_back(path->clone());
    Eigen::VectorXd current_configuration = current_configuration_;
    Eigen::VectorXd configuration_replan = configuration_replan_;
    trajectory_msgs::JointTrajectoryPoint pnt = pnt_;
    trajectory_msgs::JointTrajectoryPoint pnt_replan = pnt_replan_;
    trj_mtx_.unlock();
    replanner_mtx_.unlock();
    //checker_mtx_.unlock();

    int path_id = 10;
    int node_id = 1000;
    int wp_id = 10000;

    marker_scale = {0.01,0.01,0.01};
    marker_color =  {1.0,1.0,0.0,1.0};
    disp->changeConnectionSize(marker_scale);
    disp->displayPathAndWaypoints(current_path,path_id,wp_id,"pathplan",marker_color);
    disp->defaultConnectionSize();

    for(unsigned int i=0; i<other_paths.size();i++)
    {
      if(i==0) marker_color = {0.0f,0.0f,1.0,1.0};
      if(i==1) marker_color = {1.0,0.0f,0.0f,1.0};
      if(i==2) marker_color = {0.0,0.5,0.5,1.0};
      if(i==3) marker_color = {0.5,0.5,0.0,1.0};
      if(i==4) marker_color = {0.45,0.0,1.0,1.0};
      if(i==5) marker_color = {0.75,0.25,0.0,1.0};
      if(i==6) marker_color = {1.0,0.85,0.35,1.0};
      if(i==7) marker_color = {0.0,1.0,0.5,1.0};
      if(i==8) marker_color = {0.98,0.85,0.87,1.0};
      if(i==9) marker_color = {0.27,0.35,0.27,1.0};

      path_id += 1;
      wp_id += 1000;
      disp->displayPathAndWaypoints(other_paths.at(i),path_id,wp_id,"pathplan",marker_color);
    }

    disp->changeNodeSize(marker_scale_sphere);
    marker_color = {1.0,0.0,1.0,1.0};
    disp->displayNode(std::make_shared<pathplan::Node>(current_configuration),node_id,"pathplan",marker_color);

    Eigen::VectorXd point2project(pnt.positions.size());
    for(unsigned int i=0; i<pnt.positions.size();i++) point2project[i] = pnt.positions.at(i);
    node_id +=1;
    marker_color = {0.0,1.0,0.0,1.0};
    disp->displayNode(std::make_shared<pathplan::Node>(point2project),node_id,"pathplan",marker_color);

    node_id +=1;
    marker_color = {0.0,0.0,0.0,1.0};
    disp->displayNode(std::make_shared<pathplan::Node>(configuration_replan),node_id,"pathplan",marker_color);

    for(unsigned int i=0; i<pnt_replan.positions.size();i++) point2project[i] = pnt_replan.positions.at(i);
    node_id +=1;
    marker_color = {0.5,0.5,0.5,1.0};
    disp->displayNode(std::make_shared<pathplan::Node>(point2project),node_id,"pathplan",marker_color);
    trj_mtx_.unlock();
    disp->defaultNodeSize();

    lp.sleep();
  }
}

void ReplannerManager::spawnObjects()
{
  object_loader_msgs::addObjects srv_add_object;
  object_loader_msgs::removeObjects srv_remove_object;

  ros::Rate lp(0.5*trj_execution_thread_frequency_);

  bool object_spawned = false;
  bool second_object_spawned = true;
  bool third_object_spawned = true;
  while (!stop_)
  {
    // ////////////////////////////////////////////SPAWNING THE OBJECT/////////////////////////////////////////////
    if(real_time_>=1.10 && !third_object_spawned)  //1.5
    {
      third_object_spawned = true;
      object_spawned = false;
    }

    if(real_time_>=1.0 && !second_object_spawned)
    {
      second_object_spawned = true;
      object_spawned = false;
    }

    if(!object_spawned && real_time_>=0.50)
    {
      if (!add_obj_.waitForExistence(ros::Duration(10)))
      {
        ROS_FATAL("srv not found");
      }
      object_loader_msgs::object obj;
      obj.object_type="scatola";

      int obj_conn_pos;
      int idx_current_conn;

      replanner_mtx_.lock();
      trj_mtx_.lock();
      replanner_->getCurrentPath()->findConnection(current_configuration_,idx_current_conn);
      trj_mtx_.unlock();
      replanner_mtx_.unlock();

      if(third_object_spawned)
      {
        obj_conn_pos = idx_current_conn;
      }
      else
      {
        replanner_mtx_.lock();
        int size = replanner_->getCurrentPath()->getConnections().size();
        replanner_mtx_.unlock();

        std::srand(time(NULL));
        obj_conn_pos = rand() % (size-idx_current_conn) + idx_current_conn;

        if(obj_conn_pos == idx_current_conn) obj_conn_pos +=1;  //ELIMINA
      }
      pathplan::ConnectionPtr obj_conn;
      pathplan::NodePtr obj_parent;
      pathplan::NodePtr obj_child;
      Eigen::VectorXd obj_pos;
      if(obj_conn_pos == idx_current_conn)
      {
        replanner_mtx_.lock();
        trj_mtx_.lock();
        obj_conn = replanner_->getCurrentPath()->getConnections().at(obj_conn_pos);
        obj_child = obj_conn->getChild();
        obj_pos = obj_child->getConfiguration();

        if((obj_pos-configuration_replan_).norm()<0.20 && ((obj_conn_pos+1)<replanner_->getCurrentPath()->getConnections().size()))
        {
          //ROS_WARN("Shifting the object..");
          obj_conn = replanner_->getCurrentPath()->getConnections().at(obj_conn_pos+1);
          obj_parent = obj_conn->getParent();
          obj_child = obj_conn->getChild();

          Eigen::VectorXd conn_vect = obj_child->getConfiguration()-obj_parent->getConfiguration();
          obj_pos = obj_parent->getConfiguration()+0.5*conn_vect;
        }
        trj_mtx_.unlock();
        replanner_mtx_.unlock();
      }
      else
      {
        replanner_mtx_.lock();
        obj_conn = replanner_->getCurrentPath()->getConnections().at(obj_conn_pos);
        obj_parent = obj_conn->getParent();
        obj_child = obj_conn->getChild();
        obj_pos =  (obj_child->getConfiguration() +  obj_parent->getConfiguration())/2;
        replanner_mtx_.unlock();
      }

      moveit::core::RobotState obj_pos_state = trajectory_->fromWaypoints2State(obj_pos);
      tf::poseEigenToMsg(obj_pos_state.getGlobalLinkTransform(last_link_),obj.pose.pose);

      obj.pose.header.frame_id="world";

      srv_add_object.request.objects.clear();
      srv_add_object.request.objects.push_back(obj);

      scene_mtx_.lock();
      if (!add_obj_.call(srv_add_object))
      {
        ROS_ERROR("call to srv not ok");
      }
      if (!srv_add_object.response.success)
      {
        ROS_ERROR("srv error");
      }
      else
      {
        for (const std::string& str: srv_add_object.response.ids)
        {
          srv_remove_object.request.obj_ids.push_back(str);   //per rimuovere gli oggetti alla fine
        }
      }
      scene_mtx_.unlock();

      object_spawned = true;
      ROS_WARN("OBJECT SPAWNED");
    }

    lp.sleep();
  }

  ros::Duration(5).sleep();

  scene_mtx_.lock();
  if (!remove_obj_.waitForExistence(ros::Duration(10)))
  {
    ROS_FATAL("srv not found");
  }
  if (!remove_obj_.call(srv_remove_object))
  {
    ROS_ERROR("call to srv not ok");
  }
  if (!srv_remove_object.response.success)
  {
    ROS_ERROR("srv error");
  }
  scene_mtx_.unlock();
}
}
