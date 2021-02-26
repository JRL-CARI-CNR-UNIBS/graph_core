#include <ros/ros.h>
#include <graph_core/metrics.h>
#include <graph_core/occupancy_metrics.h>
#include <graph_core/graph/node.h>
#include <graph_core/graph/path.h>
#include <graph_core/graph/tree.h>
#include <graph_core/graph/graph_display.h>
#include <graph_core/graph/trajectory.h>
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
#include <object_loader_msgs/removeObjects.h>
#include <thread>
#include <mutex>
#include <std_msgs/Float64.h>
#include <std_srvs/Empty.h>

volatile bool stop=false;
pathplan::ReplannerPtr replanner;
Eigen::VectorXd current_configuration;
Eigen::VectorXd configuration_replan;
planning_scene::PlanningScenePtr planning_scn;
planning_scene::PlanningScenePtr planning_scn_replanning;
pathplan::CollisionCheckerPtr checker_thread_cc;
pathplan::CollisionCheckerPtr checker;
std::mutex planning_mtx;
std::mutex trj_mtx;
std::mutex checker_mtx;
std::mutex display_mtx;
std::mutex scene_mtx;
std::mutex replanner_mtx;
pathplan::TrajectoryPtr trajectory;
pathplan::DisplayPtr disp;
moveit_msgs::RobotTrajectory trj_msg;
double real_time;
double t;
double dt;
double dt_replan;
double dt_replan_no_obstruction;
double main_frequency;
double replan_frequency;
double replan_offset;
double t_replan;
trajectory_processing::SplineInterpolator interpolator;
trajectory_msgs::JointTrajectoryPoint pnt;
trajectory_msgs::JointTrajectoryPoint pnt_replan;
Eigen::VectorXd past_configuration_replan;
int n_conn;
int n_conn_replan;
bool first_replan;
bool path_obstructed;
bool computing_avoiding_path;
double checker_frequency;
double k_freq;
std::string test;
std::string group_name;
std::string base_link;
std::string last_link;
std::vector<pathplan::PathPtr> other_paths;
pathplan::PathPtr current_path;

ros::Publisher current_norm_pub;
ros::Publisher new_norm_pub;
ros::Publisher time_replanning_pub;
ros::Publisher obs_current_norm_pub;
ros::Publisher obs_new_norm_pub;
ros::Publisher obs_time_replanning_pub;

ros::ServiceClient plannning_scene_client;
ros::ServiceClient add_obj;
ros::ServiceClient remove_obj;

int cont_obs=0; //elimina

void replanning_fcn()
{
  replan_frequency = k_freq*replan_frequency;
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

  cont_obs = 0;

  while (!stop)
  {
    ros::WallTime tic_tot=ros::WallTime::now();

    trj_mtx.lock();
    t_replan=t+replan_offset;

    interpolator.interpolate(ros::Duration(t_replan),pnt_replan);
    for(unsigned int i=0; i<pnt_replan.positions.size();i++) point2project[i] = pnt_replan.positions.at(i);

    past_configuration_replan = configuration_replan;
    past_abscissa = abscissa;

    configuration_replan = replanner->getCurrentPath()->projectOnClosestConnectionKeepingCurvilinearAbscissa(point2project,past_configuration_replan,abscissa,past_abscissa,n_conn_replan);

    goal = replanner->getCurrentPath()->getConnections().back()->getChild()->getConfiguration();
    replan = ((current_configuration-goal).norm()>1e-03 && (configuration_replan-goal).norm()>1e-03);
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

    if(replan)
    {
      double time_informedOnlineRepl;
      std::string string_dt;

      checker_mtx.lock();

      scene_mtx.lock();
      moveit_msgs::PlanningScene scn;
      planning_scn->getPlanningSceneMsg(scn);
      planning_scn_replanning->setPlanningSceneMsg(scn);
      scene_mtx.unlock();

      pathplan::PathPtr current_path_copy = current_path->clone();
      std::vector<pathplan::PathPtr> other_paths_copy;
      for(const pathplan::PathPtr path:other_paths) other_paths_copy.push_back(path->clone());

      replanner_mtx.lock();     
      if(current_path_copy->findConnection(configuration_replan) == NULL)
      {
        trj_mtx.lock();
        configuration_replan = current_configuration;  //può essere che non venga trovata la repl conf e rimanga ferma per qualche ciclo, intanto la current conf va avanti, la supera e il replanned path partirà da questa e non includera la repl conf
        trj_mtx.unlock();
      }
      replanner->setCurrentConf(configuration_replan);
      replanner->setCurrentPath(current_path_copy);
      replanner->setOtherPaths(other_paths_copy);
      replanner_mtx.unlock();

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
      checker_mtx.unlock();

      planning_mtx.lock();
      if(current_path_copy->findConnection(configuration_replan) == NULL)
      {
        ROS_ERROR("PROBLEMA CONF REPL 2");
        assert(0);
      }

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

          replanner_mtx.lock();
          replanner->addOtherPath(replanner->getCurrentPath());
          replanner_mtx.unlock();

          checker_mtx.lock();
          other_paths.push_back(current_path_copy);
          checker_mtx.unlock();
        }

        std_msgs::Float64 current_norm;
        std_msgs::Float64 new_norm;
        std_msgs::Float64 time_replanning;

        checker_mtx.lock();
        replanner_mtx.lock();
        trj_mtx.lock();
        replanner->startReplannedPathFromNewCurrentConf(current_configuration);
        replanner->simplifyReplannedPath(0.1);
        if(replanner->getReplannedPath()->findConnection(current_configuration) == NULL) ROS_ERROR("PROBLEMA CURRENT CONF");

        current_norm.data = replanner->getCurrentPath()->getNormFromConf(current_configuration);
        new_norm.data = replanner->getReplannedPath()->cost();
        time_replanning.data = (toc_rep-tic_rep).toSec();

        replanner->setCurrentPath(replanner->getReplannedPath());
        current_path = replanner->getReplannedPath();

        tic_trj=ros::WallTime::now();
        moveit_msgs::RobotTrajectory tmp_trj_msg;
        trajectory->setPath(current_path);
        robot_trajectory::RobotTrajectoryPtr trj= trajectory->fromPath2Trj(pnt);
        trj->getRobotTrajectoryMsg(tmp_trj_msg);
        toc_trj=ros::WallTime::now();

        interpolator.setTrajectory(tmp_trj_msg);
        interpolator.setSplineOrder(1);

        t=0;
        t_replan=0;
        n_conn = 0;
        n_conn_replan = 0;
        abscissa = 0;
        past_abscissa = 0;
        trj_mtx.unlock();
        replanner_mtx.unlock();

        if(computing_avoiding_path)
        {
          cont_obs++;
          ROS_WARN("cont_obs: %d",cont_obs);
          obs_current_norm_pub.publish(current_norm);
          obs_new_norm_pub.publish(new_norm);
          obs_time_replanning_pub.publish(time_replanning);
        }
        else
        {
          current_norm_pub.publish(current_norm);
          new_norm_pub.publish(new_norm);
          time_replanning_pub.publish(time_replanning);
        }

        path_obstructed = false;
        computing_avoiding_path = false;
        checker_mtx.unlock();

        std::vector<double> marker_scale(3,0.01);
        std::vector<double> marker_color = {1.0,1.0,0.0,1.0};
        display_mtx.lock();
        disp->changeConnectionSize(marker_scale);
        disp->displayPathAndWaypoints(replanner->getReplannedPath(),replanned_path_id,-12,"pathplan",marker_color);
        disp->defaultConnectionSize();
        display_mtx.unlock();
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
  object_loader_msgs::addObjects srv_add_object;
  object_loader_msgs::removeObjects srv_remove_object;
  moveit_msgs::GetPlanningScene ps_srv;

  checker_frequency = k_freq*checker_frequency;

  ros::Rate lp(checker_frequency);
  bool object_spawned = false;
  bool second_object_spawned = false;
  bool third_object_spawned = false;
  bool fourth_object_spawned = false;
  while (!stop)
  {
    // ////////////////////////////////////////////SPAWNING THE OBJECT/////////////////////////////////////////////
    if(real_time>=1.85 && !fourth_object_spawned)  //CAMBIA
    {
      fourth_object_spawned = true;
      object_spawned = false;
    }

    if(real_time>=1.5 && !third_object_spawned)
    {
      third_object_spawned = true;
      object_spawned = false;
    }

    if(real_time>=1 && !second_object_spawned)
    {
      second_object_spawned = true;
      object_spawned = false;
    }

    if(!object_spawned && real_time>=0.50)
    {
      if (!add_obj.waitForExistence(ros::Duration(10)))
      {
        ROS_FATAL("srv not found");
      }
      object_loader_msgs::object obj;
      obj.object_type="scatola";

      int obj_conn_pos;
      int idx_current_conn;

      replanner_mtx.lock();
      trj_mtx.lock();
      replanner->getCurrentPath()->findConnection(current_configuration,idx_current_conn);
      trj_mtx.unlock();
      replanner_mtx.unlock();

      if(fourth_object_spawned)
      {
        obj_conn_pos = idx_current_conn;
      }
      else
      {
        replanner_mtx.lock();
        int size = replanner->getCurrentPath()->getConnections().size();
        replanner_mtx.unlock();

        std::srand(time(NULL));
        obj_conn_pos = rand() % (size-idx_current_conn) + idx_current_conn;
      }
      pathplan::ConnectionPtr obj_conn;
      pathplan::NodePtr obj_parent;
      pathplan::NodePtr obj_child;
      Eigen::VectorXd obj_pos;
      if(obj_conn_pos == idx_current_conn)
      {
        replanner_mtx.lock();
        trj_mtx.lock();
        obj_conn = replanner->getCurrentPath()->getConnections().at(obj_conn_pos);
        obj_child = obj_conn->getChild();
        obj_pos = obj_child->getConfiguration();

        if((obj_pos-configuration_replan).norm()<0.15 && ((obj_conn_pos+1)<replanner->getCurrentPath()->getConnections().size()))
        {
          //ROS_WARN("Shifting the object..");
          obj_conn = replanner->getCurrentPath()->getConnections().at(obj_conn_pos+1);
          obj_parent = obj_conn->getParent();
          obj_child = obj_conn->getChild();

          Eigen::VectorXd conn_vect = obj_child->getConfiguration()-obj_parent->getConfiguration();
          obj_pos = obj_parent->getConfiguration()+0.5*conn_vect;
        }
        trj_mtx.unlock();
        replanner_mtx.unlock();
      }
      else
      {
        replanner_mtx.lock();
        obj_conn = replanner->getCurrentPath()->getConnections().at(obj_conn_pos);
        obj_parent = obj_conn->getParent();
        obj_child = obj_conn->getChild();
        obj_pos =  (obj_child->getConfiguration() +  obj_parent->getConfiguration())/2;
        replanner_mtx.unlock();
      }

      /*pathplan::ConnectionPtr obj_conn;
      pathplan::NodePtr obj_parent;
      pathplan::NodePtr obj_child;
      Eigen::VectorXd obj_pos;

      replanner_mtx.lock();
      obj_conn_pos = replanner->getCurrentPath()->getConnections().size()-2;
      obj_conn = replanner->getCurrentPath()->getConnections().at(obj_conn_pos);
      replanner_mtx.unlock();
      obj_parent = obj_conn->getParent();
      obj_child = obj_conn->getChild();
      obj_pos = (obj_child->getConfiguration() +  obj_parent->getConfiguration())/2;*/

      moveit::core::RobotState obj_pos_state = trajectory->fromWaypoints2State(obj_pos);
      tf::poseEigenToMsg(obj_pos_state.getGlobalLinkTransform(last_link),obj.pose.pose);

      obj.pose.header.frame_id="world";

      srv_add_object.request.objects.clear();
      srv_add_object.request.objects.push_back(obj);

      scene_mtx.lock();
      if (!add_obj.call(srv_add_object))
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
      scene_mtx.unlock();

      object_spawned = true;
      ROS_WARN("OBJECT SPAWNED");
    }

    // ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    scene_mtx.lock();
    if (!plannning_scene_client.call(ps_srv))
    {
      ROS_ERROR("call to srv not ok");
    }

    if (!planning_scn->setPlanningSceneMsg(ps_srv.response.scene))
    {
      ROS_ERROR("unable to update planning scene");
    }
    std::vector<moveit_msgs::CollisionObject> collision_objs;
    planning_scn->getCollisionObjectMsgs(collision_objs);
    scene_mtx.unlock();

    checker_mtx.lock();
    current_path->isValid(checker_thread_cc);
    for(const pathplan::PathPtr& path: other_paths) path->isValid(checker_thread_cc);

    if(!computing_avoiding_path)
    {
      trj_mtx.lock();
      path_obstructed = !(current_path->isValidFromConf(current_configuration,checker_thread_cc));
      trj_mtx.unlock();
    }
    checker_mtx.unlock();

    lp.sleep();
  }

  ros::Duration(5).sleep();

  scene_mtx.lock();
  if (!remove_obj.waitForExistence(ros::Duration(10)))
  {
    ROS_FATAL("srv not found");
  }
  if (!remove_obj.call(srv_remove_object))
  {
    ROS_ERROR("call to srv not ok");
  }
  if (!srv_remove_object.response.success)
  {
    ROS_ERROR("srv error");
  }
  scene_mtx.unlock();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "node_replanner");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::NodeHandle nh;
  ros::Publisher target_pub=nh.advertise<sensor_msgs::JointState>("/joint_target",1);
  ros::Publisher time_pub=nh.advertise<std_msgs::Float64>("/time_topic",1);
  ros::ServiceClient start_log = nh.serviceClient<std_srvs::Empty> ("/start_log");
  ros::ServiceClient stop_log  = nh.serviceClient<std_srvs::Empty> ("/stop_log");

  plannning_scene_client=nh.serviceClient<moveit_msgs::GetPlanningScene>("/get_planning_scene");
  add_obj=nh.serviceClient<object_loader_msgs::addObjects>("add_object_to_scene");
  remove_obj=nh.serviceClient<object_loader_msgs::removeObjects>("/remove_object_from_scene");

  current_norm_pub = nh.advertise<std_msgs::Float64>("/current_norm_topic", 1000);
  new_norm_pub = nh.advertise<std_msgs::Float64>("/new_norm_topic", 1000);
  time_replanning_pub = nh.advertise<std_msgs::Float64>("/time_replanning_topic", 1000);
  obs_current_norm_pub = nh.advertise<std_msgs::Float64>("/obs_current_norm_topic", 1000);
  obs_new_norm_pub = nh.advertise<std_msgs::Float64>("/obs_new_norm_topic", 1000);
  obs_time_replanning_pub = nh.advertise<std_msgs::Float64>("/obs_time_replanning_topic", 1000);

  if (!plannning_scene_client.waitForExistence(ros::Duration(10)))
  {
    ROS_ERROR("unable to connect to /get_planning_scene");
  }

  ros::Duration(0.5).sleep();

  bool optimize_path;
  if (!nh.getParam("opt_path", optimize_path))
  {
    ROS_INFO("optimize_path not set, used 1");
    optimize_path = 1;
  }

  if (!nh.getParam("k_freq",k_freq))
  {
    ROS_INFO("k_freq not set, use 1");
    k_freq=1;
  }

  int init_test;
  if (!nh.getParam("init_test",init_test))
  {
    ROS_INFO("init_test not set, use 0");
    init_test=0;
  }

  int end_test;
  if (!nh.getParam("end_test",end_test))
  {
    ROS_INFO("end_test not set, use 10");
    end_test=10;
  }

  std::string test_name;
  if (!nh.getParam("/binary_logger/test_name", test_name))
  {
    ROS_INFO("test_name not set, used no_name");
    test_name = "no_name";
  }

  if (!nh.getParam("dt_replan", dt_replan))
  {
    ROS_INFO("dt_replan not set, used 0.05");
    dt_replan = 0.050;
  }

  if (!nh.getParam("dt_replan_no_obstruction", dt_replan_no_obstruction))
  {
    ROS_INFO("dt_replan_no_obstruction not set, used 0.10");
    dt_replan_no_obstruction = 0.10;
  }

  if (!nh.getParam("group_name",group_name))
  {
    ROS_ERROR("group_name not set, exit");
    return 0;
  }
  if (!nh.getParam("base_link",base_link))
  {
    ROS_ERROR("base_link not set, exit");
    return 0;
  }
  if (!nh.getParam("last_link",last_link))
  {
    ROS_ERROR("last_link not set, exit");
    return 0;
  }
  if (!nh.getParam("test_name",test))
  {
    ROS_INFO("test_name not set, use noTest");
    test="noTest";
  }

  std::vector<double> start_configuration;
  if (!nh.getParam("start_configuration",start_configuration))
  {
    ROS_ERROR("start_configuration not set, exit");
    return 0;
  }

  std::vector<double> stop_configuration;
  if (!nh.getParam("stop_configuration",stop_configuration))
  {
    ROS_ERROR("stop_configuration not set, exit");
    return 0;
  }

  for(int n_iter = init_test; n_iter<end_test; n_iter++)
  {
    ROS_WARN("ITER n: %d",n_iter+1);
    ros::Duration(1).sleep();

    // //////////////////INITIALIZATION//////////////////////////////////////
    real_time = 0.0;
    t=0.0;
    dt=0.01;
    main_frequency = 1/dt;
    replan_frequency = 1/dt_replan;
    replan_offset=(dt_replan-dt)*2;
    t_replan=t+replan_offset;
    n_conn = 0;
    n_conn_replan = 0;
    first_replan = true;
    path_obstructed = false;
    computing_avoiding_path = false;
    checker_frequency = 30;
    // /////////////////////////////////////////////////////////////////////

    moveit::planning_interface::MoveGroupInterface move_group(group_name);
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
    planning_scn = std::make_shared<planning_scene::PlanningScene>(kinematic_model);
    planning_scn_replanning = std::make_shared<planning_scene::PlanningScene>(kinematic_model);

    const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(group_name);
    std::vector<std::string> joint_names = joint_model_group->getActiveJointModelNames();

    // UPDATING PLANNING SCENE
    moveit_msgs::GetPlanningScene ps_srv;
    if (!plannning_scene_client.call(ps_srv))
    {
      ROS_ERROR("call to srv not ok");
      return 1;
    }

    if (!planning_scn->setPlanningSceneMsg(ps_srv.response.scene))
    {
      ROS_ERROR("unable to update planning scene");
      return 1;
    }
    if (!planning_scn_replanning->setPlanningSceneMsg(ps_srv.response.scene))
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

    Eigen::VectorXd start_conf = Eigen::Map<Eigen::VectorXd>(start_configuration.data(), start_configuration.size());
    Eigen::VectorXd goal_conf = Eigen::Map<Eigen::VectorXd>(stop_configuration.data(), stop_configuration.size());
    pathplan::NodePtr start_node = std::make_shared<pathplan::Node>(start_conf);

    pathplan::MetricsPtr metrics = std::make_shared<pathplan::Metrics>();
    checker_thread_cc = std::make_shared<pathplan::MoveitCollisionChecker>(planning_scn, group_name);
    checker = std::make_shared<pathplan::MoveitCollisionChecker>(planning_scn_replanning, group_name);

    // ///////////////////////////PATH PLAN & VISUALIZATION//////////////////////////////////////////////////////////////////////////

    disp = std::make_shared<pathplan::Display>(planning_scn,group_name,last_link);
    ros::Duration(0.5).sleep();
    pathplan::PathPtr path = NULL;
    trajectory = std::make_shared<pathplan::Trajectory>(path,nh,planning_scn_replanning,group_name,base_link,last_link);

    int current_node_id = -3;
    int trj_node_id = -4;
    int path_id = -324;
    int wp_id = -653;

    std::vector<pathplan::PathPtr> path_vector;

    for (unsigned int i =0; i<4; i++)
    {
      pathplan::NodePtr goal_node = std::make_shared<pathplan::Node>(goal_conf);

      pathplan::PathPtr solution = trajectory->computeBiRRTPath(start_node, goal_node, lb, ub, metrics, checker, optimize_path);
      path_vector.push_back(solution);

      std::vector<double> marker_color;
      if(i==0) marker_color = {0.5,0.5,0.0,1.0};
      if(i==1) marker_color = {0.0f,0.0f,1.0,1.0};
      if(i==2) marker_color = {1.0,0.0f,0.0f,1.0};
      if(i==3) marker_color = {0.0,0.5,0.5,1.0};

      path_id -=1;
      wp_id -=1;

      ros::Duration(0.1).sleep();
      disp->displayPathAndWaypoints(solution,path_id,wp_id,"pathplan",marker_color);
      ROS_INFO_STREAM("cost:"<<solution->cost());
    }

    // ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    current_path = path_vector.front();
    other_paths = {path_vector.at(1),path_vector.at(2),path_vector.at(3)};

    // //////////////////ELIMINA/////////////////////////
    pathplan::PathPtr path1 = current_path->clone();
    pathplan::PathPtr path2 = path_vector.at(1)->clone();
    pathplan::PathPtr path3 = path_vector.at(2)->clone();
    pathplan::PathPtr path4 = path_vector.at(3)->clone();
    // /////////////////////////////////////////////////

    trajectory->setPath(current_path);
    robot_trajectory::RobotTrajectoryPtr trj= trajectory->fromPath2Trj();
    moveit_msgs::RobotTrajectory tmp_trj_msg;
    trj->getRobotTrajectoryMsg(tmp_trj_msg);

    interpolator.setTrajectory(tmp_trj_msg);
    interpolator.setSplineOrder(1);

    pathplan::SamplerPtr samp = std::make_shared<pathplan::InformedSampler>(start_conf, goal_conf, lb, ub);
    pathplan::BiRRTPtr solver = std::make_shared<pathplan::BiRRT>(metrics, checker, samp);
    solver->config(nh);

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
    joint_state.header.stamp=ros::Time::now();

    target_pub.publish(joint_state);

    std_srvs::Empty srv_log;
    start_log.call(srv_log);

    nh.setParam("/binary_logger/test_name", std::to_string(n_iter+1)+"_"+test+ + "_" + test_name);

    stop=false;
    std::thread replanning_thread=std::thread(&replanning_fcn);
    std::thread col_check_thread=std::thread(&collision_check_fcn);

    main_frequency = k_freq*main_frequency;
    ros::Rate lp(main_frequency);

    std::vector<Eigen::VectorXd> pos_vec; //ELIMINA

    while (ros::ok() && !stop)
    {
      ros::WallTime inizio = ros::WallTime::now();

      replanner_mtx.lock();
      trj_mtx.lock();
      real_time += dt;
      t+=dt;

      interpolator.interpolate(ros::Duration(t),pnt);
      for(unsigned int i=0; i<pnt.positions.size();i++) point2project[i] = pnt.positions.at(i);
      past_current_configuration = current_configuration;
      current_configuration = replanner->getCurrentPath()->projectOnClosestConnectionKeepingPastPrj(point2project,past_current_configuration,n_conn);
      trj_mtx.unlock();
      replanner_mtx.unlock();

      pos_vec.push_back(current_configuration);

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
        joint_state.header.stamp=ros::Time::now();

        target_pub.publish(joint_state);

        std_msgs::Float64 time_msg;
        time_msg.data = real_time;
        time_pub.publish(time_msg);
      }

      ros::WallTime fine = ros::WallTime::now();
      bool in_time = lp.sleep();
      if(!in_time) ROS_ERROR("durata main %f",(fine-inizio).toSec());
    }

    // ///////// ELIMINA DA QUA ////////////////////////////////////////////

    std::vector<Eigen::VectorXd> versori;
    for(unsigned int i=1;i < pos_vec.size(); i++)
    {
      Eigen::VectorXd vers = (pos_vec.at(i)-pos_vec.at(i-1))/(pos_vec.at(i)-pos_vec.at(i-1)).norm();
      versori.push_back(vers);
    }

    std::vector<Eigen::VectorXd> nodi;
    nodi.push_back(start_conf);
    for(unsigned int i=1;i < versori.size(); i++)
    {
      nodi.push_back(pos_vec.at(i));
    }
    nodi.push_back(goal_conf);

    std::vector<pathplan::ConnectionPtr> con_vec;
    for(unsigned int i=1;i < nodi.size(); i++)
    {
      pathplan::NodePtr parent = std::make_shared<pathplan::Node>(nodi.at(i-1));
      pathplan::NodePtr child = std::make_shared<pathplan::Node>(nodi.at(i));
      pathplan::ConnectionPtr conn = std::make_shared<pathplan::Connection>(parent,child);
      conn->setCost(1);
      conn->add();

      con_vec.push_back(conn);
    }

    pathplan::PathPtr path_fake = std::make_shared<pathplan::Path>(con_vec,metrics,checker);

    ros::Duration(0.1).sleep();
    disp->clearMarkers();

    std::vector<double> marker_scale(3,0.01);
    std::vector<double> marker_color = {1.0,0.5,0.5,1.0};
    display_mtx.lock();
    disp->changeConnectionSize(marker_scale);
    disp->displayPath(path_fake,0,"pathplan",marker_color);
    disp->displayPath(path_fake,-10,"pathplan",marker_color);

    disp->defaultConnectionSize();
    disp->defaultNodeSize();
    ros::Duration(1).sleep();
    marker_color = {0.5,0.5,0.0,1.0};
    disp->displayPathAndWaypoints(path1,-1,-100,"pathplan",marker_color);
    marker_color = {0.0f,0.0f,1.0,1.0};
    disp->displayPathAndWaypoints(path2,-2,-200,"pathplan",marker_color);
    marker_color = {1.0,0.0f,0.0f,1.0};
    disp->displayPathAndWaypoints(path3,-3,-300,"pathplan",marker_color);
    marker_color = {0.0,0.5,0.5,1.0};
    disp->displayPathAndWaypoints(path4,-4,-400,"pathplan",marker_color);

    display_mtx.unlock();

    ros::Duration(10).sleep();

    // ////////// FINO A QUA //////////////////////////////////////////////

    ROS_ERROR("STOP");
    stop=true;

    if (replanning_thread.joinable())
      replanning_thread.join();
    if (col_check_thread.joinable())
      col_check_thread.join();

    // BINARY LOGGER SALVA FINO A I-1 ESIMO DATO PUBBLICATO, QUESTO AIUTA A SALVARLI TUTTI
    std_msgs::Float64 fake_data;
    sensor_msgs::JointState joint_fake;
    joint_fake.position = pnt.positions;
    joint_fake.velocity = pnt.velocities;
    joint_fake.name = joint_names;
    joint_fake.header.frame_id = kinematic_model->getModelFrame();
    joint_fake.header.stamp=ros::Time::now();
    fake_data.data = 0.0;
    obs_current_norm_pub.publish(fake_data);
    obs_new_norm_pub.publish(fake_data);
    obs_time_replanning_pub.publish(fake_data);
    current_norm_pub.publish(fake_data);
    new_norm_pub.publish(fake_data);
    time_replanning_pub.publish(fake_data);
    time_pub.publish(fake_data);
    target_pub.publish(joint_fake);

    stop_log.call(srv_log);

    ROS_ERROR("ITER n: %d -->obs pub: %d",n_iter+1,cont_obs);
  }

  return 0;
}
