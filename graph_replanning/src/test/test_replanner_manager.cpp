#include <ros/ros.h>
#include <graph_replanning/replanner_manager.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "node_replanner");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::ServiceClient plannning_scene_client=nh.serviceClient<moveit_msgs::GetPlanningScene>("/get_planning_scene");

  bool optimize_path;
  if (!nh.getParam("opt_path", optimize_path))
  {
    ROS_INFO("optimize_path not set, used 1");
    optimize_path = 1;
  }

  double k_freq;
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

    moveit::planning_interface::MoveGroupInterface move_group(group_name);
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
    planning_scene::PlanningScenePtr planning_scn = std::make_shared<planning_scene::PlanningScene>(kinematic_model);

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

    Eigen::VectorXd start_conf = Eigen::Map<Eigen::VectorXd>(start_configuration.data(), start_configuration.size());
    Eigen::VectorXd goal_conf = Eigen::Map<Eigen::VectorXd>(stop_configuration.data(), stop_configuration.size());
    pathplan::NodePtr start_node = std::make_shared<pathplan::Node>(start_conf);

    pathplan::MetricsPtr metrics = std::make_shared<pathplan::Metrics>();
    pathplan::CollisionCheckerPtr checker = std::make_shared<pathplan::MoveitCollisionChecker>(planning_scn, group_name);

    // //////////////////////////////////////PATH PLAN//////////////////////////////////////////////////////////////////////////
    pathplan::PathPtr path = NULL;
    pathplan::TrajectoryPtr trajectory = std::make_shared<pathplan::Trajectory>(path,nh,planning_scn,group_name,base_link,last_link);

    std::vector<pathplan::PathPtr> path_vector;

    for (unsigned int i =0; i<4; i++)
    {
      pathplan::NodePtr goal_node = std::make_shared<pathplan::Node>(goal_conf);
      pathplan::PathPtr solution = trajectory->computeBiRRTPath(start_node, goal_node, lb, ub, metrics, checker, optimize_path);
      path_vector.push_back(solution);
    }

    // ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    pathplan::PathPtr current_path = path_vector.front();
    std::vector<pathplan::PathPtr> other_paths = {path_vector.at(1),path_vector.at(2),path_vector.at(3)};

    pathplan::ReplannerManagerPtr replanner_manager = std::make_shared<pathplan::ReplannerManager>(current_path, other_paths, nh);
    replanner_manager->trajectoryExecutionThread();
  }

  return 0;
}
