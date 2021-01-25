#include "graph_core/graph/trajectory.h"

namespace pathplan
{


Trajectory::Trajectory(const PathPtr path,
                       const ros::NodeHandle &nh,
                       const planning_scene::PlanningScenePtr &planning_scene,
                       const std::string &group_name,
                       const std::string &base_link,
                       const std::string &last_link)
{
  trj_ = NULL;
  path_ = path;
  nh_ = nh;
  kinematic_model_ = planning_scene->getRobotModel();
  planning_scene_ = planning_scene;
  group_name_ = group_name;
  base_link_ = base_link;
  last_link_ = last_link;
  display_publisher_ = nh_.advertise<moveit_msgs::DisplayTrajectory>("/display_planned_trj", 1,true);
}

PathPtr Trajectory::computeBiRRTPath(const NodePtr &start_node, NodePtr &goal_node, const Eigen::VectorXd& lb, const Eigen::VectorXd& ub, const MetricsPtr& metrics, const CollisionCheckerPtr& checker, const bool& optimizePath)
{
  pathplan::SamplerPtr sampler = std::make_shared<pathplan::InformedSampler>(start_node->getConfiguration(), goal_node->getConfiguration(), lb, ub);

  pathplan::BiRRT solver(metrics, checker, sampler);
  solver.config(nh_);
  solver.addStart(start_node);
  solver.addGoal(goal_node);

  pathplan::PathPtr solution;
  if (!solver.solve(solution, 1000))
  {
    ROS_INFO("No solutions found");
    assert(0);
  }

  //ROS_INFO_STREAM("found a solution with cost = " << solution->cost());

  if(optimizePath)
  {
    pathplan::PathLocalOptimizer path_solver(checker, metrics);
    path_solver.config(nh_);

    solution->setTree(solver.getStartTree());
    path_solver.setPath(solution);
    path_solver.solve(solution);
    //ROS_INFO_STREAM("improve the solution to cost = " << solution->cost());

    sampler->setCost(solution->cost());
    solver.getStartTree()->addBranch(solution->getConnections());

    pathplan::LocalInformedSamplerPtr local_sampler = std::make_shared<pathplan::LocalInformedSampler>(start_node->getConfiguration(), goal_node->getConfiguration(), lb, ub);

    for (unsigned int isol = 0; isol < solution->getConnections().size() - 1; isol++)
    {
      pathplan::ConnectionPtr conn = solution->getConnections().at(isol);
      if (conn->getChild()->getConfiguration().size() != start_node->getConfiguration().size())
      {
        ROS_ERROR_STREAM("size = " << conn->getChild()->getConfiguration().size());
        ROS_WARN_STREAM("node " << conn->getChild());
      }
      try
      {
        local_sampler->addBall(conn->getChild()->getConfiguration(), solution->cost() * 0.1);
      }
      catch (...)
      {
        ROS_ERROR_STREAM("size = " << conn->getChild()->getConfiguration().size());
        ROS_WARN_STREAM("node " << conn->getChild());
        assert(0);
      }
    }
    local_sampler->setCost(solution->cost());

    //ROS_INFO("Improving solution with RRT*");
    pathplan::RRTStar opt_solver(metrics, checker, local_sampler);
    opt_solver.addStartTree(solver.getStartTree());
    opt_solver.addGoal(goal_node);
    opt_solver.config(nh_);

    std::vector<pathplan::NodePtr> white_list;
    white_list.push_back(goal_node);

    ros::Duration max_time(3);
    ros::Time t0 = ros::Time::now();

    int stall_gen = 0;
    int max_stall_gen = 200;

    std::mt19937 gen;
    std::uniform_int_distribution<> id = std::uniform_int_distribution<>(0, max_stall_gen);

    for (unsigned int idx = 0; idx < 1000; idx++)
    {
      if (ros::Time::now() - t0 > max_time)
        break;

      if (opt_solver.update(solution))
      {
        stall_gen = 0;
        //ROS_INFO_STREAM("Iteration = " << idx << " , rrt* reduce cost to = " << solution->cost());
        path_solver.setPath(solution);
        solution->setTree(opt_solver.getStartTree());
        //      path_solver.solve(solution);
        //ROS_INFO_STREAM("local solver improve the solution to cost = " << solution->cost());

        local_sampler->setCost(solution->cost());
        sampler->setCost(solution->cost());
        opt_solver.getStartTree()->purgeNodes(sampler, white_list, true);

        local_sampler->clearBalls();
        for (unsigned int isol = 0; isol < solution->getConnections().size() - 1; isol++)
        {
          pathplan::ConnectionPtr conn = solution->getConnections().at(isol);
          if (conn->getChild()->getConfiguration().size() != start_node->getConfiguration().size())
          {
            ROS_ERROR_STREAM("size = " << conn->getChild()->getConfiguration().size());
            ROS_WARN_STREAM("node " << conn->getChild());
          }
          //if (conn->getCost()>1.01*conn->norm())
          {
            try
            {
              local_sampler->addBall(conn->getChild()->getConfiguration(), solution->cost() * 0.1);
            }
            catch (...)
            {
              ROS_ERROR_STREAM("size = " << conn->getChild()->getConfiguration().size());
              ROS_WARN_STREAM("node " << conn->getChild());
              assert(0);
            }
          }
        }
      }
      else
      {
        opt_solver.getStartTree()->purgeNodes(sampler, white_list, false);
        stall_gen++;
      }


      if (idx % 10 == 0)
        //ROS_INFO("iter=%u,tree with %u nodes", idx, opt_solver.getStartTree()->getNumberOfNodes());

        if (id(gen) < stall_gen)
          opt_solver.setSampler(sampler);
        else
          opt_solver.setSampler(local_sampler);

      if (stall_gen >= max_stall_gen)
        break;
    }

    //ROS_INFO_STREAM("solution cost = " << solution->cost());
    //ROS_INFO("tree with %u nodes", opt_solver.getStartTree()->getNumberOfNodes());

    path_solver.setPath(solution);
    path_solver.solve(solution);
    //ROS_INFO_STREAM("local solver improve the solution to cost = " << solution->cost());

    //ROS_INFO_STREAM("solution\n" << *solution);

  }

  return solution;
}


std::vector<moveit::core::RobotState> Trajectory::fromWaypoints2State(const std::vector<Eigen::VectorXd> waypoints)
{
  std::vector<moveit::core::RobotState> wp_state_vector;
  for(const Eigen::VectorXd& waypoint: waypoints)
  {
    moveit::core::RobotState wp_state=fromWaypoints2State(waypoint);
    wp_state_vector.push_back(wp_state);
  }
  return wp_state_vector;
}

moveit::core::RobotState Trajectory::fromWaypoints2State(Eigen::VectorXd waypoint)
{
  moveit::core::RobotState wp_state=planning_scene_->getCurrentState();
  wp_state.setJointGroupPositions(group_name_,waypoint);
  wp_state.update();

  return wp_state;
}

robot_trajectory::RobotTrajectoryPtr Trajectory::fromPath2Trj()
{
  if(path_ == NULL)
  {
    throw std::invalid_argument("Path not assigned");
  }
  std::vector<Eigen::VectorXd> waypoints=path_->getWaypoints();
  std::vector<moveit::core::RobotState> wp_state_vector = fromWaypoints2State(waypoints);

  //Definizione della traiettoria, noti i waypoints del path
  trj_ = std::make_shared<robot_trajectory::RobotTrajectory>(kinematic_model_,group_name_);
  for(unsigned int j=0; j<waypoints.size();j++)
  {
    trj_->addSuffixWayPoint(wp_state_vector.at(j),0);
  }

  trajectory_processing::IterativeParabolicTimeParameterization iptp;
  //trajectory_processing::IterativeParabolicTimeParameterization iptp(100,0.001);
  //trajectory_processing::TimeOptimalTrajectoryGeneration iptp(0.1,0.1,0.001);

  iptp.computeTimeStamps(*trj_);

  return trj_;
}

robot_trajectory::RobotTrajectoryPtr Trajectory::fromPath2Trj(const trajectory_msgs::JointTrajectoryPoint& pnt)
{
  if(path_ == NULL)
  {
    throw std::invalid_argument("Path not assigned");
  }

  std::vector<Eigen::VectorXd> waypoints=path_->getWaypoints();
  std::vector<moveit::core::RobotState> wp_state_vector = fromWaypoints2State(waypoints);

  //Definizione della traiettoria, noti i waypoints del path
  trj_ = std::make_shared<robot_trajectory::RobotTrajectory>(kinematic_model_,group_name_);
  for(unsigned int j=0; j<waypoints.size();j++)
  {
    if (j==0)
    {
      wp_state_vector.at(j).setJointGroupPositions(group_name_,pnt.positions);
      wp_state_vector.at(j).setJointGroupVelocities(group_name_,pnt.velocities);
      wp_state_vector.at(j).setJointGroupAccelerations(group_name_,pnt.accelerations);
    }
    trj_->addSuffixWayPoint(wp_state_vector.at(j),0); //time parametrization
  }

  trajectory_processing::IterativeParabolicTimeParameterization iptp;
  //trajectory_processing::IterativeParabolicTimeParameterization iptp(100,0.001);
  //trajectory_processing::TimeOptimalTrajectoryGeneration iptp(0.1,0.1,0.001);
  iptp.computeTimeStamps(*trj_);

  return trj_;
}

robot_trajectory::RobotTrajectoryPtr Trajectory::fromPath2Trj(const trajectory_msgs::JointTrajectoryPoint& pnt, const trajectory_msgs::JointTrajectoryPoint& pnt2, const int &index, const double &dt)
{
  if(path_ == NULL)
  {
    throw std::invalid_argument("Path not assigned");
  }

  Eigen::VectorXd intermediate_pt(pnt.positions.size());
  for(unsigned int i=0; i<pnt2.positions.size();i++) intermediate_pt[i] = pnt2.positions.at(i);
  moveit::core::RobotState intermediate_state = fromWaypoints2State(intermediate_pt);

  std::vector<Eigen::VectorXd> waypoints=path_->getWaypoints();
  std::vector<moveit::core::RobotState> wp_state_vector = fromWaypoints2State(waypoints);

  //Definizione della traiettoria, noti i waypoints del path
  trj_ = std::make_shared<robot_trajectory::RobotTrajectory>(kinematic_model_,group_name_);
  for(unsigned int j=0; j<waypoints.size();j++)
  {
    if (j==0)
    {
      wp_state_vector.at(j).setJointGroupPositions(group_name_,pnt.positions);
      wp_state_vector.at(j).setJointGroupVelocities(group_name_,pnt.velocities);
      wp_state_vector.at(j).setJointGroupAccelerations(group_name_,pnt.accelerations);
    }
    trj_->addSuffixWayPoint(wp_state_vector.at(j),0); //time parametrization
  }

  trj_->insertWayPoint(index,intermediate_state,dt);

  trajectory_processing::IterativeParabolicTimeParameterization iptp;
  iptp.computeTimeStamps(*trj_);

  return trj_;
}

void Trajectory::displayTrj()
{
  moveit_msgs::RobotTrajectory trj_msg;
  trj_->getRobotTrajectoryMsg(trj_msg);

  moveit_msgs::DisplayTrajectory disp_trj;
  disp_trj.trajectory.push_back(trj_msg);
  disp_trj.model_id=kinematic_model_->getName();

  robot_state::RobotState start_state=planning_scene_->getCurrentState();
  start_state.setJointGroupPositions(group_name_,trj_msg.joint_trajectory.points.at(0).positions);
  start_state.setJointGroupVelocities(group_name_,trj_msg.joint_trajectory.points.at(0).velocities);
  start_state.setJointGroupAccelerations(group_name_,trj_msg.joint_trajectory.points.at(0).accelerations);
  moveit::core::robotStateToRobotStateMsg(start_state,disp_trj.trajectory_start);

  /*Visualize the trajectory*/
  moveit_visual_tools::MoveItVisualToolsPtr visual_tools = std::make_shared<moveit_visual_tools::MoveItVisualTools>(base_link_,"/rviz_visual_tools");

  visual_tools->loadRobotStatePub("/display_robot_state");
  visual_tools->enableBatchPublishing();
  visual_tools->trigger();

  visual_tools->publishRobotState(planning_scene_->getCurrentStateNonConst());
  visual_tools->trigger();

  display_publisher_.publish(disp_trj);  //to display the robot following the trj
  visual_tools->trigger();
}

}
