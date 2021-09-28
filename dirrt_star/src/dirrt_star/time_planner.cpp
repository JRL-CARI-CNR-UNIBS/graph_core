/*
Copyright (c) 2019, Manuel Beschi CNR-STIIMA manuel.beschi@stiima.cnr.it
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the <organization> nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/


#include <dirrt_star/time_planner.h>



namespace pathplan {
namespace dirrt_star {

TimeBasedMultiGoalPlanner::TimeBasedMultiGoalPlanner ( const std::string& name,
                                                       const std::string& group,
                                                       const moveit::core::RobotModelConstPtr& model ) :
  PlanningContext ( name, group ),
  group_(group)
{
  nh_=ros::NodeHandle(name);
  nh_.setCallbackQueue(&queue_);


  COMMENT("create MultigoalPlanner, name =%s, group = %s", name.c_str(),group.c_str());
  robot_model_=model;
  if (!robot_model_)
  {
    ROS_ERROR("robot model is not initialized");
  }
  COMMENT("model name = %s",robot_model_->getName().c_str());

  COMMENT("get joint model group");

  const moveit::core::JointModelGroup* jmg=robot_model_->getJointModelGroup(group);
  if (jmg==NULL)
    ROS_ERROR("unable to find JointModelGroup for group %s",group.c_str());


  COMMENT("get joint names of JointModelGroup=%s",jmg->getName().c_str());

  joint_names_=jmg->getActiveJointModelNames();
  dof_=joint_names_.size();
  COMMENT("number of joints  = %u",dof_);
  lower_bounds_.resize(dof_);
  upper_bounds_.resize(dof_);
  max_velocity_.resize(dof_);

  COMMENT("read bounds");
  for (unsigned int idx=0;idx<dof_;idx++)
  {
    COMMENT("joint %s",joint_names_.at(idx).c_str());
    const robot_model::VariableBounds& bounds = robot_model_->getVariableBounds(joint_names_.at(idx));
    if (bounds.position_bounded_)
    {
      lower_bounds_(idx)=bounds.min_position_;
      upper_bounds_(idx)=bounds.max_position_;
      max_velocity_(idx)=bounds.max_velocity_;
    }
  }


  if (!nh_.getParam("norm2_weigth",nu_))
  {
    ROS_DEBUG("norm2_weigth is not set, default=1e-2");
    nu_=1e-2;
  }
  metrics_=std::make_shared<pathplan::TimeBasedMetrics>(max_velocity_,nu_);

  if (!nh_.getParam("plot_interval",plot_interval_))
  {
    ROS_DEBUG("plot_interval is not set, default=1e-2");
    plot_interval_=1;
  }

  COMMENT("created Time base planner");

  double refining_time=0;
  if (!nh_.getParam("max_refine_time",refining_time))
  {
    ROS_DEBUG("refining_time is not set, default=30");
    refining_time=30;
  }
  max_refining_time_=ros::WallDuration(refining_time);

}



void TimeBasedMultiGoalPlanner::clear()
{

}


bool TimeBasedMultiGoalPlanner::solve ( planning_interface::MotionPlanDetailedResponse& res )
{
  display=std::make_shared<pathplan::Display>(planning_scene_,group_);

  ros::WallDuration max_planning_time=ros::WallDuration(request_.allowed_planning_time);
  ros::WallTime start_time = ros::WallTime::now();
  ros::WallTime refine_time = ros::WallTime::now();
  is_running_=true;

  if (!planning_scene_)
  {
    ROS_ERROR("No planning scene available");
    res.error_code_.val=moveit_msgs::MoveItErrorCodes::COLLISION_CHECKING_UNAVAILABLE;
    is_running_=false;
    return false;
  }



  planning_scene::PlanningScenePtr ptr=planning_scene::PlanningScene::clone(planning_scene_);
  checker=std::make_shared<pathplan::ParallelMoveitCollisionChecker>(ptr,group_,collision_thread_,collision_distance);

  moveit::core::RobotState start_state(robot_model_);
  moveit::core::robotStateMsgToRobotState(request_.start_state,start_state);
  if (request_.start_state.joint_state.position.size()==0)
    start_state=planning_scene_->getCurrentState();
  else
    moveit::core::robotStateMsgToRobotState(request_.start_state,start_state);

  start_state.update();
  start_state.updateCollisionBodyTransforms();

  if (!start_state.satisfiesBounds())
  {
    ROS_FATAL("Start point is  Out of bound");
    res.error_code_.val=moveit_msgs::MoveItErrorCodes::START_STATE_IN_COLLISION;
    is_running_=false;
    return false;
  }
  Eigen::VectorXd start_conf;
  start_state.copyJointGroupPositions(group_,start_conf);

  if (!checker->check(start_conf))
  {
    ROS_ERROR("Start point is in collision");

    collision_detection::CollisionRequest col_req;
    collision_detection::CollisionResult col_res;
    col_req.contacts = true;
    col_req.group_name=group_;
    planning_scene_->checkCollision(col_req,col_res,start_state);
    if (col_res.collision)
    {
      ROS_ERROR("Start state is colliding +++");
      for (const  std::pair<std::pair<std::string, std::string>, std::vector<collision_detection::Contact> >& contact: col_res.contacts)
      {
        ROS_ERROR("contact between %s and %s",contact.first.first.c_str(),contact.first.second.c_str());
      }
    }
    res.error_code_.val=moveit_msgs::MoveItErrorCodes::START_STATE_IN_COLLISION;
    is_running_=false;
    return false;
  }


  pathplan::NodePtr start_node=std::make_shared<pathplan::Node>(start_conf);
  pathplan::SamplerPtr sampler = std::make_shared<pathplan::TimeBasedInformedSampler>(lower_bounds_, upper_bounds_, lower_bounds_, upper_bounds_,max_velocity_);
  pathplan::TreeSolverPtr solver=std::make_shared<pathplan::TimeMultigoalSolver>(metrics_, checker, sampler,max_velocity_);
  if (!solver->config(nh_))
  {
    ROS_ERROR("Unable to configure the planner");
    res.error_code_.val=moveit_msgs::MoveItErrorCodes::INVALID_GROUP_NAME;
    is_running_=false;
    return false;
  }
  solver->addStart(start_node);

  queue_.callAvailable();

  bool at_least_a_goal=false;

  // joint goal
  for (unsigned int iGoal=0;iGoal<request_.goal_constraints.size();iGoal++)
  {
    ROS_DEBUG("Processing goal %u",iGoal);

    moveit_msgs::Constraints goal=request_.goal_constraints.at(iGoal);

    Eigen::VectorXd goal_configuration( goal.joint_constraints.size() );
    moveit::core::RobotState goal_state(robot_model_);

    for (auto c: goal.joint_constraints)
      goal_state.setJointPositions(c.joint_name,&c.position);
    goal_state.copyJointGroupPositions(group_,goal_configuration);

    goal_state.updateCollisionBodyTransforms();
    COMMENT("check collision on goal %u",iGoal);

    if (!checker->check(goal_configuration))
    {
      ROS_DEBUG("goal %u is in collision",iGoal);

      if (request_.goal_constraints.size()<5)
      {

        if (!goal_state.satisfiesBounds())
        {
          ROS_INFO_STREAM("End state: " << goal_configuration.transpose()<<" is  Out of bound");
        }

        collision_detection::CollisionRequest col_req;
        collision_detection::CollisionResult col_res;
        col_req.contacts = true;
        col_req.group_name=group_;
        planning_scene_->checkCollision(col_req,col_res,goal_state);
        if (col_res.collision)
        {
          ROS_INFO_STREAM("End state: " << goal_configuration.transpose()<<" is colliding");
          for (const  std::pair<std::pair<std::string, std::string>, std::vector<collision_detection::Contact> >& contact: col_res.contacts)
          {
            ROS_INFO("contact between %s and %s",contact.first.first.c_str(),contact.first.second.c_str());
          }
        }
      }
      continue;
    }
    COMMENT("goal is valid");

    pathplan::NodePtr goal_node=std::make_shared<pathplan::Node>(goal_configuration);
    solver->addGoal(goal_node);
    at_least_a_goal=true;
  }



  if (!at_least_a_goal)
  {
    ROS_ERROR("All goals are in collision");
    res.error_code_.val=moveit_msgs::MoveItErrorCodes::GOAL_IN_COLLISION;
    is_running_=false;
    return false;
  }

  // ===============================
  // BEGINNING OF THE IMPORTANT PART
  // ===============================

  // searching initial solutions
  pathplan::PathPtr solution;
  bool found_a_solution=false;
  unsigned int iteration=0;
  ros::WallTime plot_time=ros::WallTime::now()-ros::WallDuration(100);
  double cost_of_first_solution;
  while((ros::WallTime::now()-start_time)<max_planning_time)
  {
    iteration++;
    if (stop_)
    {
      ROS_INFO("Externally stopped");
      res.error_code_.val=moveit_msgs::MoveItErrorCodes::PREEMPTED;
      is_running_=false;
      return false;
    }

    solver->update(solution);
    if (!found_a_solution && solver->solved())
    {
      assert(solution);
      ROS_INFO("Find a first solution (cost=%f) in %f seconds",solver->cost(),(ros::WallTime::now()-start_time).toSec());
      found_a_solution=true;
      cost_of_first_solution=solver->cost();
      ROS_INFO("path length = %f",solver->getSolution()->computeEuclideanNorm());
      refine_time = ros::WallTime::now();
    }
    if (solver->completed())
    {
      ROS_INFO("Optimization completed (cost=%f) in %f seconds (%u iterations)",solver->cost(),(ros::WallTime::now()-start_time).toSec(),iteration);
      break;
    }

    if (found_a_solution && ((ros::WallTime::now()-refine_time)>max_refining_time_))
    {
      ROS_INFO("Refine time expired (cost=%f) in %f seconds (%u iterations)",solver->cost(),(ros::WallTime::now()-start_time).toSec(),iteration);
      break;
    }

    if (found_a_solution)
    {
      ROS_DEBUG_THROTTLE(0.5,"solution improved from %f to %f",cost_of_first_solution,solution->cost());
      if ((ros::WallTime::now()-plot_time).toSec()>plot_interval_)
      {
        display->clearMarkers();
        display->displayTree(solver->getStartTree());
        display->displayPath(solution,"pathplan",{0,1,0,1});
        plot_time=ros::WallTime::now();
      }
    }
  }

  ROS_INFO_STREAM(*solver);

  if (!found_a_solution)
  {
    ROS_ERROR("unable to find a valid path");
    res.error_code_.val=moveit_msgs::MoveItErrorCodes::PLANNING_FAILED;
    is_running_=false;
    return false;
  }
  ROS_INFO("solution improved from %f to %f",cost_of_first_solution,solution->cost());
  ROS_INFO("path length = %f",solver->getSolution()->computeEuclideanNorm());

  if (!solver->completed())
  {
    ROS_INFO("Stopped (cost=%f) after %f seconds (%u iterations)",solver->cost(),(ros::WallTime::now()-start_time).toSec(),iteration);
  }

  // =========================
  // END OF THE IMPORTANT PART
  // =========================
  COMMENT("Get waypoints");
  std::vector<Eigen::VectorXd> waypoints=solution->getWaypoints();// PUT WAY POINTS
  robot_trajectory::RobotTrajectoryPtr trj(new robot_trajectory::RobotTrajectory(robot_model_,group_));

  COMMENT("processing %zu waypoints..", waypoints.size());
  for (const Eigen::VectorXd& waypoint: waypoints )
  {
    COMMENT("processing waypoint");
    moveit::core::RobotState wp_state=start_state;
    wp_state.setJointGroupPositions(group_,waypoint);
    wp_state.update();
    trj->addSuffixWayPoint(wp_state,0);
  }
  ros::WallDuration wd = ros::WallTime::now() - start_time;


  res.processing_time_.push_back(wd.toSec());
  res.description_.emplace_back("plan");
  res.trajectory_.push_back(trj);

  res.error_code_.val=moveit_msgs::MoveItErrorCodes::SUCCESS;
  is_running_=false;
  COMMENT("ok");


  return true;
}

bool TimeBasedMultiGoalPlanner::solve ( planning_interface::MotionPlanResponse& res )
{
  ros::WallTime start_time = ros::WallTime::now();
  planning_interface::MotionPlanDetailedResponse detailed_res;
  bool success = solve(detailed_res);
  if(success)
  {
    res.trajectory_ = detailed_res.trajectory_.at(0);
  }
  ros::WallDuration wd = ros::WallTime::now() - start_time;
  res.planning_time_ = wd.toSec();
  res.error_code_ = detailed_res.error_code_;

  return success;
}

bool TimeBasedMultiGoalPlanner::terminate()
{
  stop_=true;
  ros::Time t0=ros::Time::now();
  ros::Rate lp(50);
  while (ros::ok())
  {
    if (!is_running_)
      return true;
    lp.sleep();
    if ((ros::Time::now()-t0).toSec()>5)
    {
      ROS_ERROR("Unable to stop planner %s of group %s",name_.c_str(),group_.c_str());
      return false;
    }
  }
  return false;
}



}  // namespace dirrt_star
}  // namespace pathplan
