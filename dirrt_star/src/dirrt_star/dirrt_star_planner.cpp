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


#include <dirrt_star/dirrt_star_planner.h>

namespace pathplan {
namespace dirrt_star {

DIRRTStar::DIRRTStar ( const std::string& name,
                       const std::string& group,
                       const moveit::core::RobotModelConstPtr& model ) :
  PlanningContext ( name, group ),
  group_(group)
{
  m_nh=ros::NodeHandle(name);
  COMMENT("create DIRRTStar, name =%s, group = %s", name.c_str(),group.c_str());
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
  m_dof=joint_names_.size();
  COMMENT("number of joints  = %u",m_dof);
  m_lb.resize(m_dof);
  m_ub.resize(m_dof);


  COMMENT("read bounds");
  for (unsigned int idx=0;idx<m_dof;idx++)
  {
    COMMENT("joint %s",joint_names_.at(idx).c_str());
    const robot_model::VariableBounds& bounds = robot_model_->getVariableBounds(joint_names_.at(idx));
    if (bounds.position_bounded_)
    {
      m_lb(idx)=bounds.min_position_;
      m_ub(idx)=bounds.max_position_;
    }
  }


  urdf::Model urdf_model;
  urdf_model.initParam("robot_description");

  COMMENT("create metrics");
  metrics=std::make_shared<pathplan::Metrics>();
  COMMENT("check planning scene");
  if (!planning_scene_)
    ROS_ERROR("No planning scene available");

  COMMENT("created DIRRTStar");

  double refining_time=0;
  if (!m_nh.getParam("max_refine_time",refining_time))
  {
    ROS_DEBUG("refining_time is not set, default=30");
    refining_time=30;
  }
  m_max_refining_time=ros::WallDuration(refining_time);
  m_max_planning_time=ros::WallDuration(60);


  if (!m_nh.getParam("path_optimization",m_path_optimization))
  {
    ROS_DEBUG("refining_time is not set, default: false");
    m_path_optimization=false;
  }
  if (!m_nh.getParam("tube_sampler",m_tube_sampler))
  {
    ROS_DEBUG("refining_time is not set, default: false");
    m_tube_sampler=false;
  }


}

void DIRRTStar::setPlanningScene ( const planning_scene::PlanningSceneConstPtr& planning_scene )
{
  planning_scene_=planning_scene;
  COMMENT("create checker");
  checker=std::make_shared<pathplan::MoveitCollisionChecker>(planning_scene_,group_,collision_distance);

}


bool DIRRTStar::canServiceRequest(const moveit_msgs::MotionPlanRequest &req) const
{
  if(req.group_name != getGroupName())
  {
    ROS_ERROR("Unsupported planning group '%s' requested", req.group_name.c_str());
    return false;
  }

  if (req.goal_constraints[0].joint_constraints.size() == 0)
  {
    ROS_ERROR("Can only handle joint space goals.");
    return false;
  }

  return true;
}

void DIRRTStar::clear()
{

}


bool DIRRTStar::solve ( planning_interface::MotionPlanDetailedResponse& res )
{


  m_max_planning_time=ros::WallDuration(request_.allowed_planning_time);
  ros::WallTime start_time = ros::WallTime::now();
  ros::WallTime refine_time = ros::WallTime::now();
  m_is_running=true;



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
    m_is_running=false;
    return false;
  }
  Eigen::VectorXd start_conf;
  start_state.copyJointGroupPositions(group_,start_conf);


  if (!checker->check(start_conf))
  {
    ROS_ERROR("Start point is in collision");

    start_state.update();
    start_state.updateCollisionBodyTransforms();

    if (planning_scene_->isStateColliding(start_state))
    {
      ROS_ERROR("Start state is colliding");
    }
    if (!planning_scene_->isStateValid(start_state))
    {
      ROS_ERROR("Start state is not valid");
    }
    collision_detection::CollisionRequest col_req;
    collision_detection::CollisionResult col_res;
    planning_scene_->checkCollision(col_req,col_res,start_state);
    if (col_res.collision)
    {
      ROS_ERROR("Start state is colliding");
    }

    col_req.contacts = true;

    ROS_FATAL("provo forzando la collision matrix");
    planning_scene_->checkCollision(col_req,col_res,start_state,planning_scene_->getAllowedCollisionMatrixNonConst());
    if (col_res.collision)
    {
      ROS_ERROR("Start state is colliding +++");
      for (const  std::pair<std::pair<std::string, std::string>, std::vector<collision_detection::Contact> >& contact: col_res.contacts)
      {
        ROS_ERROR("contact between %s and %s",contact.first.first.c_str(),contact.first.second.c_str());
      }
    }


    ROS_FATAL("cambio ordine");
    planning_scene_->checkCollision(col_req,col_res,start_state);
    if (col_res.collision)
    {
      ROS_ERROR("Start state is colliding +++");
      for (const  std::pair<std::pair<std::string, std::string>, std::vector<collision_detection::Contact> >& contact: col_res.contacts)
      {
        ROS_ERROR("contact between %s and %s",contact.first.first.c_str(),contact.first.second.c_str());
      }
    }



    col_req.group_name=group_;
    ROS_FATAL("provo forzando il gruppo");
    planning_scene_->checkCollision(col_req,col_res,start_state,planning_scene_->getAllowedCollisionMatrix());
    if (col_res.collision)
    {
      ROS_ERROR("Start state is colliding +++");
      for (const  std::pair<std::pair<std::string, std::string>, std::vector<collision_detection::Contact> >& contact: col_res.contacts)
      {
        ROS_ERROR("contact between %s and %s",contact.first.first.c_str(),contact.first.second.c_str());
      }
    }

    res.error_code_.val=moveit_msgs::MoveItErrorCodes::START_STATE_IN_COLLISION;
    m_is_running=false;
    return false;
  }


  std::map<double,moveit_msgs::Constraints> ordered_goals;

  pathplan::NodePtr start_node=std::make_shared<pathplan::Node>(start_conf);

  std::vector<pathplan::NodePtr> goal_nodes;
  std::vector<pathplan::SamplerPtr> samplers;
  std::vector<pathplan::TubeInformedSamplerPtr> tube_samplers;
  std::vector<pathplan::TreeSolverPtr> solvers;
  std::vector<bool> found_solution;
  std::vector<pathplan::PathLocalOptimizerPtr> local_solvers;

  bool found_a_solution=false;

  // computing minimum time
  for (unsigned int iGoal=0;iGoal<request_.goal_constraints.size();iGoal++)
  {
    moveit_msgs::Constraints goal=request_.goal_constraints.at(iGoal);
    ROS_INFO("Processing goal %u",iGoal++);

    Eigen::VectorXd final_configuration( goal.joint_constraints.size() );

    moveit::core::RobotState end_state(robot_model_);
    for (auto c: goal.joint_constraints)
    {
      end_state.setJointPositions(c.joint_name,&c.position);
    }
    end_state.copyJointGroupPositions(group_,final_configuration);

    end_state.updateCollisionBodyTransforms();
    COMMENT("check collision on goal %u",iGoal);
    if (!start_state.satisfiesBounds())
    {
      ROS_FATAL("goal %dt is  Out of bound",iGoal);
      continue;
    }
    if (!checker->check(final_configuration))
    {
      ROS_WARN("goal %u is in collision",iGoal);
      continue;
    }
    COMMENT("goal is valid");

    pathplan::TubeInformedSamplerPtr tube_sampler = std::make_shared<pathplan::TubeInformedSampler>(start_conf, final_configuration, m_lb, m_ub);
    if (m_tube_sampler)
      samplers.push_back(tube_sampler);
    else
      samplers.push_back(std::make_shared<pathplan::InformedSampler>(start_conf,final_configuration,m_lb,m_ub));

    tube_samplers.push_back(tube_sampler);
    pathplan::NodePtr goal_node=std::make_shared<pathplan::Node>(final_configuration);
    goal_nodes.push_back(goal_node);


    std::shared_ptr<pathplan::BiRRT> solver=std::make_shared<pathplan::BiRRT>(metrics,checker,samplers.back());
    solver->config(m_nh);;
    solver->addGoal(goal_node);
    solver->addStart(start_node);
    solvers.push_back(solver);
    found_solution.push_back(false);

    std::shared_ptr<pathplan::PathLocalOptimizer> path_solver=std::make_shared<pathplan::PathLocalOptimizer>(checker,metrics);
    path_solver->config(m_nh);
    local_solvers.push_back(path_solver);


    if (m_tube_sampler)
    {

      std::vector<std::vector<double>> preload_path;
      double radius;
      if (m_nh.getParam("/preload_radius",radius))
      {
        if (rosparam_utilities::getParamMatrix(m_nh,"/preload_path",preload_path))
        {

          pathplan::TreePtr tree=solver->getStartTree();
          pathplan::NodePtr parent_node=tree->getRoot();
          pathplan::PathPtr solution;
          std::vector<ConnectionPtr> connections;

          tube_sampler->setPath(preload_path);
          tube_sampler->setRadius(radius);
          tube_sampler->setLocalBias(0.5);

          if (!solver->solved())
          {
            Eigen::VectorXd preload_point(final_configuration.size());
            ROS_INFO_STREAM("start = " << start_conf.transpose());
            ROS_INFO_STREAM("root = " << parent_node->getConfiguration().transpose());
            ROS_INFO_STREAM("goal = " << final_configuration.transpose());
            for (unsigned int ipoint=0;ipoint<preload_path.size();ipoint++)
            {
              ROS_INFO("point %u of %zu",ipoint,preload_path.size());

              if (preload_path.at(ipoint).size() == final_configuration.size())
              {
                for (unsigned iax=0;iax<final_configuration.size();iax++)
                  preload_point(iax)=preload_path.at(ipoint).at(iax);
                if ((preload_point-start_conf).norm()<1e-6)
                  continue;
                ROS_INFO_STREAM("parent = " << parent_node->getConfiguration().transpose());
                ROS_INFO_STREAM("preload_point= " << preload_point.transpose());

                if (checker->checkPath(parent_node->getConfiguration(),preload_point))
                {
                  ROS_INFO("point %u can be connected to start",ipoint);


                  pathplan::NodePtr child_node=std::make_shared<pathplan::Node>(preload_point);
                  pathplan::ConnectionPtr conn=std::make_shared<pathplan::Connection>(parent_node,child_node);
                  conn->setCost(metrics->cost(parent_node,child_node));
                  conn->add();
                  connections.push_back(conn);
                  parent_node=child_node;

                }
                else
                {
                  ROS_INFO("preload path is in collision");
                  break;
                }

              }
              else
              {
                ROS_WARN("preload point has wrong dimensions");
              }
            }


            if (checker->checkPath(parent_node->getConfiguration(),goal_node->getConfiguration()))
            {
              pathplan::ConnectionPtr conn=std::make_shared<pathplan::Connection>(parent_node,goal_node);
              conn->setCost(metrics->cost(parent_node,goal_node));
              conn->add();
              connections.push_back(conn);
              solution = std::make_shared<pathplan::Path>(connections, metrics, checker);
              solution->setTree(tree);
              solver->setSolution(solution,true);
              ROS_INFO("preload path is feasible!");
              break;
            }

          }
        }
      }
    }
  }

  if (goal_nodes.size()==0)
  {
    ROS_INFO("All goals are in collision");
    res.error_code_.val=moveit_msgs::MoveItErrorCodes::GOAL_IN_COLLISION;
    m_is_running=false;
    return false;
  }

  // ===============================
  // BEGINNING OF THE IMPORTANT PART
  // ===============================

  // searching initial solutions
  pathplan::PathPtr solution;

  pathplan::PathPtr best_solution;
  while((ros::WallTime::now()-start_time)<m_max_planning_time)
  {
    if (m_stop)
    {
      ROS_INFO("Externally stopped");
      res.error_code_.val=moveit_msgs::MoveItErrorCodes::PREEMPTED;
      m_is_running=false;
      return false;
    }
    if (found_a_solution && ((ros::WallTime::now()-refine_time)>m_max_refining_time))
    {
      break;
    }

    for (unsigned int isolver=0;isolver<solvers.size();isolver++)
    {
      pathplan::TreeSolverPtr solver= solvers.at(isolver);
      ROS_INFO_THROTTLE(1,"number of nodes %u",solver->getStartTree()->getNumberOfNodes());
      if (solver->update(solution))
      {
        ROS_INFO_THROTTLE(1,"trovata una soluzione");

        bool improved=false;
        COMMENT("Find a solution");
        if (!found_solution.at(isolver))
        {
          std::shared_ptr<pathplan::RRTStar> opt_solver=std::make_shared<pathplan::RRTStar>(metrics,checker,samplers.at(isolver));
          opt_solver->addStartTree(solver->getStartTree());
          opt_solver->addGoal(goal_nodes.at(isolver));
          opt_solver->config(m_nh);
          solvers.at(isolver)=opt_solver;
          found_solution.at(isolver)=true;
        }

        if (!found_a_solution)
        {
          COMMENT("it is the first one. cost=%f",solution->cost());
          improved=true;
          found_a_solution=true;
          refine_time = ros::WallTime::now();
        }
        else if (solution->cost()<best_solution->cost())
        {
          COMMENT("it improves the actual best solution. cost=%f",solution->cost());
          improved=true;
        }
        if (m_tube_sampler)
          local_solvers.at(isolver)->setPath(solution);
        if (improved)
        {
          best_solution=solution;
          for (pathplan::SamplerPtr& sampler: samplers)
            sampler->setCost(solution->cost());
        }
      }
    }

    if (m_path_optimization)
    {
      for (unsigned int isolver=0;isolver<solvers.size();isolver++)
      {
        std::shared_ptr<pathplan::PathLocalOptimizer> path_solver=local_solvers.at(isolver);
        ros::Time t1=ros::Time::now();
        if (path_solver->step(solution))
        {
          COMMENT("Improved solution %u, cost=%f",isolver,solution->cost());
          solvers.at(isolver)->setSolution(solution);
          if (m_tube_sampler)
            tube_samplers.at(isolver)->setPath(solution);

          if (solution->cost()<=best_solution->cost())
          {
            COMMENT("it improves the actual best solution");
            best_solution=solution;
            for (pathplan::SamplerPtr& sampler: samplers)
              sampler->setCost(solution->cost());
          }

        }
        COMMENT("path solver needs %f seconds",(ros::Time::now()-t1).toSec());
      }
    }



  }



  if (!found_a_solution)
  {
    ROS_ERROR("unable to find a valid path");
    res.error_code_.val=moveit_msgs::MoveItErrorCodes::PLANNING_FAILED;
    m_is_running=false;
    return false;
  }


  // =========================
  // END OF THE IMPORTANT PART
  // =========================
  COMMENT("Get waypoints");
  std::vector<Eigen::VectorXd> waypoints=best_solution->getWaypoints();// PUT WAY POINTS
  robot_trajectory::RobotTrajectoryPtr trj(new robot_trajectory::RobotTrajectory(robot_model_,group_));

  COMMENT("processing %zu aypoints..", waypoints.size());
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
  m_is_running=false;
  COMMENT("ok");
  return true;
}

bool DIRRTStar::solve ( planning_interface::MotionPlanResponse& res )
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

bool DIRRTStar::terminate()
{
  m_stop=true;
  ros::Time t0=ros::Time::now();
  ros::Rate lp(50);
  while (ros::ok())
  {
    if (!m_is_running)
      return true;
    lp.sleep();
    if ((ros::Time::now()-t0).toSec()>5)
    {
      ROS_ERROR("Unable to stop planner %s of group %s",name_.c_str(),group_.c_str());
      return false;
    }
  }
}

}
}
