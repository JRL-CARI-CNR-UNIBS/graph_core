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
  group_(group),
  m_gen(time(0))
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
  // COMMENT("check planning scene");
  // if (!planning_scene_)
  //   ROS_ERROR("No planning scene available");

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

  m_ud = std::uniform_real_distribution<double>(0, 1);


}

void DIRRTStar::setPlanningScene ( const planning_scene::PlanningSceneConstPtr& planning_scene )
{
  planning_scene_=planning_scene;
  COMMENT("create checker");
  planning_scene::PlanningScenePtr ps=planning_scene::PlanningScene::clone(planning_scene);
  checker=std::make_shared<pathplan::MoveitCollisionChecker>(ps,group_,collision_distance);

}


void DIRRTStar::clear()
{

}


bool DIRRTStar::solve ( planning_interface::MotionPlanDetailedResponse& res )
{

  std::string dirrt_log;
  if (!m_nh.getParam("/dirrt_log",dirrt_log))
  {
    ROS_DEBUG("dirrt_log is not set, default: test");
    dirrt_log="test";
  }
  std::ofstream m_file;
  m_file.open(dirrt_log+".dirrt",std::ios::out | std::ios::binary);
  std::unique_ptr<char[]> m_buf;
  const size_t bufsize = 1024 * 1024;
  m_buf.reset(new char[bufsize]);
  m_file.rdbuf()->pubsetbuf(m_buf.get(), bufsize);

  m_max_planning_time=ros::WallDuration(request_.allowed_planning_time);
  ros::WallTime start_time = ros::WallTime::now();
  ros::WallTime refine_time = ros::WallTime::now();
  m_is_running=true;

  planning_scene::PlanningScenePtr ptr=planning_scene::PlanningScene::clone(planning_scene_);
  checker=std::make_shared<pathplan::MoveitCollisionChecker>(ptr,group_,collision_distance);


  std::vector<NodePtr> white_list;
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

    m_file.close();
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
    m_is_running=false;
    m_file.close();
    return false;
  }


  std::map<double,moveit_msgs::Constraints> ordered_goals;

  pathplan::NodePtr start_node=std::make_shared<pathplan::Node>(start_conf);
  white_list.push_back(start_node);
  std::vector<pathplan::NodePtr> goal_nodes;
  std::vector<pathplan::SamplerPtr> samplers;
  std::vector<pathplan::TubeInformedSamplerPtr> tube_samplers;
  std::vector<pathplan::TreeSolverPtr> solvers;
  std::vector<bool> found_solution;
  std::vector<pathplan::PathLocalOptimizerPtr> local_solvers;
  std::vector<double> local_probability;  // probability to improve using local sampling
   std::vector<bool> local_unoptimality;  // probability of the solution to be local unoptimal
  double cost_before_refining=std::numeric_limits<double>::infinity();
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

    if (!checker->check(final_configuration))
    {
      ROS_WARN("goal %u is in collision",iGoal);

      if (request_.goal_constraints.size()<5)
      {

        if (!end_state.satisfiesBounds())
        {
          ROS_ERROR_STREAM("End state: " << final_configuration.transpose()<<" is  Out of bound");
        }

        collision_detection::CollisionRequest col_req;
        collision_detection::CollisionResult col_res;
        col_req.contacts = true;
        col_req.group_name=group_;
        planning_scene_->checkCollision(col_req,col_res,end_state);
        if (col_res.collision)
        {
          ROS_ERROR_STREAM("End state: " << final_configuration.transpose()<<" is colliding");
          for (const  std::pair<std::pair<std::string, std::string>, std::vector<collision_detection::Contact> >& contact: col_res.contacts)
          {
            ROS_ERROR("contact between %s and %s",contact.first.first.c_str(),contact.first.second.c_str());
          }
        }
      }
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
    white_list.push_back(goal_node);

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

      local_probability.push_back(0.5);
      local_unoptimality.push_back(true);

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
            for (unsigned int ipoint=0;ipoint<preload_path.size();ipoint++)
            {
              if (preload_path.at(ipoint).size() == final_configuration.size())
              {
                for (unsigned iax=0;iax<final_configuration.size();iax++)
                  preload_point(iax)=preload_path.at(ipoint).at(iax);
                if ((preload_point-start_conf).norm()<1e-6)
                  continue;
                if ((preload_point-goal_node->getConfiguration()).norm()<1e-6)
                  continue;

                if (checker->checkPath(parent_node->getConfiguration(),preload_point))
                {
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
    else
      local_probability.push_back(0.0);

  }

  if (goal_nodes.size()==0)
  {
    ROS_INFO("All goals are in collision");
    res.error_code_.val=moveit_msgs::MoveItErrorCodes::GOAL_IN_COLLISION;
    m_is_running=false;
    m_file.close();
    return false;
  }

  // ===============================
  // BEGINNING OF THE IMPORTANT PART
  // ===============================

  // searching initial solutions
  pathplan::PathPtr solution;
  pathplan::PathPtr best_solution;
  unsigned int iter=0;
  double last_purge_cost=std::numeric_limits<double>::infinity();
  while((ros::WallTime::now()-start_time)<m_max_planning_time)
  {
    if (iter>200000)
      break;

    if (m_stop)
    {
      ROS_INFO("Externally stopped");
      res.error_code_.val=moveit_msgs::MoveItErrorCodes::PREEMPTED;
      m_is_running=false;
      m_file.close();
      return false;
    }
    if (found_a_solution && ((ros::WallTime::now()-refine_time)>m_max_refining_time))
    {
      break;
    }

    for (unsigned int isolver=0;isolver<solvers.size();isolver++)
    {
      pathplan::TreeSolverPtr solver= solvers.at(isolver);

      bool used_tube_sampler=false;
      if (m_tube_sampler && found_a_solution)
      {
        if (local_unoptimality.at(isolver))
        {
          used_tube_sampler=true;
          tube_samplers.at(isolver)->setLocalBias(1);
        }
        else
        {
          used_tube_sampler=false;
          tube_samplers.at(isolver)->setLocalBias(0);
        }
      }

      double current_cost=solver->cost();
      if (solver->update(solution))
      {
        bool improved=false;
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
          cost_before_refining=solution->cost();
          best_solution=solution;
          for (unsigned int isolver1=0;isolver1<solvers.size();isolver1++)
          {
            samplers.at(isolver1)->setCost(solution->cost());
            solvers.at(isolver1)->getStartTree()->purgeNodesOutsideEllipsoid(samplers.at(isolver1),white_list);
          }
        }
        else if (solution->cost()<best_solution->cost())
        {
          COMMENT("it improves the actual best solution. cost=%f",solution->cost());
          improved=true;
          best_solution=solution;
          double perc_improvement=(solution->cost()/std::max(1e-3,last_purge_cost));
          if (perc_improvement<.99)
          {
            last_purge_cost=solution->cost();
            for (unsigned int isolver1=0;isolver1<solvers.size();isolver1++)
            {
              samplers.at(isolver1)->setCost(solution->cost());
              solvers.at(isolver1)->getStartTree()->purgeNodesOutsideEllipsoid(samplers.at(isolver1),white_list);
            }
          }
        }

        if (m_tube_sampler)
        {
          double delta_cost=1-solver->cost()/current_cost;
          tube_samplers.at(isolver)->setPath(solution);
          if (used_tube_sampler)
            local_probability.at(isolver)=std::min(1.,m_forgetting_factor*local_probability.at(isolver)+10*(delta_cost));
        }
      }
      else
      {
        if (m_tube_sampler)
        {
          if (used_tube_sampler)
            local_probability.at(isolver)=std::max(1e-10,m_forgetting_factor*local_probability.at(isolver));
        }
      }


      if (m_tube_sampler)
      {
        double local_is_next= local_probability.at(isolver);
        if (local_unoptimality.at(isolver) && m_ud(m_gen)>(local_is_next))
          local_unoptimality.at(isolver)=false;
        else if (!local_unoptimality.at(isolver) && m_ud(m_gen)>(1-local_is_next))
          local_unoptimality.at(isolver)=true;
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

    ++iter;
    if (found_a_solution)
    {
      ROS_INFO_THROTTLE(1,"time = %f, iter = %u, nodes=%u, local prob =%f, best solution = %f, improvement = %e",(ros::WallTime::now()-start_time).toSec(),iter,solvers.at(0)->getStartTree()->getNumberOfNodes(),local_probability.at(0), best_solution->cost(), best_solution->cost()-cost_before_refining);
    }

    double time=(ros::WallTime::now()-start_time).toSec();
    m_file.write((char*) &time, sizeof(double));
    m_file.write((char*) &iter, sizeof(unsigned int));
    unsigned int nodes=solvers.at(0)->getStartTree()->getNumberOfNodes();
    m_file.write((char*) &nodes, sizeof(unsigned int));
    m_file.write((char*) &(local_probability.at(0)), sizeof(double));
    if (found_a_solution)
      m_file.write((char*) &(best_solution->cost()), sizeof(double));
    else
      m_file.write((char*) &(cost_before_refining), sizeof(double));

  }



  if (!found_a_solution)
  {
    ROS_ERROR("unable to find a valid path");
    res.error_code_.val=moveit_msgs::MoveItErrorCodes::PLANNING_FAILED;
    m_is_running=false;
    m_file.close();
    return false;
  }

  ROS_INFO("Refinement of the solution form %f to %f",cost_before_refining,best_solution->cost());


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

  m_file.close();
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
