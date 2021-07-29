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

#include <dirrt_star/dirrt_star_planner_manager.h>

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(pathplan::dirrt_star::PathPlanerManager, planning_interface::PlannerManager)

namespace pathplan {
namespace dirrt_star {

bool PathPlanerManager::initialize(const moveit::core::RobotModelConstPtr& model, const std::string& ns)
{
  COMMENT("load manager");

  std::map<std::string,std::string> planner_map;
  m_nh=ros::NodeHandle(ns);


  if (!m_nh.getParam("default_planner_config",m_default_planner_config))
  {
    ROS_DEBUG("default planning config not set");
    m_default_planner_config="";
  }

  m_nh.getParam("group_names_map",planner_map);
  for (std::pair<std::string,std::string> p: planner_map)
  {
    COMMENT("Group name = %s, planner name: %s",p.second.c_str(),p.first.c_str());

    std::string type;
    if (!m_nh.getParam(ns+"/"+p.first+"/type",type))
    {
      ROS_WARN_STREAM(ns+"/"+p.first+"/type is not set, skip this planner");
      continue;
    }

    std::shared_ptr<planning_interface::PlanningContext> ptr;
    if (!type.compare("DIRRT"))
    {
      ROS_ERROR("DIRRT has been replaced by Multigoal");
      //ptr= std::make_shared<DIRRTStar>(ns+"/"+p.first,p.second,model);
    }
    else if (!type.compare("Multigoal"))
    {
      ptr= std::make_shared<MultigoalPlanner>(ns+"/"+p.first,p.second,model);
    }
    else if (!type.compare("TimeBasedMultigoal"))
    {
      ptr= std::make_shared<TimeBasedMultiGoalPlanner>(ns+"/"+p.first,p.second,model);
    }
    else if (!type.compare("HAMPTimeBasedMultigoal"))
    {
      ptr= std::make_shared<HAMPTimeBasedMultiGoalPlanner>(ns+"/"+p.first,p.second,model);
    }
    else if (!type.compare("ProbabilisticHAMPTimeBasedMultiGoalPlanner"))
    {
      ptr= std::make_shared<ProbabilisticHAMPTimeBasedMultiGoalPlanner>(ns+"/"+p.first,p.second,model);
    }
    else
    {
      ROS_WARN_STREAM(ns+"/"+p.first+"/type is '"<<type<<"'. Available ones are: Multigoal, TimeBasedMultigoal, HAMPTimeBasedMultigoal, ProbabilisticHAMPTimeBasedMultiGoalPlanner. Skip this planner");
      continue;
    }

    ROS_INFO("Planned Id=%s on group %s",p.first.c_str(),p.second.c_str());
    m_planners.insert(std::pair<std::string, std::shared_ptr<planning_interface::PlanningContext>>(p.first,ptr));
  }
  return true;
}

bool PathPlanerManager::canServiceRequest(const moveit_msgs::MotionPlanRequest& req) const
{
  bool ok=false;
  for (std::pair<std::string, std::shared_ptr<planning_interface::PlanningContext>> planner: m_planners)
  {
    if (!planner.second->getGroupName().compare(req.group_name))
    {
      ROS_INFO("Planned Id=%s can be used for group %s",planner.first.c_str(),req.group_name.c_str());
      ok=true;
    }
  }
  return ok;
}


planning_interface::PlanningContextPtr PathPlanerManager::getPlanningContext(
  const planning_scene::PlanningSceneConstPtr &planning_scene,
  const planning_interface::MotionPlanRequest &req,
  moveit_msgs::MoveItErrorCodes &error_code) const
{
  bool any_planner=false;
  std::string planner_id;
  if (req.planner_id.size()==0)
  {
    if (m_default_planner_config.size()==0)
    {
      ROS_DEBUG("Search a planner for group %s",req.group_name.c_str());
      any_planner=true;
    }
    else
    {
      ROS_DEBUG("Use default planner %s",m_default_planner_config.c_str());
      planner_id=m_default_planner_config;
    }
  }
  else
  {
    ROS_DEBUG("Search planner %s for group %s",req.planner_id.c_str(),req.group_name.c_str());
    planner_id=req.planner_id;
  }

  bool ok=false;
  for (std::pair<std::string, std::shared_ptr<planning_interface::PlanningContext>> planner: m_planners)
  {
    ROS_DEBUG("planner %s for group %s",planner.first.c_str(),planner.second->getGroupName().c_str());
    if (!planner.second->getGroupName().compare(req.group_name))
    {
      if (any_planner)
      {
        planner_id=planner.first;
        ok=true;
        break;
      }
      else if (!planner.first.compare(planner_id))
      {
        ROS_DEBUG("Planned Id=%s can be used for group %s",planner.first.c_str(),req.group_name.c_str());
        ok=true;
      }
    }
  }
  if (!ok)
  {
    ROS_ERROR("Planner %s not found for group %s.", planner_id.c_str(), req.group_name.c_str());
    ROS_ERROR("Available planners are:\n");
    for (std::pair<std::string, std::shared_ptr<planning_interface::PlanningContext>> planner: m_planners)
      ROS_ERROR("- planner %s, group  %s",planner.first.c_str(),planner.second->getGroupName().c_str());
    return nullptr;
  }

//  req.planner_id

  std::shared_ptr<planning_interface::PlanningContext> planner = m_planners.at(planner_id);
  if (!planner)
  {
    ROS_ERROR("Planner not found");
    return planner;
  }
  else
  {
    ROS_INFO("Using  planner %s for planning on the group %s",planner->getName().c_str(),req.group_name.c_str());
  }

  planner->setPlanningScene(planning_scene);
  planner->setMotionPlanRequest(req);
  return planner;
}


void PathPlanerManager::getPlanningAlgorithms ( std::vector< std::string >& algs ) const
{
  for (std::pair<std::string, std::shared_ptr<planning_interface::PlanningContext>> planner: m_planners)
    algs.push_back(planner.first);
}

void PathPlanerManager::setPlannerConfigurations ( const planning_interface::PlannerConfigurationMap& pcs )
{
  planning_interface::PlannerManager::setPlannerConfigurations ( pcs );
}



}
}
