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
    else
    {
      ROS_WARN_STREAM(ns+"/"+p.first+"/type is '"<<type<<"'. Available ones are: Multigoal. Skip this planner");
      continue;
    }
    m_planners.insert(std::pair<std::string, std::shared_ptr<planning_interface::PlanningContext>>(p.second,ptr));
  }
  return true;
}

bool PathPlanerManager::canServiceRequest(const moveit_msgs::MotionPlanRequest& req) const
{
  if(m_planners.count(req.group_name) == 0)
  {
    return false;
  }
  // Get planner
  return m_planners.find(req.group_name)!=m_planners.end();
}


planning_interface::PlanningContextPtr PathPlanerManager::getPlanningContext(
  const planning_scene::PlanningSceneConstPtr &planning_scene,
  const planning_interface::MotionPlanRequest &req,
  moveit_msgs::MoveItErrorCodes &error_code) const
{
  ROS_DEBUG("Search a planner for group %s",req.group_name.c_str());

  if (m_planners.find(req.group_name) == m_planners.end())
  {
    ROS_ERROR("Planner not found for group %s.", req.group_name.c_str());
    return nullptr;
  }
  std::shared_ptr<planning_interface::PlanningContext> planner = m_planners.at(req.group_name);
  if (!planner)
  {
    ROS_ERROR("Planner not found");
    return planner;
  }
  else
  {
    ROS_DEBUG("founded planner %s",planner->getName().c_str());
  }

  planner->setPlanningScene(planning_scene);
  planner->setMotionPlanRequest(req);
  return planner;
}


void PathPlanerManager::getPlanningAlgorithms ( std::vector< std::string >& algs ) const
{
  planning_interface::PlannerManager::getPlanningAlgorithms ( algs );
}

void PathPlanerManager::setPlannerConfigurations ( const planning_interface::PlannerConfigurationMap& pcs )
{
  planning_interface::PlannerManager::setPlannerConfigurations ( pcs );
}



}
}
