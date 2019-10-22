#include <dgaco_planner/dgaco_planner_manager.h>
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(ha_planner::DgacoPlannerManager, planning_interface::PlannerManager);

namespace ha_planner
{
  
bool DgacoPlannerManager::initialize(const moveit::core::RobotModelConstPtr& model, const std::string& ns)
{
  std::map<std::string,std::string> planner_map;
  m_nh=ros::NodeHandle(ns);
  m_nh.getParam("group_names_map",planner_map);
  for (std::pair<std::string,std::string> p: planner_map)
  {
    ROS_FATAL("Group name = %s, planner name: %s",p.second.c_str(),p.first.c_str());
    std::shared_ptr<DgacoPlanner> ptr(new DgacoPlanner(ns+"/"+p.first,p.second,model));
    m_planners.insert(std::pair<std::string,std::shared_ptr<ha_planner::DgacoPlanner>>(p.second,ptr));

  }
  return true;
}

bool DgacoPlannerManager::canServiceRequest(const moveit_msgs::MotionPlanRequest& req) const
{
  if(m_planners.count(req.group_name) == 0)
  {
    return false;
  }
  // Get planner
  std::shared_ptr<DgacoPlanner> planner = m_planners.at(req.group_name);
  return planner->canServiceRequest(req);
}


planning_interface::PlanningContextPtr DgacoPlannerManager::getPlanningContext(
  const planning_scene::PlanningSceneConstPtr &planning_scene,
  const planning_interface::MotionPlanRequest &req,
  moveit_msgs::MoveItErrorCodes &error_code) const
{
  ROS_DEBUG("Search a planner for group %s",req.group_name.c_str());
  std::shared_ptr<DgacoPlanner> planner = m_planners.at(req.group_name);
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


void DgacoPlannerManager::getPlanningAlgorithms ( std::vector< std::string >& algs ) const
{
  planning_interface::PlannerManager::getPlanningAlgorithms ( algs );
}

void DgacoPlannerManager::setPlannerConfigurations ( const planning_interface::PlannerConfigurationMap& pcs )
{
  planning_interface::PlannerManager::setPlannerConfigurations ( pcs );
}



}

