#ifndef dgaco_planner_manager_20190501
#define dgaco_planner_manager_20190501

#include <moveit/planning_interface/planning_interface.h>
#include <dgaco_planner/dgaco_planner.h>
namespace ha_planner
{
  
class DgacoPlannerManager : public planning_interface::PlannerManager
{
public:
  virtual bool initialize(const robot_model::RobotModelConstPtr& model, const std::string& ns) override;
  std::string getDescription() const override
  {
    return "Dgaco";
  }
  bool canServiceRequest(const moveit_msgs::MotionPlanRequest &req) const override;
  

                                                
  void getPlanningAlgorithms(std::vector<std::string> &algs) const override;
  
  
  void setPlannerConfigurations(const planning_interface::PlannerConfigurationMap &pcs) override;
  
  planning_interface::PlanningContextPtr getPlanningContext(
    const planning_scene::PlanningSceneConstPtr &planning_scene,
    const planning_interface::MotionPlanRequest &req,
    moveit_msgs::MoveItErrorCodes &error_code) const override;
    
    
    
protected:
  ros::NodeHandle m_nh;
  std::map< std::string, std::shared_ptr<DgacoPlanner>> m_planners;
  moveit::core::RobotModelConstPtr m_robot_model;
};

// 

}














#endif
