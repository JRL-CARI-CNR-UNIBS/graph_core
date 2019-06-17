#ifndef dgaco_planner_20190501
#define dgaco_planner_20190501

#include <moveit/planning_interface/planning_interface.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/planning_scene/planning_scene.h>
#include <net_path_core/net.h>

namespace ha_planner
{
  
class DgacoPlanner: public planning_interface::PlanningContext
{
public:
  DgacoPlanner ( const std::string& name,
                const std::string& group,
                const moveit::core::RobotModelConstPtr& model
              );
  
  bool canServiceRequest(const moveit_msgs::MotionPlanRequest &req) const;
  
  void setPlanningScene(const planning_scene::PlanningSceneConstPtr& planning_scene);
  virtual bool solve(planning_interface::MotionPlanResponse& res) override;
  
  virtual bool solve(planning_interface::MotionPlanDetailedResponse& res) override;
  
  /** \brief If solve() is running, terminate the computation. Return false if termination not possible. No-op if
   * solve() is not running (returns true).*/
  virtual bool terminate() override;
  
  /** \brief Clear the data structures used by the planner */
  virtual void clear() override;
protected:
  moveit::core::RobotModelConstPtr robot_model_;
  //planning_scene::PlanningSceneConstPtr pl
  ros::NodeHandle m_nh;

  bool m_stop=false;
  std::shared_ptr<Net> m_net;
  unsigned int m_dof;
  std::vector<std::string> joint_names_;
  std::vector<double> m_scaling;
  std::vector<double> m_lb;
  std::vector<double> m_ub;

  int m_max_stall_rrt;
  int trials=2000;
  int number_of_nodes=300;
  int n_ants=80;
  int max_stall_gen=50;
  bool refinement=true;
  double rrt_time=0.8;
  double max_time=1.5;

};

}














#endif
