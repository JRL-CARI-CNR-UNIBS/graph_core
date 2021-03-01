#ifndef REPLANNER_MANAGER_H__
#define REPLANNER_MANAGER_H__

#include <graph_replanning/trajectory.h>
#include <moveit/robot_state/robot_state.h>
#include <graph_replanning/replanner.h>
#include <moveit_planning_helper/spline_interpolator.h>
#include <object_loader_msgs/addObjects.h>
#include <object_loader_msgs/removeObjects.h>
#include <thread>
#include <mutex>
#include <std_msgs/Float64.h>
#include <std_srvs/Empty.h>

namespace pathplan
{
class ReplannerManager;
typedef std::shared_ptr<ReplannerManager> ReplannerManagerPtr;

class ReplannerManager: public std::enable_shared_from_this<ReplannerManager>
{
protected:

  // To be assigned by the constructor
  PathPtr current_path_;
  std::vector<PathPtr> other_paths_;
  double trj_execution_thread_frequency_;
  double collision_checker_thread_frequency_;
  double dt_replan_restricted_;
  double dt_replan_relaxed_;
  std::string group_name_;
  std::string base_link_;
  std::string last_link_;
  ros::NodeHandle nh_;

  // Global variables
  bool stop_;
  bool first_replan_;
  bool path_obstructed_;
  bool computing_avoiding_path_;
  int n_conn_;
  double real_time_;
  double t_;
  double dt_;
  double replan_offset_;
  double t_replan_;
  double replanning_thread_frequency_;

  ReplannerPtr replanner_;
  Eigen::VectorXd current_configuration_;
  Eigen::VectorXd configuration_replan_;
  CollisionCheckerPtr checker_thread_cc_;
  CollisionCheckerPtr checker_;
  TrajectoryPtr trajectory_;
  planning_scene::PlanningScenePtr planning_scn_;
  planning_scene::PlanningScenePtr planning_scn_replanning_;
  trajectory_processing::SplineInterpolator interpolator_;
  trajectory_msgs::JointTrajectoryPoint pnt_;
  trajectory_msgs::JointTrajectoryPoint pnt_replan_;
  sensor_msgs::JointState joint_state_;

  std::mutex planning_mtx_;
  std::mutex trj_mtx_;
  std::mutex checker_mtx_;
  std::mutex scene_mtx_;
  std::mutex replanner_mtx_;

  ros::Publisher current_norm_pub_;
  ros::Publisher new_norm_pub_;
  ros::Publisher time_replanning_pub_;
  ros::Publisher obs_current_norm_pub_;
  ros::Publisher obs_new_norm_pub_;
  ros::Publisher obs_time_replanning_pub_;
  ros::Publisher target_pub_;
  ros::Publisher time_pub_;
  ros::ServiceClient plannning_scene_client_;
  ros::ServiceClient add_obj_;
  ros::ServiceClient remove_obj_;
  ros::ServiceClient start_log_;
  ros::ServiceClient stop_log_;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ReplannerManager(PathPtr &current_path,
                   std::vector<PathPtr> &other_paths,
                   const double &trj_execution_thread_frequency,
                   const double &collision_checker_thread_frequency,
                   const double &dt_replan_restricted,
                   const double &dt_replan_relaxed,
                   const std::string &group_name,
                   const std::string &base_link,
                   const std::string &last_link,
                   ros::NodeHandle &nh);

  void attributeInitialization();
  void subscribeTopicsAndServices();
  void replanningThread();
  void collisionCheckThread();
  void displayThread();
  void spawnObjects();
  int trajectoryExecutionThread();
};

}

#endif // REPLANNER_MANAGER_H
