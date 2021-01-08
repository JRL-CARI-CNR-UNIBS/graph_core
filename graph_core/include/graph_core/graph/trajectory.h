#pragma once
#ifndef TRAJECTORY_8_1_2021_H
#define TRAJECTORY_8_1_2021_H

#include <ros/ros.h>
#include <graph_core/metrics.h>
#include <graph_core/occupancy_metrics.h>
#include <graph_core/graph/node.h>
#include <graph_core/graph/path.h>
#include <graph_core/graph/tree.h>
#include <graph_core/solvers/rrt_connect.h>
#include <graph_core/solvers/birrt.h>
#include <graph_core/solvers/rrt_star.h>
#include <graph_core/solvers/path_solver.h>
#include <graph_core/moveit_collision_checker.h>
#include <graph_core/local_informed_sampler.h>
#include <graph_core/test_util.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <rviz_visual_tools/rviz_visual_tools.h>
#include <eigen_conversions/eigen_msg.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>

#define COMMENT(...) ROS_LOG(::ros::console::levels::Debug, ROSCONSOLE_DEFAULT_NAME, __VA_ARGS__);

namespace pathplan
{
class Trajectory;
typedef std::shared_ptr<Trajectory> TrajectoryPtr;

class Trajectory: public std::enable_shared_from_this<Trajectory>
{
protected:

  robot_trajectory::RobotTrajectoryPtr trj_;
  PathPtr path_;
  ros::NodeHandle nh_;
  robot_model::RobotModelConstPtr kinematic_model_;
  planning_scene::PlanningScenePtr planning_scene_;
  std::string group_name_;
  std::string base_link_;
  std::string last_link_;
  ros::Publisher display_publisher_;
  ros::Publisher marker_pub_;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Trajectory(const pathplan::PathPtr path,
                         const ros::NodeHandle& nh,
                         const planning_scene::PlanningScenePtr& planning_scene,
                         const std::string& group_name,
                         const std::string& base_link,
                         const std::string& last_link);

  TrajectoryPtr pointer()
  {
    return shared_from_this();
  }

  void setPath(const pathplan::PathPtr path)
  {
    path_ = path;
  }

  pathplan::PathPtr getPath()
  {
    return path_;
  }

  robot_trajectory::RobotTrajectoryPtr getTrj()
  {
    if(trj_ == NULL) ROS_ERROR("Trj not computed");
    return trj_;
  }

  // Compute a path suing BiRRT from start_node to goal_node and then it is optimized if optimizePath==1 . nh is the NodeHandler of a ros node.
  PathPtr computeBiRRTPath(const NodePtr &start_node, NodePtr &goal_node, const Eigen::VectorXd& lb, const Eigen::VectorXd& ub, const MetricsPtr& metrics, const CollisionCheckerPtr& checker, const bool& optimizePath);

  //Transform a set of waypoints in a set of robotstate
  std::vector<moveit::core::RobotState> fromWaypoints2State(const std::vector<Eigen::VectorXd> waypoints);

  moveit::core::RobotState fromWaypoints2State(Eigen::VectorXd waypoint);

  //To trasform a path to a RobotTrajectory
  robot_trajectory::RobotTrajectoryPtr fromPath2Trj();

  //To trasform a path to a RobotTrajectory
  robot_trajectory::RobotTrajectoryPtr fromPath2Trj(const trajectory_msgs::JointTrajectoryPoint& pnt);

  //To display the execution of the trj on Rviz
  void displayTrj();

};
}

#endif // TRAJECTORY_H
