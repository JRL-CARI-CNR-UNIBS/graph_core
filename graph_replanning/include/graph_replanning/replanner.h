#ifndef REPLANNER_H__
#define REPLANNER_H__
#include <eigen3/Eigen/Core>
#include <ros/ros.h>
#include <graph_core/util.h>
#include <graph_core/graph/graph_display.h>
#include <graph_core/graph/tree.h>
#include <graph_core/graph/path.h>
#include <graph_core/graph/connection.h>
#include <graph_core/graph/node.h>
#include <graph_core/metrics.h>
#include <graph_core/solvers/tree_solver.h>
#include <graph_core/solvers/path_solver.h>
#include <graph_core/collision_checker.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <rviz_visual_tools/rviz_visual_tools.h>
#include <moveit/robot_model/robot_model.h>

namespace pathplan
{
class Replanner;
typedef std::shared_ptr<Replanner> ReplannerPtr;

class Replanner: public std::enable_shared_from_this<Replanner>
{
protected:
  Eigen::VectorXd current_configuration_;        // current robot conf
  PathPtr current_path_;                         // current path traveled by the robot
  PathPtr replanned_path_;                        // replanned path
  std::vector<PathPtr> replanned_paths_vector_;   // vector of the 10 best replanned paths
  std::vector<PathPtr> other_paths_;             // initial available paths
  std::vector<PathPtr> admissible_other_paths_;  // available paths
  std::vector<NodePtr> examined_nodes_;          // node considered during the replanning
  TreeSolverPtr solver_;                         // solver
  MetricsPtr metrics_;
  CollisionCheckerPtr checker_;
  Eigen::VectorXd lb_;
  Eigen::VectorXd ub_;
  double time_first_sol_;
  double time_replanning_;
  double available_time_;
  bool success_;
  double pathSwitch_cycle_time_mean_;
  double informedOnlineReplanning_cycle_time_mean_;
  bool an_obstacle_;
  bool emergency_stop_;

  bool informedOnlineReplanning_disp_;
  bool pathSwitch_disp_;
  DisplayPtr disp_;

  bool informedOnlineReplanning_verbose_;
  bool pathSwitch_verbose_;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Replanner(Eigen::VectorXd& current_configuration,
            PathPtr& current_path,
            std::vector<PathPtr>& other_paths,
            const TreeSolverPtr& solver,
            //const BiRRTPtr solver,
            const MetricsPtr& metrics,
            const CollisionCheckerPtr& checker,
            const Eigen::VectorXd& lb,
            const Eigen::VectorXd& ub);

  PathPtr getReplannedPath()
  {
    return replanned_path_;
  }

  std::vector<PathPtr> getReplannedPathVector()
  {
    return replanned_paths_vector_;
  }

  PathPtr getCurrentPath()
  {
    return current_path_;
  }

  std::vector<PathPtr> getOtherPaths()
  {
    return other_paths_;
  }

  void setInformedOnlineReplanningVerbose()
  {
    informedOnlineReplanning_verbose_ = true;
  }

  void setPathSwitchVerbose()
  {
    pathSwitch_verbose_ = true;
  }

  void resetInformedOnlineReplanningVerbose()
  {
    informedOnlineReplanning_verbose_ = false;
  }

  void resetPathSwitchVerbose()
  {
    pathSwitch_verbose_ = false;
  }

  void setInformedOnlineReplanningDisp(const DisplayPtr &disp)
  {
    if(!disp) throw std::invalid_argument("Display not initialized");

    disp_ = disp;
    informedOnlineReplanning_disp_ = true;
  }

  void setPathSwitchDisp(const DisplayPtr &disp)
  {
    if(!disp) throw std::invalid_argument("Display not initialized");

    disp_ = disp;
    pathSwitch_disp_ = true;
  }

  void resetInformedOnlineReplanningDisp()
  {
    disp_ = NULL;
    informedOnlineReplanning_disp_ = false;
  }

  void resetPathSwitchDisp()
  {
    disp_ = NULL;
    pathSwitch_disp_ = false;
  }

  void setEmergencyStop()
  {
    emergency_stop_ = true;
  }

  void resetEmergencyStop()
  {
    emergency_stop_ = false;
  }


  void setCurrentPath(const PathPtr& path)
  {
    current_path_ = path;
    admissible_other_paths_ = other_paths_;
    examined_nodes_.clear();
    success_ = 0;
  }

  void setOtherPaths(const std::vector<PathPtr> &other_paths)
  {
    other_paths_ = other_paths;
    admissible_other_paths_ = other_paths_;
    examined_nodes_.clear();
    success_ = 0;
  }

  void setCurrentConf(const Eigen::VectorXd& q)
  {
    current_configuration_ = q;
    solver_->resetProblem();
    solver_->addStart(std::make_shared<Node>(q));
    examined_nodes_.clear();
    success_ = 0;
  }

  Eigen::VectorXd getCurrentConf()
  {
    return current_configuration_;
  }

  MetricsPtr getMetrics()
  {
    return metrics_;
  }
  CollisionCheckerPtr getChecker()
  {
    return checker_;
  }

  void setChecker(const CollisionCheckerPtr &checker)
  {
    checker_ = checker;
  }

  void addOtherPath(const PathPtr& path)
  {
    other_paths_.push_back(path);
  }

  ReplannerPtr pointer()
  {
    return shared_from_this();
  }

  bool getSuccess()
  {
    return success_;
  }

  bool simplifyReplannedPath(const double& distance);

  bool checkPathValidity(const CollisionCheckerPtr &this_checker = NULL);

  void startReplannedPathFromNewCurrentConf(Eigen::VectorXd &configuration);

  //It find the portion of current_path_ between the obstacle and the goal and add it as first element of a vector containing the other available paths
  std::vector<PathPtr> addAdmissibleCurrentPath(const int &idx_current_conn, PathPtr& admissible_current_path);

  //It directly connect the node to the goal
  bool connect2goal(const PathPtr &current_path, const NodePtr& node, PathPtr &new_path);

  //Starting from node of current_path_ it tries to find a connection to all the available paths of admissible_other_paths_
  bool pathSwitch(const PathPtr& current_path, const NodePtr& node, const bool& succ_node, PathPtr &new_path, PathPtr &subpath_from_path2, int &connected2path_number);

  //It menages the replanning calling more times pathSwitch from different nodes and giving the correct set of available paths
  bool informedOnlineReplanning(const int& informed, const bool& succ_node, const double &max_time  = std::numeric_limits<double>::infinity());

};
}

#endif // REPLANNER_H
