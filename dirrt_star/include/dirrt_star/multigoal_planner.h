#pragma once
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


#include <moveit/planning_interface/planning_interface.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/planning_scene/planning_scene.h>
#include <graph_core/solvers/multigoal.h>
#include <graph_core/solvers/path_solver.h>
#include <graph_core/metrics.h>
#include <graph_core/avoidance_goal_cost_function.h>
#include <graph_core/avoidance_metrics.h>
#include <graph_core/parallel_moveit_collision_checker.h>
#include <graph_core/tube_informed_sampler.h>
#include <rosparam_utilities/rosparam_utilities.h>
#include <geometry_msgs/PoseArray.h>
#include <ros/callback_queue.h>

#include <graph_core/graph/graph_display.h>

#define COMMENT(...) ROS_LOG(::ros::console::levels::Debug, ROSCONSOLE_DEFAULT_NAME, __VA_ARGS__)


#include <fstream>
#include <iostream>


namespace pathplan {
namespace dirrt_star {

class MultigoalPlanner: public planning_interface::PlanningContext
{
public:
  MultigoalPlanner ( const std::string& name,
                const std::string& group,
                const moveit::core::RobotModelConstPtr& model
              );


  virtual bool solve(planning_interface::MotionPlanResponse& res) override;
  virtual bool solve(planning_interface::MotionPlanDetailedResponse& res) override;

  /** \brief If solve() is running, terminate the computation. Return false if termination not possible. No-op if
   * solve() is not running (returns true).*/
  virtual bool terminate() override;

  /** \brief Clear the data structures used by the planner */
  virtual void clear() override;

  void centroidCb(const geometry_msgs::PoseArrayConstPtr& msg);

protected:
  moveit::core::RobotModelConstPtr robot_model_;
  //planning_scene::PlanningSceneConstPtr pl
  ros::NodeHandle m_nh;
  std::shared_ptr<pathplan::Display> display;

  ros::WallDuration m_max_refining_time;
  ros::CallbackQueue m_queue;

  bool use_avoidance_goal_=false;
  bool use_avoidance_metrics_=false;
  unsigned int m_dof;
  std::vector<std::string> joint_names_;
  Eigen::VectorXd m_lb;
  Eigen::VectorXd m_ub;
  Eigen::VectorXd m_max_speed_;
  std::string group_;
  std::string tool_frame;
  bool display_flag=false;;

  pathplan::MetricsPtr metrics_;
  pathplan::AvoidanceMetricsPtr avoidance_metrics_;
  pathplan::AvoidanceGoalCostFunctionPtr m_avoidance_goal_cost_fcn;
  pathplan::CollisionCheckerPtr checker;

  ros::Subscriber m_centroid_sub;

  double collision_distance_=0.04;
  double collision_thread_=5;
  bool m_is_running=false;
  bool m_stop=false;


};

}
}
