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
#include <graph_core/solvers/birrt.h>
#include <graph_core/solvers/path_solver.h>
#include <graph_core/metrics.h>
#include <graph_core/moveit_collision_checker.h>
#include <graph_core/tube_informed_sampler.h>
#include <rosparam_utilities/rosparam_utilities.h>
#define COMMENT(...) ROS_LOG(::ros::console::levels::Debug, ROSCONSOLE_DEFAULT_NAME, __VA_ARGS__)


#include <fstream>
#include <iostream>


namespace pathplan {
namespace dirrt_star {

class DIRRTStar: public planning_interface::PlanningContext
{
public:
  DIRRTStar ( const std::string& name,
                const std::string& group,
                const moveit::core::RobotModelConstPtr& model
              );


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

  ros::WallDuration m_max_planning_time;
  ros::WallDuration m_max_refining_time;

  unsigned int m_dof;
  std::vector<std::string> joint_names_;
  Eigen::VectorXd m_lb;
  Eigen::VectorXd m_ub;
  std::string group_;
  bool m_tube_sampler;
  bool m_path_optimization;
  double m_forgetting_factor=0.99;
  double m_minimum_success_rate=1e-12;

  pathplan::MetricsPtr metrics;
  pathplan::CollisionCheckerPtr checker;

  std::mt19937 m_gen;
  std::uniform_real_distribution<double> m_ud;

  double collision_distance=0.04;
  bool m_is_running=false;
  bool m_stop=false;


};

}
}
