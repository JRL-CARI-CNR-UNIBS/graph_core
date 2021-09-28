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
#include <dirrt_star/multigoal_planner.h>
#include <dirrt_star/time_planner.h>
#include <dirrt_star/hamp_time_planner.h>
#include <dirrt_star/probabilist_hamp_time_planner.h>

namespace pathplan {
namespace dirrt_star {
class PathPlanerManager : public planning_interface::PlannerManager
{
public:
  virtual bool initialize(const robot_model::RobotModelConstPtr& model, const std::string& ns) override;
  std::string getDescription() const override
  {
    return "DIRRT";
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

  std::map< std::string, std::shared_ptr<planning_interface::PlanningContext>> m_planners;
  moveit::core::RobotModelConstPtr m_robot_model;
  std::string m_default_planner_config;
};

//

}
}

