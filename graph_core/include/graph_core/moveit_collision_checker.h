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


#include <graph_core/collision_checker.h>
#include <moveit/planning_scene/planning_scene.h>

namespace pathplan
{

class MoveitCollisionChecker: public CollisionChecker
{
protected:
  planning_scene::PlanningScenePtr planning_scene_;
  collision_detection::CollisionRequest req_;
  collision_detection::CollisionResult res_;
  robot_state::RobotStatePtr state_;
  std::string group_name_;


public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  MoveitCollisionChecker(const planning_scene::PlanningScenePtr& planning_scene,
                         const std::string& group_name,
                         const double& min_distance = 0.01):
    CollisionChecker(min_distance),
    group_name_(group_name),
    planning_scene_(planning_scene)
  {

    state_ = std::make_shared<robot_state::RobotState>(planning_scene_->getCurrentState());
    if (!planning_scene_)
      ROS_ERROR("invalid planning scene");
  }

  virtual void setPlanningSceneMsg(const moveit_msgs::PlanningScene& msg)
  {
    if (!planning_scene_->setPlanningSceneMsg(msg))
    {
      ROS_ERROR_THROTTLE(1,"unable to upload scene");
    }
  }

  virtual CollisionCheckerPtr clone()
  {
    planning_scene::PlanningScenePtr planning_scene = planning_scene::PlanningScene::clone(planning_scene_);
    return std::make_shared<MoveitCollisionChecker>(planning_scene,group_name_,min_distance_);
  }

  virtual bool check(const Eigen::VectorXd& configuration)
  {
    *state_ = planning_scene_->getCurrentState();
    state_->setJointGroupPositions(group_name_, configuration);
    if (!state_->satisfiesBounds())
    {
      ROS_DEBUG("Out of bound");
      return false;
    }
    state_->update();
    state_->updateCollisionBodyTransforms();
    return planning_scene_->isStateValid(*state_,group_name_);

  }

  virtual void setPlanningScene(planning_scene::PlanningScenePtr &planning_scene)
  {
    planning_scene_ = planning_scene;
  }

};
}
