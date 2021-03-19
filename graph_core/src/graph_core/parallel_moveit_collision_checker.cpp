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



#include <graph_core/parallel_moveit_collision_checker.h>


namespace pathplan
{

ParallelMoveitCollisionChecker::ParallelMoveitCollisionChecker(const planning_scene::PlanningSceneConstPtr &planning_scene,
                                                               const std::string& group_name,
                                                               const int& threads_num,
                                                               const double& min_distance):
  MoveitCollisionChecker(planning_scene,group_name,min_distance),
  threads_num_(threads_num)
{
  if (threads_num<=0)
    throw std::invalid_argument("number of thread should be positive");

  stop_threads_=false;
  at_least_a_collision_=false;
  stop_check_=true;
  thread_iter_=0;
  for (int idx=0;idx<threads_num_;idx++)
  {
    stopped_.push_back(true);
    completed_.push_back(false);

    planning_scenes_.push_back(planning_scene::PlanningScene::clone(planning_scene_));
    queues_.push_back(std::vector<Eigen::VectorXd>());
    threads.push_back(std::thread(&ParallelMoveitCollisionChecker::collisionThread,this,idx));
  }
}

void ParallelMoveitCollisionChecker::resetQueue()
{
  at_least_a_collision_=false;
  stop_check_=true;
  thread_iter_=0;
  for (int idx=0;idx<threads_num_;idx++)
  {
    while (!stopped_.at(idx))
      std::this_thread::sleep_for(std::chrono::microseconds(1));
    queues_.at(idx).clear();
    completed_.at(idx)=false;
  }
}

void ParallelMoveitCollisionChecker::queueUp(const Eigen::VectorXd &q)
{

  completed_.at(thread_iter_)=false;
  queues_.at(thread_iter_++).push_back(q);
  if (thread_iter_>=threads_num_)
    thread_iter_=0;
}

bool ParallelMoveitCollisionChecker::checkAllQueues()
{
  stop_check_=false;
  while (true)
  {
    std::this_thread::sleep_for(std::chrono::microseconds(1));
    bool still_run=false;
    for (int idx=0;idx<threads_num_;idx++)
      if (!completed_.at(idx))
        still_run=true;

    if (at_least_a_collision_)
    {
      stop_check_=true;
      return false;
    }
    if (!still_run)
      break;
  }
  return  !at_least_a_collision_;
}

void ParallelMoveitCollisionChecker::collisionThread(int thread_idx)
{
  robot_state::RobotStatePtr state_;
  while(!stop_threads_)
  {
    if (stop_check_)
    {
      stopped_.at(thread_idx)=true;
      completed_.at(thread_idx)=false;
      std::this_thread::sleep_for(std::chrono::microseconds(1));
      state_ = std::make_shared<robot_state::RobotState>(planning_scene_->getCurrentState());
      continue;
    }
    if (completed_.at(thread_idx))
    {
      std::this_thread::sleep_for(std::chrono::microseconds(1));
      continue;
    }
    for (const Eigen::VectorXd& configuration: queues_.at(thread_idx))
    {
      state_->setJointGroupPositions(group_name_, configuration);
//      if (!state_->satisfiesBounds())
//      {
//        at_least_a_collision_=true;
//        stopped_.at(thread_idx)=true;
//      completed_.at(thread_idx)=true;
//        stop_check_=true;
//        break;
//      }
//      state_->update();
      state_->updateCollisionBodyTransforms();
      if (!planning_scenes_.at(thread_idx)->isStateValid(*state_,group_name_))
      {
        at_least_a_collision_=true;
        stopped_.at(thread_idx)=true;
        completed_.at(thread_idx)=true;
        stop_check_=true;
        break;
      }
      if (stop_check_)
        break;
    }
    stopped_.at(thread_idx)=true;
    completed_.at(thread_idx)=true;
  }
}

}  // pathplan
