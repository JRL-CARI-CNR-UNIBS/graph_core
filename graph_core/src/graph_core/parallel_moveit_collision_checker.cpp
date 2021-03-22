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

ParallelMoveitCollisionChecker::ParallelMoveitCollisionChecker(const planning_scene::PlanningScenePtr &planning_scene,
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
    scene_mutex_.push_back(std::make_shared<std::mutex>());
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

    queues_mutex_.lock();
    queues_.at(idx).clear();
    queues_mutex_.unlock();

    completed_.at(idx)=false;
  }
}

void ParallelMoveitCollisionChecker::queueUp(const Eigen::VectorXd &q)
{
  if(q.size() != 0)
  {
    completed_.at(thread_iter_)=false;

    queues_mutex_.lock();
    queues_.at(thread_iter_).push_back(q);
    queues_mutex_.unlock();

    thread_iter_ +=1;
    if (thread_iter_>=threads_num_)
      thread_iter_=0;
  }
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

void ParallelMoveitCollisionChecker:: collisionThread(int thread_idx)
{
  scene_mutex_.at(thread_idx)->lock();
  robot_state::RobotStatePtr state=std::make_shared<robot_state::RobotState>(planning_scenes_.at(thread_idx)->getCurrentState());
  scene_mutex_.at(thread_idx)->unlock();

  while(!stop_threads_)
  {
    if (stop_check_)
    {
      stopped_.at(thread_idx)=true;
      completed_.at(thread_idx)=false;
      std::this_thread::sleep_for(std::chrono::microseconds(1));
      //state = std::make_shared<robot_state::RobotState>(planning_scenes_.at(thread_idx)->getCurrentState()); Perchè qua? L'ho spostato prima del for
      continue;                                                                                        //           |
    }                                                                                                  //           |
    if (completed_.at(thread_idx))                                                                     //           |
    {                                                                                                  //           |
      std::this_thread::sleep_for(std::chrono::microseconds(1));                                       //           |
      continue;                                                                                        //           |
    }                                                                                                  //           |
                                                                                                       //           v
    scene_mutex_.at(thread_idx)->lock();
    state = std::make_shared<robot_state::RobotState>(planning_scenes_.at(thread_idx)->getCurrentState()); //Spostato qua
    scene_mutex_.at(thread_idx)->unlock();


    queues_mutex_.lock();
    std::vector<Eigen::VectorXd> queue  = queues_.at(thread_idx);
    queues_mutex_.unlock();

    for (const Eigen::VectorXd& configuration: queue)
    {
      //      *state=planning_scenes_.at(thread_idx)->getCurrentState();

      if(configuration.size()==0)  //tieni per qualche test poi cancella
      {
        ROS_INFO_STREAM ("conf vuota: "<<configuration.transpose()<<" queue: "<<thread_idx << " length queue: "<<queue.size());
        throw std::invalid_argument("conf vuota thread");
      }

      scene_mutex_.at(thread_idx)->lock();
      state->setJointGroupPositions(group_name_, configuration);

      if (!state->satisfiesBounds())
      {
        at_least_a_collision_=true;
        stopped_.at(thread_idx)=true;
        completed_.at(thread_idx)=true;
        stop_check_=true;

        scene_mutex_.at(thread_idx)->unlock();
        break;
      }
      state->update();                        //servono entrambi? Se guardo linea 618 http://docs.ros.org/en/melodic/api/moveit_core/html/robot__state_8cpp_source.html#l00536 vedo che update chiama gia updateCollisionBodyTransform
      state->updateCollisionBodyTransforms(); //servono entrambi? Se guardo linea 618 http://docs.ros.org/en/melodic/api/moveit_core/html/robot__state_8cpp_source.html#l00536 vedo che update chiama gia updateCollisionBodyTransform

      if (!planning_scenes_.at(thread_idx)->isStateValid(*state,group_name_))
      {
        at_least_a_collision_=true;
        stopped_.at(thread_idx)=true;
        completed_.at(thread_idx)=true;
        stop_check_=true;

        scene_mutex_.at(thread_idx)->unlock();
        break;
      }
      scene_mutex_.at(thread_idx)->unlock();

      if (stop_check_)
        break;
    }
    stopped_.at(thread_idx)=true;
    completed_.at(thread_idx)=true;
  }
}

}  // pathplan