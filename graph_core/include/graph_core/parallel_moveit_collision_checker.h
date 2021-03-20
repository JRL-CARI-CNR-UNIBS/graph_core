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


#include <graph_core/moveit_collision_checker.h>
#include <moveit/planning_scene/planning_scene.h>
#include <thread>
#include <mutex>

namespace pathplan
{

class ParallelMoveitCollisionChecker: public MoveitCollisionChecker
{
protected:
  int threads_num_;
  int thread_iter_=0;
  bool stop_threads_;
  bool stop_check_;
  bool at_least_a_collision_;

  std::vector<bool> stopped_;
  std::vector<bool> completed_;
  std::vector<std::vector<Eigen::VectorXd>> queues_;
  std::vector<std::thread> threads;
  std::vector<planning_scene::PlanningScenePtr> planning_scenes_;
  std::vector <std::shared_ptr<std::mutex>> mutex_;

  void resetQueue();
  void queueUp(const Eigen::VectorXd &q);
  bool checkAllQueues();
  void collisionThread(int thread_idx);

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  ParallelMoveitCollisionChecker(const planning_scene::PlanningScenePtr& planning_scene,
                                 const std::string& group_name,
                                 const int& threads_num=4,
                                 const double& min_distance = 0.01);

  ~ParallelMoveitCollisionChecker()
  {
    ROS_INFO("closing collision threads");
    stop_threads_=true;
    stop_check_=true;
    for (int idx=0;idx<threads_num_;idx++)
    {
      if (threads.at(idx).joinable())
        threads.at(idx).join();
    }
    ROS_INFO("collision threads closed");
  }


  virtual void setPlanningSceneMsg(const moveit_msgs::PlanningScene& msg)
  {
    stop_check_=true;
    for (int idx=0;idx<threads_num_;idx++)
    {
      while (!stopped_.at(idx))
        std::this_thread::sleep_for(std::chrono::microseconds(1));
    }
    if (!planning_scene_->setPlanningSceneMsg(msg))
    {
      ROS_ERROR("unaple to upload planning scn");
      //ROS_ERROR_THROTTLE(1,"unable to upload scene");
    }
    for (int idx=0;idx<threads_num_;idx++)
    {
      mutex_.at(idx)->lock();
      if(!planning_scenes_.at(idx)->setPlanningSceneMsg(msg))
      {
        ROS_ERROR("unaple to upload planning scn");
        //ROS_ERROR_THROTTLE(1,"unable to upload scene");
      }
      mutex_.at(idx)->unlock();
    }
  }

  virtual void setPlanningScene(planning_scene::PlanningScenePtr &planning_scene)
  {
    stop_check_=true;
    for (int idx=0;idx<threads_num_;idx++)
    {
      while (!stopped_.at(idx))
        std::this_thread::sleep_for(std::chrono::microseconds(1));
    }
    planning_scene_ = planning_scene;
    for (int idx=0;idx<threads_num_;idx++)
    {
      mutex_.at(idx)->lock();
      planning_scenes_.at(idx)=planning_scene::PlanningScene::clone(planning_scene_);
      mutex_.at(idx)->unlock();
    }
  }

  virtual bool checkPath(const Eigen::VectorXd& configuration1,
                         const Eigen::VectorXd& configuration2)
  {
    if(configuration1.size() == 0) ROS_ERROR("CONF1 VUOTA"); //ELIMINA
    if(configuration2.size() == 0) ROS_ERROR("CONF2 VUOTA");
    if(configuration1.size() == 0 || configuration2.size() == 0) throw std::invalid_argument("conf vuota");


    resetQueue();
    queueUp(configuration1);
    queueUp(configuration2);
    double distance = (configuration2 - configuration1).norm();
    if (distance < min_distance_)
      return checkAllQueues();

    Eigen::VectorXd conf(configuration1.size());
    double n = 2;

    while (distance > n * min_distance_)
    {
      for (double idx = 1; idx < n; idx += 2)
      {
        conf = configuration1 + (configuration2 - configuration1) * idx / n;
        if(conf.size() == 0)
        {
          ROS_ERROR("CONF VUOTA"); //ELIMINA
          throw std::invalid_argument("conf vuota");
        }
        queueUp(conf);
      }
      n *= 2;
    }
    return checkAllQueues();
  }

  virtual bool checkPathFromConf(const Eigen::VectorXd& parent,
                                 const Eigen::VectorXd& child,
                                 const Eigen::VectorXd& this_conf)
  {
    if(parent.size() == 0) ROS_ERROR("parent VUOTA"); //ELIMINA
    if(child.size() == 0) ROS_ERROR("child VUOTA");
    if(this_conf.size() == 0) ROS_ERROR("this conf VUOTA");
    if(parent.size() == 0 || child.size() == 0|| this_conf.size() == 0)throw std::invalid_argument("conf vuota");

    resetQueue();

    double dist_child = (this_conf-child).norm();
    double dist_parent = (parent-this_conf).norm();
    double dist = (parent-child).norm();

    if((dist-dist_child-dist_parent)>1e-04)
    {
      ROS_ERROR("The conf is not on the connection between parent and child");
      assert(0);
      return false;
    }

    queueUp(this_conf);
    queueUp(child);

    double distance = (this_conf - child).norm();
    if(distance < min_distance_)
      return checkAllQueues();

    double this_abscissa = (parent-this_conf).norm()/(parent-child).norm();
    double abscissa;
    double n = 2;
    Eigen::VectorXd conf;
    while (distance > n * min_distance_)
    {
      for (double idx = 1; idx < n; idx += 2)
      {
        abscissa = idx/n;
        if(abscissa>=this_abscissa)
        {
          conf = parent + (child - parent) * abscissa;
          if(conf.size() == 0)
          {
            ROS_ERROR(" conf2 VUOTA");
            throw std::invalid_argument("conf vuota");
          }
          queueUp(conf);
        }
      }
      n *= 2;
    }
    return checkAllQueues();
  }

};
}  // namaspace pathplan
