#pragma once
/*
Copyright (c) 2021, Marco Faroni CNR-STIIMA marco.faroni@stiima.cnr.it
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

#include <ros/ros.h>
#include <graph_core/multi_goal_selection/policies/policy_base.h>
#include <graph_core/multi_goal_selection/goal_selection_util.h>

namespace multi_goal_selection
{

class PolicyMAB: public PolicyBase
{
public:
  PolicyMAB(const std::string& name, const int& n_goals) : PolicyBase(name, n_goals)
  {
    pull_counter_.resize(n_goals);
    std::fill(pull_counter_.begin(), pull_counter_.end(), 0);
    expected_reward_.resize(n_goals);
    std::fill(expected_reward_.begin(), expected_reward_.end(), 0.0);
  }

  std::vector<double> getProbabilities()
  {
    std::fill(goals_probabilities_.begin(), goals_probabilities_.end(), 0.0);
    int arm = selectNextArm();
    goals_probabilities_.at(arm) = 1.0;
    return goals_probabilities_;
  }

  virtual int selectNextArm() = 0;

  bool reinitAvgReward(std::vector<double> rewards)
  {
    if (rewards.size() != n_goals_)
    {
      ROS_FATAL("Wrong dize of vector rewards.");
      return false;
    }
    expected_reward_ = rewards;
    std::fill(pull_counter_.begin(), pull_counter_.end(), 1);

    return true;
  }

  virtual void updateState(const int& i_goal, const double& reward)
  {
    pull_counter_[i_goal]+=1;
    expected_reward_[i_goal]+=reward;
  }

  virtual std::string toString() = 0;

  virtual void print()
  {
    std::cout << toString().c_str() << "\n";
    std::cout << "Number of arms: " << n_goals_ << "\n";
    std::cout << "Rewards: ";
    for (unsigned int idx=0;idx<n_goals_;idx++)
    {
      std::cout << expected_reward_[idx] << ", ";
    }
    std::cout << "\n";
    std::cout << "Pull counters: ";
    for (unsigned int idx=0;idx<n_goals_;idx++)
    {
      std::cout << pull_counter_[idx] << ", ";
    }
    std::cout << "\n";
  }

protected:
  std::vector<int> pull_counter_;
  std::vector<double> expected_reward_;

};
typedef std::shared_ptr<PolicyMAB> PolicyMABPtr;

}
