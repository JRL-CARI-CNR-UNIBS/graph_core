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
#include <graph_core/multi_goal_selection/policies/policy_mab.h>

namespace multi_goal_selection
{

class PolicyUniformOnVolume: public PolicyMAB
{
public:
  PolicyUniformOnVolume(const std::string& name, const int& n_goals, const int& n_dof) : PolicyMAB(name, n_goals)
  {
    volume_exp_ = 0.5*(n_dof-1);
    volume_factors_sums_.resize(n_goals_);
    std::fill(volume_factors_sums_.begin(), volume_factors_sums_.end(), 0.0);
  }

  int selectNextArm()
  {
    if (best_cost_squared_!=std::numeric_limits<double>::infinity())
    {
      double rnd = std::uniform_real_distribution<double>(0.0,volume_factors_sums_.back())(gen_);
      for (unsigned int idx_goal=0;idx_goal<n_goals_;idx_goal++)
      {
        if (rnd<=volume_factors_sums_.at(idx_goal))
        {
          return idx_goal;
        }
      }
    }
    else
    {
      return std::uniform_int_distribution<int>(0,n_goals_-1)(gen_);
      ROS_WARN_THROTTLE(0.1,"No solution found up to now. Using uniform on goals.");
    }
    ROS_ERROR("Error in arm selection.");
    return -1;
  };

  void updateState(const int& i_goal, const double& reward)
  {
    if (pull_counter_.at(i_goal)==0)
    {
      expected_reward_.at(i_goal) = std::pow(reward,2); // expected_reward_ =  utopias^2 during warm start. Note: reward must be BestCost
    }
    else if (std::pow(reward,2) < best_cost_squared_)
    {
      best_cost_squared_ = std::pow(reward,2);

      for (unsigned int idx_goal=0;idx_goal<n_goals_;idx_goal++)
      {
        double volume_factor = std::pow(best_cost_squared_-expected_reward_.at(idx_goal),volume_exp_);
        if (volume_factor<=0)
        {
          volume_factor = 0;
        }
        if (idx_goal==0)
        {
          volume_factors_sums_.at(idx_goal)=volume_factor;
        }
        else
        {
          volume_factors_sums_.at(idx_goal)=volume_factors_sums_.at(idx_goal-1) + volume_factor;
        }
      }
    }
    pull_counter_.at(i_goal)++;
  };

  std::string toString()
  {
    std::string str = "uniform on volume";
    return str;
  };
protected:
  double best_cost_squared_=std::numeric_limits<double>::infinity();
  double volume_exp_=0.0;
  std::vector<double> volume_factors_sums_;

};
typedef std::shared_ptr<PolicyUniformOnVolume> PolicyUniformOnVolumePtr;

}
