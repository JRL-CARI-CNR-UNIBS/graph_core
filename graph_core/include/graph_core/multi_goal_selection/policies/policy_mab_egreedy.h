#pragma once

#include <graph_core/multi_goal_selection/policies/policy_mab.h>

namespace multi_goal_selection
{

class PolicyMABEGreedy : public PolicyMAB
{

protected:
  double epsilon_coef_; //epsilon (random play prob) - epsilon_base/(current)t
                      //epsilonbase = cK/d2
  double egreedy_forgetting_factor_;
  std::vector<int> pulled_arms_;

public:
  PolicyMABEGreedy(const std::string& name, const int& n_goals) : PolicyMAB(name, n_goals)
  {
    pull_counter_ = std::vector<int>(n_goals_, 0);
    expected_reward_ = std::vector<double>(n_goals_, 0.0);

    if (!nh_.getParam("epsilon_coef", epsilon_coef_))
    {
      ROS_DEBUG("%s/epsilon_coef is not set. Deafult: 0.1",nh_.getNamespace().c_str());
      epsilon_coef_=0.1;
    }
    if (!nh_.getParam("egreedy_forgetting_factor", egreedy_forgetting_factor_))
    {
      ROS_DEBUG("%s/egreedy_forgetting_factor is not set. Deafult: 0.0",nh_.getNamespace().c_str());
      egreedy_forgetting_factor_=0.0;
    }
    else
      ROS_INFO("%s/egreedy_forgetting_factor set to %f",nh_.getNamespace().c_str(),egreedy_forgetting_factor_);
  }
  
  virtual int selectNextArm()
  {
//    double n = vectorSum(pull_counter_);
//    double en = epsilon_coef_/n;
    double rand = std::uniform_real_distribution<double>(0.0,1.0)(gen_);
    if(epsilon_coef_ > rand){ //random choice
      return std::uniform_int_distribution<int>(0, n_goals_-1)(gen_);
    }
    else
    {
      double max_expectation=-std::numeric_limits<double>::infinity();
      unsigned int i_goal_best=0;
      for(unsigned int i_goal=0; i_goal<n_goals_; i_goal++)
      {
        if(pull_counter_[i_goal]==0)
          return i_goal;

        if (expected_reward_[i_goal] > max_expectation)
        {
          max_expectation = expected_reward_[i_goal];
          i_goal_best = i_goal;
        }
      }
      return i_goal_best;
    }
  }

  virtual void updateState(const int& i_goal, const double& reward)
  {
    if (egreedy_forgetting_factor_<=0.0)
    {
      pull_counter_[i_goal]+=1;
      expected_reward_[i_goal]+=(reward-expected_reward_[i_goal])/pull_counter_[i_goal];
    }
    else
    {
      for (unsigned int idx=0;idx<expected_reward_.size();idx++)
        expected_reward_[idx]=(1-egreedy_forgetting_factor_)*expected_reward_[idx];
      expected_reward_[i_goal]+=egreedy_forgetting_factor_*reward;
      epsilon_coef_=std::min((1-egreedy_forgetting_factor_)*epsilon_coef_+egreedy_forgetting_factor_*reward,1.0);
    }
  }

  virtual std::string toString()
  {
    std::string str="Egreedy Policy with epsilon_coef_=";
    str+=std::to_string(epsilon_coef_);
    return str;
  }

};
typedef std::shared_ptr<PolicyMABEGreedy> PolicyMABEGreedyPtr;

} //namespace
