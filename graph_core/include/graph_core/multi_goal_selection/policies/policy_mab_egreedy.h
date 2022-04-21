#pragma once

#include <graph_core/multi_goal_selection/policies/policy_mab.h>

namespace multi_goal_selection
{

class PolicyMABEGreedy : public PolicyMAB
{

protected:
  double epsilon_coef_; //epsilon (random play prob) - epsilon_base/(current)t
                      //epsilonbase = cK/d2

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

        double exp = expected_reward_[i_goal]/pull_counter_[i_goal];
        if (exp > max_expectation)
        {
          max_expectation = exp;
          i_goal_best = i_goal;
        }
      }
      return i_goal_best;
    }
  }

  virtual void updateState(const int& i_goal, const double& reward)
  {
    pull_counter_[i_goal]+=1;
    expected_reward_[i_goal]+=reward;
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
