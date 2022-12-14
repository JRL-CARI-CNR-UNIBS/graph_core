#pragma once

#include <graph_core/multi_goal_selection/policies/policy_mab.h>

namespace multi_goal_selection
{

class PolicyMABUCB : public PolicyMAB
{

protected:
  double alpha_;

public:
  PolicyMABUCB(const std::string& name, const int& n_goals) : PolicyMAB(name, n_goals)
  {
    pull_counter_ = std::vector<int>(n_goals_, 0);
    expected_reward_ = std::vector<double>(n_goals_, 0.0);

    if (!nh_.getParam("ucb_alpha", alpha_))
    {
      ROS_DEBUG("%s/ucb_alpha is not set. Deafult: 1.0",nh_.getNamespace().c_str());
      alpha_=1.0;
    }
  }
  
  virtual int selectNextArm()
  {

    double n = vectorSum(pull_counter_);

    double max_expectation=-std::numeric_limits<double>::infinity();
    unsigned int i_goal_best=0;
    for(int i_goal=0; i_goal<n_goals_; i_goal++)
    {
      if(pull_counter_[i_goal]==0)
        return i_goal;

      double exp = expected_reward_[i_goal]/pull_counter_[i_goal]
          + sqrt( (alpha_ * log(n)) / (double)pull_counter_[i_goal] );
      if (exp > max_expectation)
      {
        max_expectation = exp;
        i_goal_best = i_goal;
      }
    }
    return i_goal_best;
  }

  virtual void updateState(const int& i_goal, const double& reward)
  {
    pull_counter_[i_goal]+=1;
    expected_reward_[i_goal]+=reward;
  }

  virtual std::string toString()
  {
    std::string str="UCB Policy with alpha=";
    str+=std::to_string(alpha_);
    return str;
  }

};
typedef std::shared_ptr<PolicyMABUCB> PolicyMABUCBPtr;

} //namespace
