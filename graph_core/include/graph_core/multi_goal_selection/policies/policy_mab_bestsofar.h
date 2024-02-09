#pragma once

#include <graph_core/multi_goal_selection/policies/policy_mab.h>
#include <std_msgs/Float64.h>


namespace multi_goal_selection
{

class PolicyMABBestSoFar : public PolicyMAB
{

protected:
  std::vector<double> utopias_;
  std::vector<double> alphas_, betas_, thetas_;
  std::vector<double> best_rewards_;
  double beta_gain_=1.0;

public:
  PolicyMABBestSoFar(const std::string& name, const int& n_goals) : PolicyMAB(name, n_goals)
  {
    pull_counter_ = std::vector<int>(n_goals_, 0);
    best_rewards_ = std::vector<double>(n_goals_, -std::numeric_limits<double>::infinity());
    utopias_ = std::vector<double>(n_goals_, 0.0);

    alphas_ = std::vector<double>(n_goals_, 1.0);
    betas_  = std::vector<double>(n_goals_, 1.0 );
    thetas_ = std::vector<double>(n_goals_, 0.0);

    if (!nh_.getParam("beta_gain", beta_gain_))
    {
      ROS_DEBUG("%s/beta_gain is not set. Deafult: %f",nh_.getNamespace().c_str(), beta_gain_);
    }

    ROS_WARN("BestSoFar initialized with beta_gain =%f", beta_gain_);
  }
  
  virtual int selectNextArm()
  {
    /*sample b-distribution, scale to [utopia,best_reward], then take the argmax */

    // take the worse reward except for -inf values and use it as lb for scaling
    double lb0 = std::numeric_limits<double>::infinity();
    if (vectorMax(best_rewards_) > -std::numeric_limits<double>::infinity())
    {
      for (int idy=0; idy<n_goals_; idy++)
      {
        if (best_rewards_[idy] > -std::numeric_limits<double>::infinity() && best_rewards_[idy] < lb0)
        {
          lb0 = best_rewards_[idy];
        }
      }
    }

    for(int idx=0; idx<n_goals_; idx++)
    {
      thetas_[idx] = beta_distribution<double>(alphas_[idx], betas_[idx])(gen_) ; // sample b-distribution \in [0,1]
      double lb = lb0;
      if (best_rewards_[idx] > lb0)
      {
        lb=best_rewards_[idx];
      }
      if (lb < std::numeric_limits<double>::infinity())
      {
        thetas_[idx] = thetas_[idx] * (utopias_[idx] - lb) + lb; // scale on range [best_cost_so_far, utopia]
      }
      ROS_INFO_THROTTLE(0.1,"arm=%d, lb = %f, ub = %f, br=%f, theta=%f", idx, lb, utopias_[idx],best_rewards_[idx],thetas_[idx]);
    }


    return vectorMaxIndex(thetas_);

    for(int idx=0; idx<n_goals_; idx++)
    {
      double lb = best_rewards_[idx];
      // exception: if the current goal does not have a solution yet, use the worse solution among all goals
      // otherwise goals without a solution would likely be skipped
      if (lb == -std::numeric_limits<double>::infinity())
      {
        // this is a bit cumbersome, but I'm just taking the worse reward except for -inf values
        double worst_reward=std::numeric_limits<double>::infinity();
        for(int idy=0; idy<n_goals_; idy++)
        {
          if (best_rewards_[idy] > -std::numeric_limits<double>::infinity() && best_rewards_[idy]<worst_reward)
          {
            worst_reward = best_rewards_[idy];
          }
        }
        if (worst_reward < std::numeric_limits<double>::infinity())
        {
          lb=worst_reward;
        }
      }
      thetas_[idx] = thetas_[idx] * (utopias_[idx] - lb) + lb; // scale on range [best_cost_so_far, utopia]
      ROS_INFO_THROTTLE(0.1,"lb = %f, ub = %f, br=%f, theta=%f", lb, utopias_[idx],best_rewards_[idx],thetas_[idx]);
    }

    return vectorMaxIndex(thetas_);
  }

  virtual void updateState(const int& i_goal, const double& reward)
  {
    if (pull_counter_.at(i_goal)==0) // executed during warm start
    {
      utopias_.at(i_goal) = reward;
    }
    else
    {
      if (reward>best_rewards_[i_goal])
      {
        best_rewards_[i_goal] = reward;
        // reinit beta-distribution
        alphas_[i_goal]=1.0; 
        betas_[i_goal] =1.0;
      } 
      else
      {
        betas_[i_goal] += beta_gain_; // push beta-distribution to lower bound
      }
    }

    pull_counter_[i_goal]++;
    
  }

  virtual std::string toString()
  {
    std::string str="BestSoFar Policy";
    return str;
  }

};
typedef std::shared_ptr<PolicyMABBestSoFar> PolicyMABBestSoFarPtr;

} //namespace
