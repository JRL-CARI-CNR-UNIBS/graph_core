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
  double lb0_ = -std::numeric_limits<double>::infinity();

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

    ROS_WARN(toString().c_str());
  }
  
  virtual int selectNextArm()
  {
    /*sample b-distribution, scale to [utopia,best_reward], then take the argmax */

    for(int idx=0; idx<n_goals_; idx++)
    {
      thetas_[idx] = beta_distribution<double>(alphas_[idx], betas_[idx])(gen_) ; // sample b-distribution \in [0,1]
      double lb = lb0_;
      if (best_rewards_[idx] > lb0_)
      {
        lb=best_rewards_[idx];
      }
      if (lb != std::numeric_limits<double>::infinity() && lb != -std::numeric_limits<double>::infinity())
      {
        thetas_[idx] = thetas_[idx] * (utopias_[idx] - lb) + lb; // scale on range [best_cost_so_far, utopia]
      }
      ROS_INFO_THROTTLE(0.1,"arm=%d, lb = %f, ub = %f, br=%f, theta=%f", idx, lb, utopias_[idx],best_rewards_[idx],thetas_[idx]);
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
      /*
      if (reward>vectorMax(best_rewards_))
      {
        // reinit all beta-distributions if best cost improved
        std::fill(alphas_.begin(), alphas_.end(), 1.0);
        std::fill(betas_.begin(), betas_.end(), 1.0);
      }
      */
      if (reward>best_rewards_[i_goal])
      {
        best_rewards_[i_goal] = reward;
        // reinit ith beta-distribution if ith reward improved
        alphas_[i_goal]=1.0; 
        betas_[i_goal] =1.0;
      } 
      else
      {
        betas_[i_goal] += beta_gain_; // push beta-distribution to lower bound
      }
    }

    /*
    // take the worse reward except for -inf values and use it as lb for scaling
    lb0_ = std::numeric_limits<double>::infinity();
    if (vectorMax(best_rewards_) > -std::numeric_limits<double>::infinity())
    {
      for (int idy=0; idy<n_goals_; idy++)
      {
        if (best_rewards_[idy] > -std::numeric_limits<double>::infinity() && best_rewards_[idy] < lb0_)
        {
          lb0_ = best_rewards_[idy];
        }
      }
    }
    */

    // set lb for scaling equal to best cost
    lb0_ = vectorMax(best_rewards_);

    pull_counter_[i_goal]++;
    
  }

  virtual std::string toString()
  {
    std::string str="BestSoFar Policy initialized with beta_gain = ";
    str+=std::to_string(beta_gain_);
    return str;
  }

};
typedef std::shared_ptr<PolicyMABBestSoFar> PolicyMABBestSoFarPtr;

} //namespace
