#pragma once

#include <graph_core/multi_goal_selection/policies/policy_mab.h>
//#include <std_msgs/Float64.h>


namespace multi_goal_selection
{

class PolicyMABKFMANB : public PolicyMAB
{

protected:
  double sigma_obs_2_;
  double sigma_tr_2_;
  double eta_ = 1.0;
  std::vector<double> sigma_arms_2_;
  std::vector<double> sampled_rewards_;


  //ros::Publisher eps_pub_ = nh_.advertise<std_msgs::Float64>("eps_greedy", 1000);
  //ros::Publisher reward_max_pub_ = nh_.advertise<std_msgs::Float64>("max_reward", 1000);
  //ros::Publisher reward_second_max_pub_ = nh_.advertise<std_msgs::Float64>("second_max_reward", 1000);
  //std_msgs::Float64 msg_;

public:
  PolicyMABKFMANB(const std::string& name, const int& n_goals) : PolicyMAB(name, n_goals)
  {
    pull_counter_ = std::vector<int>(n_goals_, 0);
    expected_reward_ = std::vector<double>(n_goals_, 0.0);
    sigma_arms_2_ = std::vector<double>(n_goals_, 0.3*0.3);
    sampled_rewards_ = std::vector<double>(n_goals_, 0.0);

    double tmp;
    if (!nh_.getParam("sigma_obs", tmp))
    {
      tmp = 0.01;
      ROS_ERROR("%s/sigma_obs is not set. Default: %f",nh_.getNamespace().c_str(),tmp);
    }
    sigma_obs_2_ = tmp*tmp;

    if (!nh_.getParam("sigma_tr", tmp))
    {
      tmp = 0.1;
      ROS_ERROR("%s/sigma_tr is not set. Default: %f",nh_.getNamespace().c_str(),tmp);
    }
    sigma_tr_2_ = tmp*tmp;

    ROS_WARN("KFMANB initialized with sigma_obs=%f, "
             "sigma_tr=%f, "
             "eta=%f, ",
             std::sqrt(sigma_obs_2_), std::sqrt(sigma_tr_2_), eta_);

  }

  virtual int selectNextArm()
  {
    for(unsigned int idx=0; idx<n_goals_; idx++)
    {
      sampled_rewards_[idx] = std::normal_distribution<>(expected_reward_[idx], std::sqrt(sigma_arms_2_[idx]))(gen_) ;
    }

    ROS_WARN_ONCE("first reward id = %d, val = %f", vectorMaxIndex(sampled_rewards_), sampled_rewards_[vectorMaxIndex(sampled_rewards_)]);
    return vectorMaxIndex(sampled_rewards_);
  }

  virtual void updateState(const int& i_goal, const double& reward)
  {
    eta_ = std::max(1e-10,0.9*eta_+0.1*std::abs(reward)); // see McConachie and Berenson, TASE 2018
    double sigma_eta_2 = sigma_tr_2_*eta_*eta_;

    for (unsigned int idx=0;idx<n_goals_;idx++)
    {
      if (idx==i_goal)
      {
        expected_reward_[idx] = ((sigma_arms_2_[idx] + sigma_eta_2)*reward + sigma_obs_2_*expected_reward_[idx]) /
                                (sigma_arms_2_[idx] + sigma_eta_2 + sigma_obs_2_);

        sigma_arms_2_[idx] = (sigma_arms_2_[idx] + sigma_eta_2)*sigma_obs_2_ /
                           (sigma_arms_2_[idx] + sigma_eta_2 + sigma_obs_2_);
      }
      else
      {
        sigma_arms_2_[idx] = sigma_arms_2_[idx] + sigma_tr_2_;
      }
    }
    pull_counter_[i_goal]++;
  }

  virtual std::string toString()
  {
    std::string str="KFMANB Policy with sigma_obs_2 = ";
    str+=std::to_string(sigma_obs_2_);
    str+=" and sigma_tr_2 = ";
    str+=std::to_string(sigma_tr_2_);
    return str;
  }

};
typedef std::shared_ptr<PolicyMABKFMANB> PolicyMABKFMANBPtr;

} //namespace
