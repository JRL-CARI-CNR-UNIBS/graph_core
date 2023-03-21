#pragma once

#include <graph_core/multi_goal_selection/policies/policy_mab.h>
#include <std_msgs/Float64MultiArray.h>


namespace multi_goal_selection
{

class PolicyMABKFMANB : public PolicyMAB
{

protected:
  bool sigma_obs_dyn_=false;
  double sigma_gain_=1.0;
  double sigma_obs_2_;
  double sigma_tr_2_;
  double eta_ = 1.0;
  std::vector<double> sigma_arms_2_;
  std::vector<double> sampled_rewards_;

  ros::Publisher kf_error_pub_;



  //ros::Publisher eps_pub_ = nh_.advertise<std_msgs::Float64>("eps_greedy", 1000);
  //ros::Publisher reward_max_pub_ = nh_.advertise<std_msgs::Float64>("max_reward", 1000);
  //ros::Publisher reward_second_max_pub_ = nh_.advertise<std_msgs::Float64>("second_max_reward", 1000);
  //std_msgs::Float64 msg_;

public:

  PolicyMABKFMANB(const std::string& name, const int& n_goals) : PolicyMAB(name, n_goals)
  {
    pull_counter_ = std::vector<int>(n_goals_, 0);
    expected_reward_ = std::vector<double>(n_goals_, 0.0);
    sigma_arms_2_ = std::vector<double>(n_goals_, 0.0);
    sampled_rewards_ = std::vector<double>(n_goals_, 0.0);

    kf_error_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("/kf_error", 100);

    double tmp;
    if (!nh_.getParam("sigma_obs", tmp))
    {
      tmp = 0.01;
      ROS_DEBUG("%s/sigma_obs is not set. Default: %f",nh_.getNamespace().c_str(),tmp);
    }
    sigma_obs_2_ = tmp*tmp;

    if (!nh_.getParam("sigma_tr", tmp))
    {
      tmp = 0.1;
      ROS_DEBUG("%s/sigma_tr is not set. Default: %f",nh_.getNamespace().c_str(),tmp);
    }
    sigma_tr_2_ = tmp*tmp;

    if (!nh_.getParam("sigma_arms", tmp))
    {
      tmp = 0.2;
      ROS_DEBUG("%s/sigma_arms is not set. Default: %f",nh_.getNamespace().c_str(),tmp);
    }
    std::fill(sigma_arms_2_.begin(), sigma_arms_2_.end(), tmp*tmp);

    if (!nh_.getParam("sigma_obs_dyn", sigma_obs_dyn_))
    {
      ROS_DEBUG("%s/sigma_obs_dyn is not set. Default: false",nh_.getNamespace().c_str());
    }
    if (sigma_obs_dyn_)
    {
      if (!nh_.getParam("sigma_gain", sigma_gain_))
      {
        ROS_DEBUG("%s/sigma_gain is not set. Default: %f",nh_.getNamespace().c_str(),sigma_gain_);
      }
    }

    ROS_DEBUG("KFMANB initialized with sigma_obs=%f, "
             "sigma_tr=%f, "
             "eta=%f, "
             "reward_max=%f, "
             "sigma_arm (max)=%f, ",
             std::sqrt(sigma_obs_2_),
             std::sqrt(sigma_tr_2_),
             eta_,
             vectorMax(expected_reward_),
             std::sqrt(vectorMax(sigma_arms_2_) ));

  }

  std::vector<double> getVariance()
  {
    return sigma_arms_2_;
  }

  virtual int selectNextArm()
  {
    for(int idx=0; idx<n_goals_; idx++)
    {
      sampled_rewards_[idx] = std::normal_distribution<>(expected_reward_[idx], std::sqrt(sigma_arms_2_[idx]))(gen_) ;
    }

    /*
    ROS_INFO("\nexp reward = ");
    for(int idx=0; idx<n_goals_; idx++)
    {
      std::cout << expected_reward_[idx] << "+" << std::sqrt(sigma_arms_2_[idx]) << ", ";
    }
    ROS_INFO("\nsampled rewards = ");
    for(int idx=0; idx<n_goals_; idx++)
    {
      std::cout << sampled_rewards_[idx] << ", ";
    }
    ROS_INFO("best = %d, val = %f", vectorMaxIndex(sampled_rewards_), sampled_rewards_[vectorMaxIndex(sampled_rewards_)]);
    */
    return vectorMaxIndex(sampled_rewards_);
  }

  bool reinitRewards(std::vector<double> rewards, std::vector<double> std_devs)
  {
    if (!multi_goal_selection::PolicyMAB::reinitRewards(rewards, std_devs))
    {
      return false;
    }

    if (std_devs.size() != n_goals_)
    {
      ROS_FATAL("Wrong size of vector std_devs.");
      return false;
    }

    /*
    for (int idx=0;idx<sigma_arms_2_.size();idx++)
    {
      sigma_arms_2_[idx] = std_devs[idx]*std_devs[idx];
    }
    */

    return true;

  }

  virtual void updateState(const int& i_goal, const double& reward, const double& variance)
  {
    if (sigma_obs_dyn_)
      sigma_obs_2_ = sigma_gain_*variance;
    updateState(i_goal, reward);
  }

  virtual void updateState(const int& i_goal, const double& reward)
  {

    std_msgs::Float64MultiArray kf_err_msg;
    for (unsigned int idx=0;idx<expected_reward_.size();idx++)
    {
      kf_err_msg.data.push_back((expected_reward_[idx]-initial_reward_[idx]));
    }

    kf_error_pub_.publish(kf_err_msg);

    eta_ = std::max(1e-10,0.9*eta_+0.1*std::abs(reward)); // see McConachie and Berenson, TASE 2018
    double sigma_eta_2 = sigma_tr_2_*eta_*eta_;

    for (int idx=0;idx<n_goals_;idx++)
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
    str+=", sigma_tr_2 = ";
    str+=std::to_string(sigma_tr_2_);
    str+=", sigma_arms_2 (max) = ";
    str+=std::to_string(vectorMax(sigma_arms_2_));
    return str;
  }

};
typedef std::shared_ptr<PolicyMABKFMANB> PolicyMABKFMANBPtr;

} //namespace
