#include <ros/ros.h>
#include <graph_core/multi_goal_selection/goal_selection_manager.h>
#include <random>
#include <algorithm>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_goal_selection");
  ros::NodeHandle nh;

  std::string policy_type = "MultiArmedBandit"; // MultiArmedBandit Custom
  std::string policy_name = "eGreedy"; // eGreedy Custom1
  std::string reward_fcn = "RelativeImprovement";
  unsigned int n_goals = 10;
  unsigned int dof = 3;

  std::mt19937 rand_gen(time(0));

  std::vector<double> true_costs(n_goals);
  std::vector<double> costs(n_goals);
  std::vector<double> utopias(n_goals);
  std::vector<bool> were_goals_selected(n_goals);

  std::vector<double> probabilities(n_goals);
  double offset = 10.0;

  std::vector<double> cost_log_baseline;
  cost_log_baseline.clear();
  std::vector<int> arm_log_baseline;
  arm_log_baseline.clear();

  std::vector<double> cost_log;
  cost_log.clear();
  std::vector<int> arm_log;
  arm_log.clear();

  for (unsigned int i_goal=0;i_goal<n_goals;i_goal++)
  {
    true_costs.at(i_goal) = offset + i_goal;
    costs.at(i_goal) = std::numeric_limits<double>::infinity();
    utopias.at(i_goal) = 0.0;
    were_goals_selected.at(i_goal) = false;
    probabilities.at(i_goal) = 1.0;
  }
  auto rng = std::default_random_engine {};
  std::shuffle(std::begin(true_costs), std::end(true_costs), rng);
  std::cout << "True rewards: ";
  for (auto& el: true_costs)
    std::cout << el << ", ";

  int best_arm = *(std::minmax_element(true_costs.begin(), true_costs.end())).first;
  std::cout << "Best arm = " << best_arm << std::endl;

  double best_cost = *std::min_element(costs.begin(), costs.end());
  double best_cost_baseline = best_cost;

  multi_goal_selection::GoalSelectionManager manager(nh.getNamespace(),n_goals,dof);

  if (manager.isWarmStartSet())
  {
    manager.warmStart(costs,utopias,best_cost);
  }

  for (unsigned int iter=0;iter<200;iter++)
  {
    probabilities = manager.calculateProbabilities(were_goals_selected,costs,utopias,best_cost);

    were_goals_selected = std::vector<bool>(n_goals,false);
    int arm = -1;
    for (unsigned int i_goal=0;i_goal<n_goals;i_goal++)
    {
      double rand = std::uniform_real_distribution<double>(0.0,1.0)(rand_gen);
      if (rand < probabilities.at(i_goal))
      {
        arm = i_goal;
        double cost = true_costs.at(i_goal) + std::uniform_real_distribution<double>(0.0,1.0)(rand_gen);
        if (cost<best_cost)
          best_cost=cost;
        were_goals_selected.at(i_goal) = true;
      }
    }

    // baseline
    int select_goal = std::uniform_int_distribution<int>(0,n_goals-1)(rand_gen);
    double cost_baseline = true_costs.at(select_goal) + std::uniform_real_distribution<double>(0.0,1.0)(rand_gen);
    if (cost_baseline<best_cost_baseline)
      best_cost_baseline=cost_baseline;

    cost_log.push_back(best_cost);
    arm_log.push_back(arm);

    cost_log_baseline.push_back(best_cost_baseline);
    arm_log_baseline.push_back(select_goal);

  }

  std::cout << "[Adaptive, Uniform] : " << best_cost << " , " << best_cost_baseline << std::endl;

  nh.setParam("egreedy/cost",cost_log);
  nh.setParam("egreedy/arm",arm_log);


  nh.setParam("baseline/cost",cost_log_baseline);
  nh.setParam("baseline/arm",arm_log_baseline);

  return 0;
}
