/*
Copyright (c) 2019, Manuel Beschi CNR-STIIMA manuel.beschi@stiima.cnr.it
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

#include <graph_core/solvers/tree_solver.h>

namespace graph
{
namespace core
{

bool TreeSolver::config(const YAML::Node &config)
{
  config_ = config;

  if (!config_["max_distance"])
  {
    CNR_WARN(logger_,"max_distance is not set, using 1.0");
    max_distance_ = 1.0;
  }
  else
  {
    max_distance_ = config_["max_distance"].as<double>();
  }

  if (!config_["use_kdtree"])
  {
    CNR_WARN(logger_,"use_kdtree is not set, using true");
    use_kdtree_ = true;
  }
  else
  {
    use_kdtree_ = config_["use_kdtree"].as<bool>();
  }

  if (!config_["informed"])
  {
    CNR_WARN(logger_,"informed is not set, using true");
    informed_ = true;
  }
  else
  {
    informed_ = config_["informed"].as<bool>();
  }

  if (!config_["extend"])
  {
    CNR_WARN(logger_,"extend is not set, using false (connect algorithm)");
    extend_ = false;
  }
  else
  {
    extend_ = config_["extend"].as<bool>();
  }

  if (!config["warp"])
  {
    CNR_DEBUG(logger_,"%s/warp is not set, using false");
    warp_ = false;
  }
  else
  {
    warp_ = config_["warp"].as<bool>();
  }

  if (!warp_)
  {
    if (!config_["warp_once"])
    {
      CNR_DEBUG(logger_,"%warp_once is not set. using false");
      first_warp_ = false;
    }
    else
    {
      first_warp_ = config_["warp_once"].as<bool>();
    }
  }
  else
  {
    first_warp_ = true;
  }

  if (!config_["utopia_tolerance"])
  {
    CNR_WARN(logger_,"utopia_tolerance is not set. using 0.01");
    utopia_tolerance_ = 0.01;
  }
  else
  {
    utopia_tolerance_ = config_["utopia_tolerance"].as<double>();
  }

  if (utopia_tolerance_ <= 0.0)
  {
    CNR_WARN(logger_,"utopia_tolerance cannot be negative, set equal to 0.0");
    utopia_tolerance_ = 0.0;
  }
  utopia_tolerance_ += 1.0;

  dof_ = sampler_->getDimension();
  configured_ = true;
  return true;
}

bool TreeSolver::setProblem(const double &max_time)
{
  init_ = false;
  if (!start_tree_)
    return false;
  if (!goal_node_)
    return false;
  goal_cost_ = goal_cost_fcn_->cost(goal_node_);

  best_utopia_ = goal_cost_+metrics_->utopia(start_tree_->getRoot()->getConfiguration(),goal_node_->getConfiguration());
  init_ = true;

  if (start_tree_->isInTree(goal_node_))
  {
    solution_ = std::make_shared<Path>(start_tree_->getConnectionToNode(goal_node_), metrics_, checker_, logger_);
    solution_->setTree(start_tree_);

    path_cost_ = solution_->cost();
    sampler_->setCost(path_cost_);
    start_tree_->addNode(goal_node_);

    solved_ = true;
    cost_=path_cost_+goal_cost_;
    return true;
  }

  NodePtr new_node;

  if(start_tree_->connectToNode(goal_node_, new_node,max_time))  //for direct connection to goal
  {
    solution_ = std::make_shared<Path>(start_tree_->getConnectionToNode(goal_node_), metrics_, checker_, logger_);
    solution_->setTree(start_tree_);

    path_cost_ = solution_->cost();
    sampler_->setCost(path_cost_);
    start_tree_->addNode(goal_node_);

    solved_ = true;
    CNR_DEBUG(logger_,"A direct solution is found\n" << *solution_);
  }
  else
  {
    path_cost_ = std::numeric_limits<double>::infinity();
  }
  cost_=path_cost_+goal_cost_;
  return true;
}

bool TreeSolver::solve(PathPtr &solution, const unsigned int& max_iter, const double& max_time)
{
  std::chrono::time_point<std::chrono::system_clock> tic = std::chrono::system_clock::now();

  if(max_time <=0.0)
    return false;

  for (unsigned int iter = 0; iter < max_iter; iter++)
  {
    if (update(solution))
    {
      solved_ = true;
      return true;
    }

    std::chrono::time_point<std::chrono::system_clock> now = std::chrono::system_clock::now();
    std::chrono::duration<double> difference = now - tic;
    if(difference.count() >= 0.98*max_time) break;
  }

  return false;
}

bool TreeSolver::computePath(const Eigen::VectorXd& start_conf, const Eigen::VectorXd& goal_conf, const YAML::Node& config, PathPtr &solution, const double &max_time, const unsigned int &max_iter)
{
  NodePtr start_node = std::make_shared<Node>(start_conf,logger_);
  NodePtr goal_node  = std::make_shared<Node>(goal_conf ,logger_);

  return computePath(start_node,goal_node,config,solution,max_time,max_iter);
}

bool TreeSolver::computePath(const NodePtr &start_node, const NodePtr &goal_node, const YAML::Node& config, PathPtr &solution, const double &max_time, const unsigned int& max_iter)
{
  resetProblem();
  this->config(config);
  if(not addStart(start_node))
    return false;
  if(not addGoal(goal_node))
    return false;

  std::chrono::time_point<std::chrono::system_clock> tic = std::chrono::system_clock::now();
  if(!solve(solution, max_iter, max_time))
  {
    CNR_WARN(logger_,"No solutions found. Time: "<<std::chrono::duration<double>(std::chrono::system_clock::now()-tic).count()<<", max time: "<<max_time);
    return false;
  }
  return true;
}

bool TreeSolver::setSolution(const PathPtr &solution)
{
  if (not solution)
  {
    CNR_WARN(logger_,"Solution is empty");
    return false;
  }

  if(not solution ->getTree())
  {
    CNR_WARN(logger_,"Tree is empty");
    return false;
  }

  if(not config_)
  {
    CNR_WARN(logger_,"Solver not configured");
    return false;
  }

  double path_cost = solution->cost();
  double goal_cost = goal_cost_fcn_->cost(solution->getGoalNode());
  double cost = path_cost+goal_cost;

  if(cost< std::numeric_limits<double>::infinity())
  {
    solution_ = solution;
    start_tree_ = solution_->getTree();

    path_cost_ = path_cost;
    goal_node_ = solution->getGoalNode();
    goal_cost_ = goal_cost;
    cost_ = cost;

    best_utopia_ = goal_cost_+metrics_->utopia(start_tree_->getRoot()->getConfiguration(),goal_node_->getConfiguration());

    solved_ = true;
    completed_ = (cost_ <= (utopia_tolerance_ * best_utopia_));

    sampler_->setCost(path_cost_);

    init_ = true;

    CNR_DEBUG(logger_,"Solution set. Solved %d, completed %d, cost %f, utopia %f",solved_,completed_,cost_,best_utopia_*utopia_tolerance_);
    return true;
  }
  else
  {
    CNR_WARN(logger_,"Invalid solution, not set in the solver");
    return false;
  }
}

bool TreeSolver::importFromSolver(const TreeSolverPtr& solver)
{
  CNR_DEBUG(logger_,"Import from Tree solver");

  if(this == solver.get())// Avoid self-assignment
    return true;

  if(not config(solver->getConfig()))
  {
    CNR_ERROR(logger_,"Cannot import from the solver because the configuration failed");
    return false;
  }

  goal_cost_fcn_ = solver->goal_cost_fcn_;
  solved_ = solver->solved_;
  completed_ = solver->completed_;
  init_ = solver->init_;
  configured_ = solver->configured_;
  start_tree_ = solver->start_tree_;
  dof_ = solver->dof_;
  config_ = solver->config_;
  max_distance_ = solver->max_distance_;
  extend_ = solver->extend_;
  utopia_tolerance_ = solver->utopia_tolerance_;
  use_kdtree_ = solver->use_kdtree_;
  informed_ = solver->informed_;
  warp_ = solver->warp_;
  first_warp_ = solver->first_warp_;
  goal_node_ = solver->goal_node_;
  path_cost_ = solver->path_cost_;
  goal_cost_ = solver->goal_cost_;
  cost_ = solver->cost_;
  solution_ = solver->solution_;
  best_utopia_ = solver->best_utopia_;

  return true;
}

void TreeSolver::printMyself(std::ostream &os) const
{
  os << "Configured: " << configured();
  os << ". Problem set: " << init();

  if(init())
  {
    os << ".\n Start node: " << *start_tree_->getRoot();
    os << "\n Goal node: " << *goal_node_;
  }

  os << "\nSolved: " << solved();
  os << "\nCompleted: " << completed();

  if(solved())
  {
    os << "Cost: " << cost_;
    os << ". Path cost: " << path_cost_;
    os << ". Goal cost: " << goal_cost_;
    os << ".\n Path: " << *getSolution();
  }

}
} //end namespace core
} // end namespace graph
