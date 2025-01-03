/*
Copyright (c) 2024, Manuel Beschi and Cesare Tonola, JRL-CARI CNR-STIIMA/UNIBS,
manuel.beschi@unibs.it, c.tonola001@unibs.it All rights reserved.

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
bool TreeSolver::config(const std::string& param_ns)
{
  param_ns_ = param_ns;
  get_param(logger_, param_ns_, "max_distance", max_distance_, 1.0);
  get_param(logger_, param_ns_, "use_kdtree", use_kdtree_, true);
  get_param(logger_, param_ns_, "extend", extend_, false);
  get_param(logger_, param_ns_, "utopia_tolerance", utopia_tolerance_, 0.01);

  if (utopia_tolerance_ <= 0.0)
  {
    CNR_WARN(logger_, "utopia_tolerance cannot be negative, set equal to 0.0");
    utopia_tolerance_ = 0.0;
  }
  utopia_tolerance_ += 1.0;

  dof_ = sampler_->getDimension();
  configured_ = true;
  can_improve_ = true;
  return true;
}

bool TreeSolver::setProblem(const double& max_time)
{
  problem_set_ = false;
  if (!start_tree_)
    return false;
  if (!goal_node_)
    return false;

  can_improve_ = true;
  goal_cost_ = goal_cost_fcn_->cost(goal_node_);

  best_utopia_ =
      goal_cost_ + metrics_->utopia(start_tree_->getRoot()->getConfiguration(), goal_node_->getConfiguration());
  problem_set_ = true;

  if (start_tree_->isInTree(goal_node_))
  {
    solution_ = std::make_shared<Path>(start_tree_->getConnectionToNode(goal_node_), metrics_, checker_, logger_);
    solution_->setTree(start_tree_);

    path_cost_ = solution_->cost();
    sampler_->setCost(path_cost_);
    start_tree_->addNode(goal_node_);

    solved_ = true;
    cost_ = path_cost_ + goal_cost_;
    return true;
  }

  NodePtr new_node;

  if (start_tree_->connectToNode(goal_node_, new_node,
                                 max_time))  // for direct connection to goal
  {
    solution_ = std::make_shared<Path>(start_tree_->getConnectionToNode(goal_node_), metrics_, checker_, logger_);
    solution_->setTree(start_tree_);

    path_cost_ = solution_->cost();
    sampler_->setCost(path_cost_);
    start_tree_->addNode(goal_node_);

    solved_ = true;
    CNR_TRACE(logger_, "A direct solution is found\n" << *solution_);
  }
  else
  {
    path_cost_ = std::numeric_limits<double>::infinity();
  }
  cost_ = path_cost_ + goal_cost_;
  return true;
}

bool TreeSolver::solve(PathPtr& solution, const unsigned int& max_iter, const double& max_time)
{
  if (not initialized_)
    return false;

  auto tic = graph_time::now();

  if (max_time <= 0.0)
    return false;

  for (unsigned int iter = 0; iter < max_iter; iter++)
  {
    if (update(solution))
    {
      solved_ = true;
      return true;
    }

    if (toSeconds(graph_time::now(), tic) >= 0.98 * max_time)
      break;
  }

  return false;
}

bool TreeSolver::computePath(const Eigen::VectorXd& start_conf, const Eigen::VectorXd& goal_conf,
                             const std::string& param_ns, PathPtr& solution, const double& max_time,
                             const unsigned int& max_iter)
{
  NodePtr start_node = std::make_shared<Node>(start_conf, logger_);
  NodePtr goal_node = std::make_shared<Node>(goal_conf, logger_);

  return computePath(start_node, goal_node, param_ns, solution, max_time, max_iter);
}

bool TreeSolver::computePath(const NodePtr& start_node, const NodePtr& goal_node, const std::string& param_ns,
                             PathPtr& solution, const double& max_time, const unsigned int& max_iter)
{
  resetProblem();
  this->config(param_ns);
  if (not addStart(start_node))
    return false;
  if (not addGoal(goal_node))
    return false;

  finalizeProblem();

  auto tic = graph_time::now();
  if (!solve(solution, max_iter, max_time))
  {
    CNR_WARN(logger_, "No solutions found. Time: " << toSeconds(graph_time::now(), tic) << ", max time: " << max_time);
    return false;
  }
  return true;
}

bool TreeSolver::setSolution(const PathPtr& solution)
{
  if (not solution)
  {
    CNR_WARN(logger_, "Solution is empty");
    return false;
  }

  if (not solution->getTree())
  {
    CNR_WARN(logger_, "Tree is empty");
    return false;
  }

  if (not configured_)
  {
    CNR_WARN(logger_, "Solver not configured");
    return false;
  }

  double path_cost = solution->cost();
  double goal_cost = goal_cost_fcn_->cost(solution->getGoalNode());
  double cost = path_cost + goal_cost;

  if (cost < std::numeric_limits<double>::infinity())
  {
    solution_ = solution;
    start_tree_ = solution_->getTree();

    path_cost_ = path_cost;
    goal_node_ = solution->getGoalNode();
    goal_cost_ = goal_cost;
    cost_ = cost;

    best_utopia_ =
        goal_cost_ + metrics_->utopia(start_tree_->getRoot()->getConfiguration(), goal_node_->getConfiguration());

    solved_ = true;
    can_improve_ = not(cost_ <= (utopia_tolerance_ * best_utopia_));

    sampler_->setCost(path_cost_);

    problem_set_ = true;

    CNR_DEBUG(logger_, "Solution set. Solved %d, can improve? %d, cost %f, utopia %f", solved_, can_improve_, cost_,
              best_utopia_ * utopia_tolerance_);
    return true;
  }
  else
  {
    CNR_WARN(logger_, "Invalid solution, not set in the solver");
    return false;
  }
}

bool TreeSolver::importFromSolver(const TreeSolverPtr& solver)
{
  CNR_DEBUG(logger_, "Import from Tree solver");

  if (this == solver.get())  // Avoid self-assignment
    return true;

  if (not config(solver->param_ns_))
  {
    CNR_ERROR(logger_, "Cannot import from the solver because the configuration failed");
    return false;
  }

  goal_cost_fcn_ = solver->goal_cost_fcn_;
  solved_ = solver->solved_;
  can_improve_ = solver->can_improve_;
  initialized_ = solver->initialized_;
  problem_set_ = solver->problem_set_;
  configured_ = solver->configured_;
  start_tree_ = solver->start_tree_;
  dof_ = solver->dof_;
  param_ns_ = solver->param_ns_;
  max_distance_ = solver->max_distance_;
  extend_ = solver->extend_;
  utopia_tolerance_ = solver->utopia_tolerance_;
  use_kdtree_ = solver->use_kdtree_;
  goal_node_ = solver->goal_node_;
  path_cost_ = solver->path_cost_;
  goal_cost_ = solver->goal_cost_;
  cost_ = solver->cost_;
  solution_ = solver->solution_;
  best_utopia_ = solver->best_utopia_;

  return true;
}

void TreeSolver::printMyself(std::ostream& os) const
{
  os << "Configured: " << configured();
  os << ". Problem set: " << problemStatus();

  if (problemStatus())
  {
    os << ".\nStart node: " << *start_tree_->getRoot();
    os << ".\nGoal node: " << *goal_node_;
  }

  os << ".\nSolved: " << solved();
  os << ". Can improve? " << canImprove();

  if (solved())
  {
    os << ". Cost: " << cost_;
    os << ". Path cost: " << path_cost_;
    os << ". Goal cost: " << goal_cost_;
    os << ".\n Path: " << *getSolution();
  }
}
}  // end namespace core
}  // end namespace graph
