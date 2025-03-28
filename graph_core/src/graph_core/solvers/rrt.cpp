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

#include <graph_core/solvers/rrt.h>

namespace graph
{
namespace core
{
bool RRT::addGoal(const NodePtr& goal_node, const double& max_time)
{
  if (!configured_)
  {
    CNR_ERROR(logger_, "Solver is not configured!");
    return false;
  }

  if (not sampler_->inBounds(goal_node->getConfiguration()))
  {
    CNR_WARN(logger_, "Goal not in bounds");
    return false;
  }

  solved_ = false;
  goal_node_ = goal_node;

  goal_cost_ = goal_cost_fcn_->cost(goal_node);

  setProblem(max_time);

  return true;
}

bool RRT::addStart(const NodePtr& start_node, const double& max_time)
{
  if (!configured_)
  {
    CNR_ERROR(logger_, "Solver is not configured!");
    return false;
  }

  if (not sampler_->inBounds(start_node->getConfiguration()))
  {
    CNR_WARN(logger_, "Start not in bounds");
    return false;
  }

  solved_ = false;
  start_tree_ = std::make_shared<Tree>(start_node, max_distance_, checker_, metrics_, logger_, use_kdtree_);

  setProblem(max_time);

  return true;
}

bool RRT::addStartTree(const TreePtr& start_tree, const double& max_time)
{
  assert(start_tree);
  start_tree_ = start_tree;
  solved_ = false;

  setProblem(max_time);
  return true;
}
void RRT::resetProblem()
{
  goal_node_.reset();
  start_tree_.reset();
  problem_set_ = false;
  solved_ = false;
  can_improve_ = true;
}

bool RRT::update(PathPtr& solution)
{
  CNR_TRACE(logger_, "RRT::update");

  if (solved_)
  {
    CNR_DEBUG(logger_, "already found a solution");
    solution = solution_;
    can_improve_ = false;
    return true;
  }

  if (sampler_->collapse())
    return false;

  return RRT::update(sampler_->sample(), solution);
}

bool RRT::update(const Eigen::VectorXd& configuration, PathPtr& solution)
{
  CNR_TRACE(logger_, "RRT::update");

  if (solved_)
  {
    CNR_DEBUG(logger_, "already found a solution");
    solution = solution_;
    can_improve_ = false;
    return true;
  }

  NodePtr new_start_node;
  bool add_to_start;
  add_to_start = extend_ ? start_tree_->extend(configuration, new_start_node) :
                           start_tree_->connect(configuration, new_start_node);

  if (add_to_start)
  {
    if ((new_start_node->getConfiguration() - goal_node_->getConfiguration()).norm() <= max_distance_)
    {
      if (checker_->checkConnection(new_start_node->getConfiguration(), goal_node_->getConfiguration()))
      {
        ConnectionPtr conn = std::make_shared<Connection>(new_start_node, goal_node_, logger_);
        conn->setCost(metrics_->cost(new_start_node, goal_node_));
        conn->add();
        solution_ = std::make_shared<Path>(start_tree_->getConnectionToNode(goal_node_), metrics_, checker_, logger_);
        solution_->setTree(start_tree_);
        start_tree_->addNode(goal_node_);
        path_cost_ = solution_->cost();
        cost_ = path_cost_ + goal_cost_;
        sampler_->setCost(path_cost_);
        solution = solution_;
        solved_ = true;
        can_improve_ = false;
        return true;
      }
    }
  }

  return false;
}

bool RRT::update(const NodePtr& n, PathPtr& solution)
{
  CNR_DEBUG(logger_, "RRT::update");

  if (solved_)
  {
    CNR_DEBUG(logger_, "already found a solution");
    solution = solution_;
    return true;
  }

  NodePtr new_start_node;
  bool add_to_start =
      extend_ ? start_tree_->extendToNode(n, new_start_node) : start_tree_->connectToNode(n, new_start_node);

  if (add_to_start)
  {
    if ((new_start_node->getConfiguration() - goal_node_->getConfiguration()).norm() <= max_distance_)
    {
      if (checker_->checkConnection(new_start_node->getConfiguration(), goal_node_->getConfiguration()))
      {
        ConnectionPtr conn = std::make_shared<Connection>(new_start_node, goal_node_, logger_);
        conn->setCost(metrics_->cost(new_start_node, goal_node_));
        conn->add();
        solution_ = std::make_shared<Path>(start_tree_->getConnectionToNode(goal_node_), metrics_, checker_, logger_);
        solution_->setTree(start_tree_);
        start_tree_->addNode(goal_node_);
        path_cost_ = solution_->cost();
        cost_ = path_cost_ + goal_cost_;
        sampler_->setCost(path_cost_);
        solution = solution_;
        solved_ = true;
        can_improve_ = false;
        return true;
      }
    }
  }
  return false;
}

}  // end namespace core
}  // end namespace graph
