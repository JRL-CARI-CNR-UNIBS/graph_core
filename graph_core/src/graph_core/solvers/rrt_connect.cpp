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

#include <graph_core/solvers/rrt_connect.h>

namespace pathplan
{

bool RRTConnect::config(const ros::NodeHandle& nh)
{
  nh_ = nh;
  max_distance_ = 1;
  configured_=true;
  return true;
}

bool RRTConnect::addGoal(const NodePtr &goal_node, const double &max_time)
{
  if (!configured_)
  {
    ROS_ERROR("Solver is not configured.");
    return false;
  }
  solved_ = false;
  goal_node_ = goal_node;
  goal_cost_=goal_cost_fcn_->cost(goal_node);
  setProblem(max_time);

  return true;
}

bool RRTConnect::addStart(const NodePtr &start_node, const double &max_time)
{
  if (!configured_)
  {
    ROS_ERROR("Solver is not configured.");
    return false;
  }
  solved_ = false;
  start_tree_ = std::make_shared<Tree>(start_node, Forward, max_distance_, checker_, metrics_);

  setProblem(max_time);

  return true;
}

bool RRTConnect::addStartTree(const TreePtr &start_tree)
{
  assert(start_tree);
  start_tree_ = start_tree;
  solved_ = false;

  setProblem();
  return true;
}
void RRTConnect::resetProblem()
{
  goal_node_.reset();
  start_tree_.reset();
  solved_=false;
}

bool RRTConnect::setProblem(const double &max_time)
{
  if (!start_tree_)
    return false;
  if (!goal_node_)
    return false;

  utopia_ = (goal_node_->getConfiguration() - start_tree_->getRoot()->getConfiguration()).norm();
  init_ = true;
  NodePtr new_node;

  if (start_tree_->connectToNode(goal_node_, new_node, max_time))
  {
    solution_ = std::make_shared<Path>(start_tree_->getConnectionToNode(goal_node_), metrics_, checker_);
    solution_->setTree(start_tree_);

    path_cost_ = solution_->cost();

    sampler_->setCost(path_cost_);
    start_tree_->addNode(goal_node_);

    solved_ = true;
    PATH_COMMENT_STREAM("A direct solution is found\n" << *solution_);
  }
  else
  {
    path_cost_ = std::numeric_limits<double>::infinity();
  }
  cost_=path_cost_+goal_cost_;
  return true;
}

bool RRTConnect::update(PathPtr &solution)
{
  PATH_COMMENT("RRTConnect::update");

  if (solved_)
  {
    PATH_COMMENT("already found a solution");
    solution = solution_;
    return true;
  }

  if (sampler_->collapse())
    return false;

  return update(sampler_->sample(), solution);
}

bool RRTConnect::update(const Eigen::VectorXd& point, PathPtr &solution)
{
  PATH_COMMENT("RRTConnect::update");

  if (solved_)
  {
    PATH_COMMENT("already found a solution");
    solution = solution_;
    return true;
  }



  NodePtr new_node;
  if (start_tree_->connect(point, new_node))
  {

    if ((new_node->getConfiguration() - goal_node_->getConfiguration()).norm() < max_distance_)
    {
      if (checker_->checkPath(new_node->getConfiguration(), goal_node_->getConfiguration()))
      {
        ConnectionPtr conn = std::make_shared<Connection>(new_node, goal_node_);
        conn->setCost(metrics_->cost(new_node, goal_node_));
        conn->add();
        solution_ = std::make_shared<Path>(start_tree_->getConnectionToNode(goal_node_), metrics_, checker_);
        solution_->setTree(start_tree_);
        start_tree_->addNode(goal_node_);
        path_cost_ = solution_->cost();
        cost_=path_cost_+goal_cost_;
        sampler_->setCost(path_cost_);
        solution = solution_;
        solved_ = true;
        return true;
      }
    }
  }
  return false;

}


bool RRTConnect::update(const NodePtr& n, PathPtr &solution)
{
  PATH_COMMENT("RRTConnect::update");

  if (solved_)
  {
    PATH_COMMENT("already found a solution");
    solution = solution_;
    return true;
  }

  NodePtr new_node;
  if (start_tree_->connectToNode(n, new_node))
  {

    if ((new_node->getConfiguration() - goal_node_->getConfiguration()).norm() < max_distance_)
    {
      if (checker_->checkPath(new_node->getConfiguration(), goal_node_->getConfiguration()))
      {
        ConnectionPtr conn = std::make_shared<Connection>(new_node, goal_node_);
        conn->setCost(metrics_->cost(new_node, goal_node_));
        conn->add();
        solution_ = std::make_shared<Path>(start_tree_->getConnectionToNode(goal_node_), metrics_, checker_);
        solution_->setTree(start_tree_);
        start_tree_->addNode(goal_node_);
        path_cost_ = solution_->cost();
        cost_=path_cost_+goal_cost_;
        sampler_->setCost(path_cost_);
        solution = solution_;
        solved_ = true;
        return true;
      }
    }
  }
  return false;

}

TreeSolverPtr RRTConnect::clone(const MetricsPtr& metrics, const CollisionCheckerPtr& checker, const SamplerPtr& sampler)
{
  return std::make_shared<RRTConnect>(metrics,checker,sampler);
}


}


