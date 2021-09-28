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


#include <graph_core/solvers/birrt.h>

namespace pathplan
{

bool BiRRT::config(const ros::NodeHandle &nh)
{
  nh_ = nh;
  extend_ = false;
  return RRTConnect::config(nh);

}
bool BiRRT::addGoal(const NodePtr &goal_node, const double &max_time)
{
  goal_tree_ = std::make_shared<Tree>(goal_node, Backward, max_distance_, checker_, metrics_);

  return RRTConnect::addGoal(goal_node, max_time);
}

bool BiRRT::update(PathPtr &solution)
{
  PATH_COMMENT("RRTConnect::update");
  if (solved_)
  {
    PATH_COMMENT("alreay found a solution");
    solution = solution_;
    return true;
  }

  if (sampler_->collapse())
  {
    PATH_COMMENT("collapsed");
    return false;
  }


  Eigen::VectorXd configuration = sampler_->sample();

  return update(configuration, solution);
}

bool BiRRT::update(const Eigen::VectorXd& point, PathPtr& solution)
{
  PATH_COMMENT("RRTConnect::update");
  if (solved_)
  {
    PATH_COMMENT("alreay find a solution");
    solution = solution_;
    return true;
  }


  NodePtr new_start_node, new_goal_node;
  bool add_to_start, add_to_goal;

  Eigen::VectorXd configuration = point;
  if (extend_)
    add_to_start = start_tree_->extend(configuration, new_start_node);
  else
    add_to_start = start_tree_->connect(configuration, new_start_node);


  if (add_to_start)
  {
    if (extend_)
      add_to_goal = goal_tree_->extendToNode(new_start_node, new_goal_node);
    else
      add_to_goal = goal_tree_->connectToNode(new_start_node, new_goal_node);
  }
  else
  {
    if (extend_)
      add_to_goal = goal_tree_->extend(configuration, new_goal_node);
    else
      add_to_goal = goal_tree_->connect(configuration, new_goal_node);
  }

  if (add_to_start && add_to_goal && new_goal_node == new_start_node)
  {
    std::vector<ConnectionPtr> goal_subpath = goal_tree_->getConnectionToNode(new_goal_node);
    goal_tree_->keepOnlyThisBranch(goal_subpath);
    start_tree_->addBranch(goal_subpath);

    solution_ = std::make_shared<Path>(start_tree_->getConnectionToNode(goal_node_), metrics_, checker_);
    solution_->setTree(start_tree_);
    path_cost_ = solution_->cost();
    cost_=path_cost_+goal_cost_;
    sampler_->setCost(path_cost_);
    solution = solution_;
    solved_ = true;
    return true;
  }
  return false;
}

void BiRRT::clean()
{
  goal_tree_->keepOnlyThisBranch(solution_->getConnections());
}


bool BiRRT::update(const NodePtr& n, PathPtr& solution)
{
  PATH_COMMENT("RRTConnect::update");
  if (solved_)
  {
    PATH_COMMENT("alreay find a solution");
    solution = solution_;
    return true;
  }

  NodePtr new_start_node, new_goal_node;
  bool add_to_start, add_to_goal;
  if (extend_)
    add_to_start = start_tree_->extendToNode(n, new_start_node);
  else
    add_to_start = start_tree_->connectToNode(n, new_start_node);


  if (add_to_start)
  {
    if (extend_)
      add_to_goal = goal_tree_->extendToNode(new_start_node, new_goal_node);
    else
      add_to_goal = goal_tree_->connectToNode(new_start_node, new_goal_node);
  }
  else
  {
    if (extend_)
      add_to_goal = goal_tree_->extendToNode(n, new_goal_node);
    else
      add_to_goal = goal_tree_->connectToNode(n, new_goal_node);
  }

  if (add_to_start && add_to_goal && new_goal_node == new_start_node)
  {
    std::vector<ConnectionPtr> goal_subpath = goal_tree_->getConnectionToNode(new_goal_node);
    goal_tree_->keepOnlyThisBranch(goal_subpath);
    start_tree_->addBranch(goal_subpath);

    solution_ = std::make_shared<Path>(start_tree_->getConnectionToNode(goal_node_), metrics_, checker_);
    solution_->setTree(start_tree_);
    path_cost_ = solution_->cost();
    cost_=path_cost_+goal_cost_;
    sampler_->setCost(path_cost_);
    solution = solution_;
    solved_ = true;
    return true;

  }
  return false;
}

TreeSolverPtr BiRRT::clone(const MetricsPtr& metrics, const CollisionCheckerPtr& checker, const SamplerPtr& sampler)
{
  return std::make_shared<BiRRT>(metrics,checker,sampler);
}

}
