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

#include <graph_core/solvers/rrt_star.h>

namespace pathplan
{

bool RRTStar::addStartTree(const TreePtr &start_tree)
{
  assert(start_tree);
  start_tree_ = start_tree;


  if (goal_node_)
  {
    utopia_ = goal_cost_+(goal_node_->getConfiguration() - start_tree_->getRoot()->getConfiguration()).norm();
    init_ = true;
    solution_ = std::make_shared<Path>(start_tree_->getConnectionToNode(goal_node_), metrics_, checker_);
    solution_->setTree(start_tree_);
    path_cost_ = solution_->cost();
    cost_=path_cost_+goal_cost_;
  }
  else
    init_ = false;

  return true;
}


bool RRTStar::addGoal(const NodePtr &goal_node)
{
  solved_ = false;
  goal_node_ = goal_node;
  goal_cost_ = goal_cost_fcn_->cost(goal_node);
  if (start_tree_)
  {
    utopia_ = goal_cost_+(goal_node_->getConfiguration() - start_tree_->getRoot()->getConfiguration()).norm();
    init_ = true;

    solution_ = std::make_shared<Path>(start_tree_->getConnectionToNode(goal_node_), metrics_, checker_);
    solution_->setTree(start_tree_);
    path_cost_ = solution_->cost();
    cost_=path_cost_+goal_cost_;
  }
  else
    init_ = false;
  return true;
}
bool RRTStar::config(const ros::NodeHandle& nh)
{
  nh_ = nh;
  dof_=sampler_->getDimension();
  r_rewire_factor_ = 1.1* std::pow( 2*(1+1/dof_), 1./dof_);
  solved_ = true;
  return true;
}

bool RRTStar::update(PathPtr& solution)
{
  if (!init_)
    return false;
  if (cost_ <= 1.003 * utopia_)
  {
    completed_=true;
    solution=solution_;
    return true;
  }

  return update(sampler_->sample(), solution);

}


bool RRTStar::update(const Eigen::VectorXd& point, PathPtr& solution)
{
  if (!init_)
    return false;
  if (path_cost_ <= 1.003 * utopia_)
  {
    ROS_INFO("Already optimal");
    solution=solution_;
    completed_=true;
    return true;
  }
  double cost = solution_->cost();
//  double r_rewire = std::min(start_tree_->getMaximumDistance(), r_rewire_factor_ * sampler_->getSpecificVolume() * std::pow(std::log(start_tree_->getNumberOfNodes())/start_tree_->getNumberOfNodes(),1./dof_));
  double r_rewire = start_tree_->getMaximumDistance();
  bool improved = start_tree_->rewire(point, r_rewire);
  if (improved)
  {
    if (start_tree_->costToNode(goal_node_) >= (cost - 1e-8))
      return false;

    solution_ = std::make_shared<Path>(start_tree_->getConnectionToNode(goal_node_), metrics_, checker_);
    solution_->setTree(start_tree_);

    path_cost_ = solution_->cost();
    cost_ = path_cost_+goal_cost_;

    sampler_->setCost(path_cost_);
  }
  solution = solution_;
  return improved;
}


bool RRTStar::update(const NodePtr& n, PathPtr& solution)
{
  if (!init_)
    return false;
  if (path_cost_ <= 1.003 * utopia_)
  {
    completed_=true;
    solution=solution_;
    return true;
  }
  double cost = solution_->cost();
  //double r_rewire = std::min(start_tree_->getMaximumDistance(), r_rewire_factor_ * sampler_->getSpecificVolume() * std::pow(std::log(start_tree_->getNumberOfNodes())/start_tree_->getNumberOfNodes(),1./dof_));
  double r_rewire = start_tree_->getMaximumDistance();
  bool improved = start_tree_->rewireToNode(n, r_rewire);

  if (improved)
  {
    if (start_tree_->costToNode(goal_node_) >= (cost - 1e-8))
      return false;

    solution_ = std::make_shared<Path>(start_tree_->getConnectionToNode(goal_node_), metrics_, checker_);
    solution_->setTree(start_tree_);

    path_cost_ = solution_->cost();


    sampler_->setCost(path_cost_);
  }
  solution = solution_;
  return improved;
}

bool RRTStar::solve(PathPtr &solution, const unsigned int& max_iter)
{
  bool improved = false;
  for (unsigned int iter = 0; iter < max_iter; iter++)
  {
    if (update(solution))
    {
      ROS_DEBUG("Improved in %u iterations", iter);
      solved_ = true;
      improved = true;
    }
  }
  return improved;
}

TreeSolverPtr RRTStar::clone(const MetricsPtr& metrics, const CollisionCheckerPtr& checker, const SamplerPtr& sampler)
{
  return std::make_shared<RRTStar>(metrics,checker,sampler);
}

}
