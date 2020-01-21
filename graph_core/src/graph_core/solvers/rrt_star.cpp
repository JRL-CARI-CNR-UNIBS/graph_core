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

namespace pathplan {

bool RRTStar::addStartTree(const TreePtr &start_tree)
{
  assert(start_tree);
  start_tree_=start_tree;


  if (goal_node_)
  {
    utopia_=(goal_node_->getConfiguration()-start_tree_->getRoot()->getConfiguration()).norm();
    init_=true;
    solution_=std::make_shared<Path>(start_tree_->getConnectionToNode(goal_node_),metrics_,checker_);
  }
  else
    init_=false;
}


bool RRTStar::addGoal(const NodePtr &goal_node)
{
  solved_=false;
  goal_node_=goal_node;

  if (start_tree_)
  {
    utopia_=(goal_node_->getConfiguration()-start_tree_->getRoot()->getConfiguration()).norm();
    init_=true;

    solution_=std::make_shared<Path>(start_tree_->getConnectionToNode(goal_node_),metrics_,checker_);

  }
  else
    init_=false;
  return true;
}
bool RRTStar::config(const ros::NodeHandle& nh)
{
  r_rewire_=1;
  solved_=true;

}

bool RRTStar::update(PathPtr& solution)
{
  if (!init_)
    return false;
  if (cost_<=1.003*utopia_)
    return true;

  bool improved= start_tree_->rewire(sampler_->sample(),r_rewire_);

  if (improved)
  {
    solution_=std::make_shared<Path>(start_tree_->getConnectionToNode(goal_node_),metrics_,checker_);
    cost_=solution_->cost();
    sampler_->setCost(cost_);
  }
  solution=solution_;
  return improved;
}

bool RRTStar::solve(PathPtr &solution, const unsigned int& max_iter)
{
  bool improved=false;
  for (unsigned int iter=0;iter<max_iter;iter++)
  {
    if (update(solution))
    {
      ROS_DEBUG("Improved in %u iterations",iter);
      solved_=true;
      improved=true;
    }
  }
  return improved;
}

}
