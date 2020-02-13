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
  max_distance_=0.4;
}

bool RRTConnect::addGoal(const NodePtr &goal_node)
{
  solved_=false;
  goal_node_=goal_node;
  setProblem();
  return true;
}

bool RRTConnect::addStart(const NodePtr &start_node)
{
  solved_=false;
  start_tree_=std::make_shared<Tree>(start_node,Forward,max_distance_,checker_,metrics_);
  setProblem();
  return true;
}

bool RRTConnect::addStartTree(const TreePtr &start_tree)
{
  assert(start_tree);
  start_tree_=start_tree;
  solved_=false;

  setProblem();
  return true;
}

bool RRTConnect::setProblem()
{
  if (!start_tree_)
    return false;
  if (!goal_node_)
    return false;

  utopia_=(goal_node_->getConfiguration()-start_tree_->getRoot()->getConfiguration()).norm();
  init_=true;
  NodePtr new_node;
  if (start_tree_->connectToNode(goal_node_,new_node))
  {
    solution_=std::make_shared<Path>(start_tree_->getConnectionToNode(goal_node_),metrics_,checker_);
    cost_=solution_->cost();
    sampler_->setCost(cost_);
    start_tree_->addNode(goal_node_);
    solved_=true;
    ROS_FATAL_STREAM("A direct solution is found\n"<<*solution_);
  }
  else
  {
    cost_=std::numeric_limits<double>::infinity();
  }
}

bool RRTConnect::update(PathPtr &solution)
{
  if (solved_)
  {
    solution=solution_;
    return true;
  }
  NodePtr new_node;
  if (start_tree_->connect(sampler_->sample(),new_node))
  {

    if ((new_node->getConfiguration()-goal_node_->getConfiguration()).norm()<max_distance_)
    {
      if (checker_->checkPath(new_node->getConfiguration(),goal_node_->getConfiguration()))
      {
        ConnectionPtr conn=std::make_shared<Connection>(new_node,goal_node_);
        conn->setCost(metrics_->cost(new_node,goal_node_));
        conn->add();
        solution_=std::make_shared<Path>(start_tree_->getConnectionToNode(goal_node_),metrics_,checker_);
        start_tree_->addNode(goal_node_);
        cost_=solution_->cost();
        sampler_->setCost(cost_);
        solution=solution_;
        solved_=true;
        return true;
      }
    }
  }

}

}


