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

namespace graph
{
namespace core
{

bool BiRRT::importFromSolver(const BiRRTPtr& solver)
{
  CNR_DEBUG(logger_,"Import from BiRRT solver");

  if(this == solver.get())// Avoid self-assignment
    return true;

  if(RRT::importFromSolver(std::static_pointer_cast<RRT>(solver)))
  {
    goal_tree_ = solver->goal_tree_;
    return true;
  }
  else
  {
    CNR_ERROR(logger_,"Import from solver failed");
    return false;
  }
}

bool BiRRT::importFromSolver(const TreeSolverPtr& solver)
{
  if(std::dynamic_pointer_cast<BiRRT>(solver) != nullptr)
  {
    return BiRRT::importFromSolver(std::static_pointer_cast<BiRRT>(solver));
  }
  else
  {
    return TreeSolver::importFromSolver(solver);
  }
}

bool BiRRT::addGoal(const NodePtr &goal_node, const double &max_time)
{
  goal_tree_ = std::make_shared<Tree>(goal_node, max_distance_, checker_, metrics_, logger_, use_kdtree_);

  return RRT::addGoal(goal_node, max_time);
}

bool BiRRT::update(PathPtr &solution)
{
  CNR_DEBUG(logger_,"RRTConnect::update");
  if (solved_)
  {
    CNR_DEBUG(logger_,"alreay found a solution");
    solution = solution_;
    return true;
  }

  if (sampler_->collapse())
  {
    CNR_DEBUG(logger_,"collapsed");
    return false;
  }

  Eigen::VectorXd configuration = sampler_->sample();

  return update(configuration, solution);
}

bool BiRRT::update(const Eigen::VectorXd& configuration, PathPtr& solution)
{
  CNR_DEBUG(logger_,"RRTConnect::update");
  if (solved_)
  {
    CNR_DEBUG(logger_,"already found a solution");
    solution = solution_;
    return true;
  }

  double distance;
  NodePtr new_start_node, new_goal_node;
  bool add_to_start, add_to_goal, tree_connected;

  add_to_start = extend_? start_tree_->extend(configuration, new_start_node):
                          start_tree_->connect(configuration, new_start_node);

  add_to_goal = extend_? goal_tree_->extend(configuration, new_goal_node):
                         goal_tree_->connect(configuration, new_goal_node);

  tree_connected = false;
  if(add_to_start && add_to_goal)
  {
    distance = (new_start_node->getConfiguration()-new_goal_node->getConfiguration()).norm();
    tree_connected = (distance<TOLERANCE);
    CNR_DEBUG(logger_,"Distance between node added to start tree and node added to goal tree: %f (tolerance %f)",distance,TOLERANCE);
  }

  if (tree_connected) // a solution is found
  {
    CNR_DEBUG(logger_,"Trees connected");

    NodePtr parent=new_goal_node->getParents().at(0);
    double cost_to_parent=new_goal_node->parentConnection(0)->getCost();
    std::chrono::time_point<std::chrono::system_clock> time_cost = new_goal_node->parentConnection(0)->getTimeCostUpdate();
    std::vector<ConnectionPtr> connections=goal_tree_->getConnectionToNode(parent);
    for (ConnectionPtr& conn: connections)
      conn->flip();

    ConnectionPtr conn_to_goal_parent=std::make_shared<Connection>(new_start_node,parent,logger_);
    conn_to_goal_parent->add();
    conn_to_goal_parent->setCost(cost_to_parent);
    conn_to_goal_parent->setTimeCostUpdate(time_cost);
    start_tree_->addNode(parent,false);
    for (ConnectionPtr& conn: connections)
      start_tree_->addNode(conn->getChild(),false);

    solution_ = std::make_shared<Path>(start_tree_->getConnectionToNode(goal_node_), metrics_, checker_, logger_);
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


bool BiRRT::update(const NodePtr& n, PathPtr& solution)
{
  CNR_DEBUG(logger_,"RRTConnect::update");
  if (solved_)
  {
    CNR_DEBUG(logger_,"alreay find a solution");
    solution = solution_;
    return true;
  }

  double distance;
  NodePtr new_start_node, new_goal_node;
  bool add_to_start, add_to_goal, tree_connected;

  add_to_start = extend_? start_tree_->extendToNode(n, new_start_node):
                          start_tree_->connectToNode(n, new_start_node);

  add_to_goal = extend_? goal_tree_->extendToNode(n, new_goal_node):
                         goal_tree_->connectToNode(n, new_goal_node);

  tree_connected = false;
  if(add_to_start && add_to_goal)
  {
    distance = (new_start_node->getConfiguration()-new_goal_node->getConfiguration()).norm();
    tree_connected = (distance<TOLERANCE);
    CNR_DEBUG(logger_,"Distance between node added to start tree and node added to goal tree: %f (tolerance %f)",distance,TOLERANCE);
  }

  if (tree_connected) // a solution is found
  {
    CNR_DEBUG(logger_,"Trees connected");

    NodePtr parent=new_goal_node->getParents().at(0);
    double cost_to_parent=new_goal_node->parentConnection(0)->getCost();
    std::chrono::time_point<std::chrono::system_clock> time_cost = new_goal_node->parentConnection(0)->getTimeCostUpdate();
    new_goal_node->disconnect();

    std::vector<ConnectionPtr> connections=goal_tree_->getConnectionToNode(parent);
    for (ConnectionPtr& conn: connections)
      conn->flip();

    ConnectionPtr conn_to_goal_parent=std::make_shared<Connection>(new_start_node,parent,logger_);
    conn_to_goal_parent->add();
    conn_to_goal_parent->setCost(cost_to_parent);
    conn_to_goal_parent->setTimeCostUpdate(time_cost);
    start_tree_->addNode(parent,false);
    for (ConnectionPtr& conn: connections)
      start_tree_->addNode(conn->getChild(),false);

    solution_ = std::make_shared<Path>(start_tree_->getConnectionToNode(goal_node_), metrics_, checker_, logger_);
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

} //end namespace core
} // end namespace graph



#include <cnr_class_loader/register_macro.hpp>
CLASS_LOADER_REGISTER_CLASS(graph::core::BiRRT, graph::core::TreeSolver)
