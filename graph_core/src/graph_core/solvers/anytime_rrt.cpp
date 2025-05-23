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

#include <graph_core/solvers/anytime_rrt.h>

namespace graph
{
namespace core
{
bool AnytimeRRT::importFromSolver(const AnytimeRRTPtr& solver)
{
  CNR_DEBUG(logger_, "Import from AnytimeRRT solver");

  if (this == solver.get())  // Avoid self-assignment
    return true;

  if (RRT::importFromSolver(std::static_pointer_cast<RRT>(solver)))
  {
    bias_ = solver->bias_;
    delta_ = solver->delta_;
    cost_impr_ = solver->cost_impr_;
    cost2beat_ = solver->cost2beat_;
    improve_sampler_ = solver->improve_sampler_;
    new_tree_ = solver->new_tree_;
    tmp_goal_node_ = solver->tmp_goal_node_;

    return true;
  }
  else
  {
    CNR_ERROR(logger_, "Import from solver failed");
    return false;
  }
}

bool AnytimeRRT::importFromSolver(const TreeSolverPtr& solver)
{
  if (std::dynamic_pointer_cast<AnytimeRRT>(solver) != nullptr)
  {
    return AnytimeRRT::importFromSolver(std::static_pointer_cast<AnytimeRRT>(solver));
  }
  else
  {
    return TreeSolver::importFromSolver(solver);
  }
}

bool AnytimeRRT::solve(PathPtr& solution, const unsigned int& max_iter, const double& max_time)
{
  if (not initialized_)
    return false;

  auto tic = graph_time::now();
  double time;

  if (solved_)
  {
    solution = solution_;

    if (cost_ <= utopia_tolerance_ * best_utopia_)
    {
      CNR_TRACE(logger_, "Utopia reached!");
      can_improve_ = false;
      return true;
    }
  }

  if (max_time <= 0.0)
    return false;

  // RRT to find quickly a first sub-optimal solution
  bool success;
  unsigned int n_failed_iter = 0;

  time = toSeconds(graph_time::now(), tic);
  while (time < 0.98 * max_time && (not solved_) && n_failed_iter < FAILED_ITER)
  {
    success = RRT::solve(solution, max_iter, (max_time - time));

    CNR_TRACE(logger_, "Tree has " << start_tree_->getNumberOfNodes() << " nodes");

    if (not success)
      n_failed_iter++;

    time = toSeconds(graph_time::now(), tic);
    CNR_TRACE(logger_, "time " << time);
  }

  if (not solved_)
    return false;

  CNR_TRACE(logger_, "Path cost: " << path_cost_);

  if (cost_ <= utopia_tolerance_ * best_utopia_)
  {
    CNR_TRACE(logger_, "Utopia reached!");
    can_improve_ = false;
    return true;
  }

  // Informed trees to find better solutions
  NodePtr initial_start_node = start_tree_->getRoot();
  NodePtr initial_goal_node = goal_node_;

  bool improved;
  bool better_solution_found = false;
  n_failed_iter = 0;

  time = toSeconds(graph_time::now(), tic);
  while (time < 0.98 * max_time && (can_improve_) && n_failed_iter < FAILED_ITER)
  {
    NodePtr tmp_start_node = std::make_shared<Node>(initial_start_node->getConfiguration(), logger_);
    NodePtr tmp_goal_node = std::make_shared<Node>(initial_goal_node->getConfiguration(), logger_);
    improved = AnytimeRRT::improve(tmp_start_node, tmp_goal_node, solution, max_iter, (max_time - time));

    CNR_TRACE(logger_, "New tree has " << new_tree_->getNumberOfNodes() << " nodes");

    improved ? (n_failed_iter = 0) : (n_failed_iter++);

    if (improved)
    {
      better_solution_found = true;
      CNR_TRACE(logger_, "tmp_start " << *tmp_start_node);
      CNR_TRACE(logger_, "tmp_goal  " << *tmp_goal_node);

      assert([&]() -> bool {
        if (start_tree_ != new_tree_)
        {
          CNR_TRACE(logger_, "start tree " << start_tree_ << " new tree " << new_tree_);
          return false;
        }
        return true;
      }());
    }

    if (cost_ <= utopia_tolerance_ * best_utopia_)
    {
      CNR_TRACE(logger_, "Utopia reached!");
      can_improve_ = false;
      break;
    }

    time = toSeconds(graph_time::now(), tic);
    CNR_TRACE(logger_, "time " << time);
  }

  CNR_TRACE(logger_, "Better solution found? " << better_solution_found);

  // Start and goal ptr in start tree may be different from the ones given as
  // input to the solver. Replace the actual start and goal ptr with the desired
  // ones.

  if (better_solution_found && goal_node_ != initial_goal_node)
  {
    // Rewire the tree goal (set goal_node)
    initial_goal_node->disconnect();

    ConnectionPtr conn =
        std::make_shared<Connection>(solution_->getConnections().back()->getParent(), initial_goal_node, logger_);
    conn->setCost(solution_->getConnections().back()->getCost());
    conn->add();

    start_tree_->removeNode(goal_node_);  // tpm_goal_node
    goal_node_ = initial_goal_node;
    start_tree_->addNode(goal_node_);

    solution_ = solution =
        std::make_shared<Path>(start_tree_->getConnectionToNode(goal_node_), metrics_, checker_, logger_);
    solution_->setTree(start_tree_);
  }

  if (better_solution_found && start_tree_->getRoot() != initial_start_node)
  {
    // Rewire the tree root (set start_node)
    initial_start_node->disconnect();

    NodePtr root = start_tree_->getRoot();  // tmp_start_node

    std::vector<NodePtr> root_children;

    ConnectionPtr conn_root_child_on_path = start_tree_->getConnectionToNode(goal_node_).front();
    NodePtr root_child_on_path = conn_root_child_on_path->getChild();
    double cost_first_conn_on_path = conn_root_child_on_path->getCost();

    ConnectionPtr conn2child;
    for (unsigned int i = 0; i < root->getChildConnectionsSize(); i++)
    {
      conn2child = root->childConnection(i);
      assert(conn2child->getParent() == start_tree_->getRoot());

      if (conn2child->getChild() != root_child_on_path)
        root_children.push_back(conn2child->getChild());
    }

    start_tree_->changeRoot(goal_node_);  // change root before removing the old root

    ConnectionPtr conn2node_on_path = std::make_shared<Connection>(root_child_on_path, initial_start_node, logger_);
    conn2node_on_path->setCost(cost_first_conn_on_path);
    conn2node_on_path->add();

    ConnectionPtr parent_conn;
    for (unsigned int i = 0; i < root_children.size(); i++)
    {
      ConnectionPtr conn = std::make_shared<Connection>(initial_start_node, root_children.at(i),
                                                        logger_);  // the parent of start_node is the node on the path,
                                                                   // the others are children

      parent_conn = root_children.at(i)->getParentConnections().front();
      conn->setCost(parent_conn->getCost());
      conn->add();
    }

    start_tree_->removeNode(root);
    start_tree_->addNode(initial_start_node);
    start_tree_->changeRoot(initial_start_node);

    solution_ = solution =
        std::make_shared<Path>(start_tree_->getConnectionToNode(goal_node_), metrics_, checker_, logger_);
    solution_->setTree(start_tree_);
  }

  return solved_;
}

bool AnytimeRRT::improve(NodePtr& start_node, PathPtr& solution, const unsigned int& max_iter, const double& max_time)
{
  double cost2beat = (1.0 - cost_impr_) * path_cost_;
  return improve(start_node, solution, cost2beat, max_iter, max_time);
}

bool AnytimeRRT::improve(NodePtr& start_node, PathPtr& solution, const double& cost2beat, const unsigned int& max_iter,
                         const double& max_time)
{
  NodePtr tmp_goal_node = std::make_shared<Node>(goal_node_->getConfiguration(), logger_);
  return improve(start_node, tmp_goal_node, solution, cost2beat, max_iter, max_time);
}

bool AnytimeRRT::improve(NodePtr& start_node, NodePtr& goal_node, PathPtr& solution, const unsigned int& max_iter,
                         const double& max_time)
{
  double cost2beat = (1.0 - cost_impr_) * path_cost_;
  CNR_TRACE(logger_, "cost2beat: " << cost2beat);
  return improve(start_node, goal_node, solution, cost2beat, max_iter, max_time);
}

bool AnytimeRRT::improve(NodePtr& start_node, NodePtr& goal_node, PathPtr& solution, const double& cost2beat,
                         const unsigned int& max_iter, const double& max_time)
{
  auto tic = graph_time::now();

  if (max_time <= 0.0)
    return false;

  double utopia = metrics_->utopia(start_node->getConfiguration(),
                                   goal_node->getConfiguration());  // start and goal may be different from
                                                                    // the previous ones
  can_improve_ = true;

  if (cost_ <= utopia_tolerance_ * utopia)  // also if start and/or goal are changed, the
                                            // old path is better to follow
  {
    CNR_TRACE(logger_, "Utopia reached! Utopia: " << utopia_tolerance_ * utopia << " path cost: " << path_cost_);
    can_improve_ = false;
    return false;
  }

  if (cost2beat <= utopia)
  {
    CNR_TRACE(logger_, "The cost to beat is less than utopia, impossible to reach! Utopia: "
                           << utopia << " cost to beat: " << cost2beat);
    return false;
  }

  new_tree_ = std::make_shared<Tree>(start_node, max_distance_, checker_, metrics_, logger_, use_kdtree_);

  tmp_goal_node_ = goal_node;
  cost2beat_ = cost2beat;

  bias_ = bias_ - delta_;
  if (bias_ < 0.1)
    bias_ = 0.1;

  improve_sampler_ = std::make_shared<InformedSampler>(start_node->getConfiguration(), goal_node->getConfiguration(),
                                                       sampler_->getLB(), sampler_->getUB(), logger_);
  improve_sampler_->setCost(cost2beat);  //(1-cost_impr_)*path_cost_

  for (unsigned int iter = 0; iter < max_iter; iter++)
  {
    if (AnytimeRRT::improveUpdate(solution))
    {
      CNR_TRACE(logger_, "Improved path cost: " << path_cost_);

      assert(solution->getTree() == solution_->getTree());
      assert(start_tree_ == new_tree_);

      return true;
    }

    if (toSeconds(graph_time::now(), tic) >= 0.98 * max_time)
      break;
  }

  return false;
}

bool AnytimeRRT::config(const std::string& param_ns)
{
  setParameters();
  return RRT::config(param_ns);
}

void AnytimeRRT::resetProblem()
{
  RRT::resetProblem();
  new_tree_.reset();
}

bool AnytimeRRT::improveUpdate(PathPtr& solution)
{
  CNR_TRACE(logger_, "AnytimeRRT::improveUpdate");

  if (not can_improve_)
  {
    CNR_TRACE(logger_, "already found the best solution");
    solution = solution_;
    return true;
  }

  if (improve_sampler_->collapse())
    return false;

  return AnytimeRRT::improveUpdate(improve_sampler_->sample(), solution);
}

bool AnytimeRRT::improveUpdate(const Eigen::VectorXd& point, PathPtr& solution)
{
  CNR_TRACE(logger_, "AnytimeRRT::improveUpdate");

  if (not can_improve_)
  {
    CNR_TRACE(logger_, "already found the best solution");
    solution = solution_;
    return true;
  }

  NodePtr new_node;
  if (new_tree_->informedExtend(point, new_node, tmp_goal_node_->getConfiguration(), cost2beat_, bias_))
  {
    if ((new_node->getConfiguration() - tmp_goal_node_->getConfiguration()).norm() <= max_distance_)
    {
      std::vector<ConnectionPtr> conn2node = new_tree_->getConnectionToNode(new_node);
      double cost_node2goal = metrics_->cost(new_node, tmp_goal_node_);
      double new_solution_cost = cost_node2goal;

      for (const ConnectionPtr& conn : conn2node)
        new_solution_cost += conn->getCost();

      CNR_TRACE(logger_, "A new solution with cost " << new_solution_cost << " was found");

      if (new_solution_cost < solution_->cost())
      {
        CNR_TRACE(logger_, "Its cost is less than that of current solution ");

        if (checker_->checkConnection(new_node->getConfiguration(), tmp_goal_node_->getConfiguration()))
        {
          CNR_TRACE(logger_, "Connection to goal collision free");

          assert(tmp_goal_node_->getParentConnectionsSize() == 0);

          //          start_tree_->removeNode(goal_node_);
          goal_node_ = tmp_goal_node_;
          goal_cost_ = goal_cost_fcn_->cost(goal_node_);

          ConnectionPtr conn_node2goal = std::make_shared<Connection>(new_node, goal_node_, logger_);
          conn_node2goal->setCost(cost_node2goal);
          conn_node2goal->add();

          new_tree_->addNode(goal_node_);
          start_tree_ = new_tree_;

          solution = std::make_shared<Path>(start_tree_->getConnectionToNode(goal_node_), metrics_, checker_, logger_);
          solution->setTree(start_tree_);
          solution_ = solution;

          best_utopia_ = goal_cost_ + metrics_->utopia(start_tree_->getRoot(), goal_node_);
          ;

          path_cost_ = solution_->cost();
          cost_ = path_cost_ + goal_cost_;

          CNR_TRACE(logger_, "Path cost %f, utopia %f", path_cost_, best_utopia_);

          improve_sampler_->setCost(std::min(path_cost_, cost2beat_));

          return true;
        }
        else
          CNR_TRACE(logger_, "Connection to goal not collision free, discarding the solution");
      }
    }
  }
  return false;
}

bool AnytimeRRT::update(const NodePtr& n, PathPtr& solution)
{
  CNR_ERROR(logger_, "Update to node not yet available");
  return false;
}

}  // end namespace core
}  // end namespace graph
