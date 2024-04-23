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

namespace graph
{
namespace core
{

bool RRTStar::addStartTree(const TreePtr &start_tree, const double &max_time)
{
  assert(start_tree);
  start_tree_ = start_tree;
  return setProblem(max_time);
}

bool RRTStar::config(const std::string &param_ns)
{
  RRT::config(param_ns);

  solved_ = false;
  get_param(logger_,param_ns_,"rewire_radius",r_rewire_,2.0*max_distance_);
  return true;
}

bool RRTStar::importFromSolver(const RRTStarPtr& solver)
{
  CNR_DEBUG(logger_,"Import from RRTStar solver");

  if(this == solver.get())// Avoid self-assignment
    return true;

  if(RRT::importFromSolver(std::static_pointer_cast<RRT>(solver)))
  {
    r_rewire_ = solver->r_rewire_;
    return true;
  }
  else
  {
    CNR_ERROR(logger_,"Import from solver failed");
    return false;
  }
}

bool RRTStar::importFromSolver(const TreeSolverPtr& solver)
{
  if(std::dynamic_pointer_cast<RRTStar>(solver) != nullptr)
  {
    return RRTStar::importFromSolver(std::static_pointer_cast<RRTStar>(solver));
  }
  else
  {
    return TreeSolver::importFromSolver(solver);
  }
}

void RRTStar::updateRewireRadius()
{
  //TO DO: update rewire radius as stated by RRT* paper
  //r_rewire_ =
  return;
}

bool RRTStar::update(PathPtr& solution)
{
  CNR_DEBUG(logger_,"RRT*::update");

  return update(sampler_->sample(), solution);
}

bool RRTStar::update(const Eigen::VectorXd& configuration, PathPtr& solution)
{
  CNR_DEBUG(logger_,"RRT*::update");

  if (!problem_set_)
  {
    CNR_DEBUG(logger_,"RRT* -> not init");

    return false;
  }
  if (cost_ <= utopia_tolerance_ * best_utopia_)
  {
    CNR_DEBUG(logger_,"RRT*:: Solution already optimal");

    solution=solution_;
    completed_=true;
    return true;
  }

  updateRewireRadius();

  if(not solved_)
  {
    CNR_DEBUG(logger_,"RRT* -> solving");

    NodePtr new_node;
    if(start_tree_->rewire(configuration,r_rewire_,new_node))
    {
      if((new_node->getConfiguration() - goal_node_->getConfiguration()).norm() < max_distance_)
      {
        if(checker_->checkConnection(new_node->getConfiguration(), goal_node_->getConfiguration()))
        {
          ConnectionPtr conn = std::make_shared<Connection>(new_node, goal_node_, logger_);
          conn->setCost(metrics_->cost(new_node, goal_node_));
          conn->add();

          solution_ = std::make_shared<Path>(start_tree_->getConnectionToNode(goal_node_), metrics_, checker_, logger_);
          solution_->setTree(start_tree_);
          solution = solution_;

          start_tree_->addNode(goal_node_);

          path_cost_ = solution_->cost();
          cost_=path_cost_+goal_cost_;
          sampler_->setCost(path_cost_);

          solved_ = true;

          return true;
        }
      }
    }
    return false;
  }
  else
  {
    CNR_DEBUG(logger_,"RRT* -> improving");

    bool improved = start_tree_->rewire(configuration, r_rewire_);
    if(improved)
    {
      if (start_tree_->costToNode(goal_node_) >= (solution_->cost() - 1e-8))
        return false;

      solution_ = std::make_shared<Path>(start_tree_->getConnectionToNode(goal_node_), metrics_, checker_, logger_);
      solution_->setTree(start_tree_);

      path_cost_ = solution_->cost();
      cost_ = path_cost_+goal_cost_;
      sampler_->setCost(path_cost_);
    }
    solution = solution_;
    return improved;
  }
}


bool RRTStar::update(const NodePtr& n, PathPtr& solution)
{
  CNR_DEBUG(logger_,"RRT*::update");

  if (!problem_set_)
  {
    CNR_DEBUG(logger_,"RRT* -> not init");

    return false;
  }
  if (cost_ <= utopia_tolerance_ * best_utopia_)
  {
    CNR_DEBUG(logger_,"RRT*:: Solution already optimal");

    solution=solution_;
    completed_=true;
    return true;
  }

  r_rewire_ = computeRewireRadius();

  if(not solved_)
  {
    CNR_DEBUG(logger_,"RRT* -> solving");
    NodePtr new_node;
    if(start_tree_->rewireToNode(n,r_rewire_,new_node))
    {
      if((new_node->getConfiguration() - goal_node_->getConfiguration()).norm() < max_distance_)
      {
        if(checker_->checkConnection(new_node->getConfiguration(), goal_node_->getConfiguration()))
        {
          ConnectionPtr conn = std::make_shared<Connection>(new_node, goal_node_, logger_);
          conn->setCost(metrics_->cost(new_node, goal_node_));
          conn->add();

          solution_ = std::make_shared<Path>(start_tree_->getConnectionToNode(goal_node_), metrics_, checker_, logger_);
          solution_->setTree(start_tree_);
          solution = solution_;

          start_tree_->addNode(goal_node_);

          path_cost_ = solution_->cost();
          cost_=path_cost_+goal_cost_;
          sampler_->setCost(path_cost_);

          solved_ = true;

          return true;
        }
      }
    }
    return false;
  }
  else
  {
    CNR_DEBUG(logger_,"RRT* -> improving");

    //double r_rewire = std::min(start_tree_->getMaximumDistance(), r_rewire_factor_ * sampler_->getSpecificVolume() * std::pow(std::log(start_tree_->getNumberOfNodes())/start_tree_->getNumberOfNodes(),1./dof_));
    bool improved = start_tree_->rewireToNode(n, r_rewire_);

    if (improved)
    {
      if (start_tree_->costToNode(goal_node_) >= (solution_->cost() - 1e-8))
        return false;

      solution_ = std::make_shared<Path>(start_tree_->getConnectionToNode(goal_node_), metrics_, checker_, logger_);
      solution_->setTree(start_tree_);

      path_cost_ = solution_->cost();
      cost_ = path_cost_+goal_cost_;
      sampler_->setCost(path_cost_);
    }
    solution = solution_;
    return improved;
  }
}

bool RRTStar::solve(PathPtr &solution, const unsigned int& max_iter, const double &max_time)
{
  if(not initialized_)
    return false;

  std::chrono::time_point<std::chrono::system_clock> tic = std::chrono::system_clock::now();
  bool solved = false;
  unsigned int n_iter = 0;
  for (unsigned int iter = 0; iter < max_iter; iter++)
  {
    n_iter++;
    if(update(solution))
    {
      CNR_DEBUG(logger_,"Improved or solved in %u iterations", n_iter);
      solved_ = true;
      solved = true;

      n_iter = 0;

      if(completed_)
        break;
    }
    if(std::chrono::duration<double>(std::chrono::system_clock::now()-tic).count()>=0.98*max_time)
      break;
  }

  CNR_DEBUG(logger_,"Solved: %d. Completed: %d. Cost: %f. Utopia: %f", solved_,completed_,cost_,best_utopia_*utopia_tolerance_);

  return solved;
}

} //end namespace core
} // end namespace graph
