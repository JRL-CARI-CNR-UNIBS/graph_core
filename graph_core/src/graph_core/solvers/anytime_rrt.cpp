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

#include <graph_core/solvers/anytime_rrt.h>

namespace pathplan
{

bool AnytimeRRT::solveWithRRT(PathPtr& solution,
                              const unsigned int& max_iter,
                              const double &max_time)
{
  ros::WallTime tic = ros::WallTime::now();

  if(max_time <=0.0) return false;

  for (unsigned int iter = 0; iter < max_iter; iter++)
  {
    if (RRT::update(solution))
    {
      solved_ = true;
      return true;
    }

    if((ros::WallTime::now()-tic).toSec()>=0.98*max_time) break;
  }
  return false;
}

void AnytimeRRT::importFromSolver(const AnytimeRRTPtr& solver)
{
  ROS_INFO_STREAM("Import from AnytimeRRT solver");

  RRT::importFromSolver(std::static_pointer_cast<RRT>(solver));

  bias_            = solver->getBias();
  delta_           = solver->getDelta();
  new_tree_        = solver->getNewTree();
  cost_impr_       = solver->getCostImpr();
  new_tree_solved_ = solver->getNewTreeSolved();
}

void AnytimeRRT::importFromSolver(const TreeSolverPtr& solver)
{
  const std::type_info& rrt_type = typeid(RRT);
  const std::type_info& anytime_rrt_type = typeid(AnytimeRRT);
  const std::type_info& type = typeid(*solver);

  if(std::type_index(type) == std::type_index(anytime_rrt_type))  //Check before RRT
    AnytimeRRT::importFromSolver(std::static_pointer_cast<AnytimeRRT>(solver));

  else if(std::type_index(type) == std::type_index(rrt_type))
    RRT::importFromSolver(std::static_pointer_cast<RRT>(solver));

  else
    TreeSolver::importFromSolver(solver);
}

bool AnytimeRRT::solve(PathPtr &solution, const unsigned int& max_iter, const double& max_time)
{
  ros::WallTime tic = ros::WallTime::now();
  double time;

  if(solved_)
  {
    solution = solution_;

    if (path_cost_ <= 1.003 * utopia_)
    {
      ROS_INFO("Utopia reached!");
      completed_=true;
      return true;
    }
  }

  if(max_time <=0.0) return false;

  // RRT to find quickly a first sub-optimal solution
  unsigned int n_failed_iter = 0;
  time = (ros::WallTime::now()-tic).toSec();
  while(time<0.98*max_time && solved_ == false && n_failed_iter<FAILED_ITER)
  {
    bool success = AnytimeRRT::solveWithRRT(solution,max_iter,(max_time-time));
    if(!success)
      n_failed_iter += 1;

    if(success)
    {
      solved_= true;
      solution_ = solution;
    }

    time = (ros::WallTime::now()-tic).toSec();
  }

  if(solved_ == false)
    return false;

  ROS_INFO_STREAM("Path cost: "<<path_cost_);

  if (path_cost_ <= 1.003 * utopia_)
  {
    ROS_INFO("Utopia reached!");
    completed_=true;
    return true;
  }

  // Informed trees to find better solutions
  n_failed_iter = 0;
  time = (ros::WallTime::now()-tic).toSec();
  while(time<0.98*max_time && completed_ == false && n_failed_iter<FAILED_ITER)
  {
    NodePtr start_node = std::make_shared<Node>(start_tree_->getRoot()->getConfiguration());
    goal_node_ = std::make_shared<Node>(goal_node_->getConfiguration());

    bool success = AnytimeRRT::improve(start_node,solution,max_iter,(max_time-time));
    if(!success)
      n_failed_iter += 1;

    if(success != new_tree_solved_)
      assert(0);

    if(path_cost_ <= 1.003 * utopia_)
    {
      ROS_INFO("Utopia reached!");
      completed_=true;
      return true;
    }

    time = (ros::WallTime::now()-tic).toSec();
  }
  return solved_;
}
bool AnytimeRRT::improve(NodePtr& start_node, NodePtr& goal_node, PathPtr& solution, const unsigned int& max_iter, const double &max_time)
{
  goal_node_ = goal_node;
  return improve(start_node, solution, max_iter, max_time);
}

bool AnytimeRRT::improve(NodePtr& start_node, PathPtr& solution, const unsigned int& max_iter, const double &max_time)
{
  ros::WallTime tic = ros::WallTime::now();

  if(max_time <=0.0) return false;

  new_tree_ = std::make_shared<Tree>(start_node, Forward, max_distance_, checker_, metrics_);
  new_tree_solved_ = false;

  bias_ = bias_-delta_;
  if(bias_<0.1) bias_ = 0.1;

  sampler_->setCost(path_cost_); //(1-cost_impr_)*path_cost_

  for (unsigned int iter = 0; iter < max_iter; iter++)
  {
    if(AnytimeRRT::update(solution))
    {
      ROS_INFO_STREAM("Improved path cost: "<<path_cost_);

      new_tree_solved_ = true;
      start_tree_ = new_tree_;
      solution_->setTree(start_tree_); //do not delete

      return true;
    }

    if((ros::WallTime::now()-tic).toSec()>=0.98*max_time) break;
  }
  return false;
}

bool AnytimeRRT::config(const ros::NodeHandle& nh)
{
  bias_ = 0.9;
  setParameters();
  RRT::config(nh);
}

void AnytimeRRT::resetProblem()
{
  new_tree_ = NULL;
  completed_ = false;
  new_tree_solved_ = false;

  RRT::resetProblem();
}

bool AnytimeRRT::update(PathPtr &solution)
{
  PATH_COMMENT("AnytimeRRT::update");

  if (completed_ || new_tree_solved_)
  {
    PATH_COMMENT("already found the best solution");
    solution = solution_;
    return true;
  }

  if (sampler_->collapse())
    return false;

  return AnytimeRRT::update(sampler_->sample(), solution);
}

bool AnytimeRRT::update(const Eigen::VectorXd& point, PathPtr &solution)
{
  PATH_COMMENT("AnytimeRRT::update");

  if (completed_ || new_tree_solved_)
  {
    PATH_COMMENT("already found the best solution");
    solution = solution_;
    return true;
  }

  NodePtr new_node;
  Eigen::VectorXd goal_conf = goal_node_->getConfiguration();
  if (new_tree_->informedExtend(point,new_node, goal_conf, (1-cost_impr_)*path_cost_,bias_))
  {
    if ((new_node->getConfiguration() - goal_node_->getConfiguration()).norm() < max_distance_)
    {
      if(checker_->checkPath(new_node->getConfiguration(), goal_node_->getConfiguration()))
      {
        ConnectionPtr conn = std::make_shared<Connection>(new_node, goal_node_);
        conn->setCost(metrics_->cost(new_node, goal_node_));
        conn->add();
        solution_ = std::make_shared<Path>(new_tree_->getConnectionToNode(goal_node_), metrics_, checker_);
        solution_->setTree(new_tree_);
        new_tree_->addNode(goal_node_);
        path_cost_ = solution_->cost();
        cost_=path_cost_+goal_cost_;
        sampler_->setCost(path_cost_);
        solution = solution_;
        new_tree_solved_ = true;
        return true;
      }
    }
  }
  return false;
}

bool AnytimeRRT::update(const NodePtr& n, PathPtr &solution)
{
  ROS_ERROR("Update to node not yet available");
  return false;
}

TreeSolverPtr AnytimeRRT::clone(const MetricsPtr& metrics, const CollisionCheckerPtr& checker, const SamplerPtr& sampler)
{
  return std::make_shared<AnytimeRRT>(metrics,checker,sampler);
}

}


