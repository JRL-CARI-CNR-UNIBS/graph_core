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

#include <graph_core/solvers/tree_solver.h>

namespace pathplan
{

bool TreeSolver::config(const ros::NodeHandle& nh)
{
  nh_ = nh;
  max_distance_ = 1;
  if (!nh.getParam("max_distance",max_distance_))
  {
    ROS_WARN("%s/max_distance is not set. using 1.0",nh.getNamespace().c_str());
    max_distance_=1.0;
  }

  if (!nh.getParam("use_kdtree",use_kdtree_))
  {
    ROS_DEBUG("%s/use_kdtree is not set. set true",nh.getNamespace().c_str());
    use_kdtree_=true;
  }

  if (!nh.getParam("informed",informed_))
  {
    ROS_DEBUG("%s/informed is not set. using true (informed set enble)",nh.getNamespace().c_str());
    informed_=true;
  }

  if (!nh.getParam("extend",extend_))
  {
    ROS_WARN("%s/extend is not set. using false (connect algorithm)",nh.getNamespace().c_str());
    extend_=false;
  }

  if (!nh.getParam("warp",warp_))
  {
    ROS_DEBUG("%s/warp is not set. using false",nh.getNamespace().c_str());
    warp_=false;
  }

  if (not warp_)
  {
    if (!nh.getParam("warp_once",first_warp_))
    {
      ROS_DEBUG("%s/warp_once is not set. using false",nh.getNamespace().c_str());
      first_warp_=false;
    }
  }
  else
    first_warp_=true;

  if (!nh.getParam("utopia_tolerance",utopia_tolerance_))
  {
    ROS_WARN("%s/utopia_tolerance is not set. using 0.01",nh.getNamespace().c_str());
    utopia_tolerance_=0.01;
  }
  if (utopia_tolerance_<=0.0)
  {
    ROS_WARN("%s/utopia_tolerance cannot be negative, set equal to 0.0",nh.getNamespace().c_str());
    utopia_tolerance_=0.0;
  }
  utopia_tolerance_+=1.0;
  dof_=sampler_->getDimension();
  configured_=true;
  return true;
}


bool TreeSolver::solve(PathPtr &solution, const unsigned int& max_iter, const double& max_time)
{
  ros::WallTime tic = ros::WallTime::now();

  if(max_time <=0.0) return false;

  for (unsigned int iter = 0; iter < max_iter; iter++)
  {
    if (update(solution))
    {
      //ROS_INFO("Solved in %u iterations", iter);
      solved_ = true;
      return true;
    }

    if((ros::WallTime::now()-tic).toSec()>=0.98*max_time)
      break;
  }
  return false;
}

bool TreeSolver::computePath(const NodePtr &start_node, const NodePtr &goal_node, const ros::NodeHandle& nh, PathPtr &solution, const double &max_time, const unsigned int max_iter)
{
  config(nh);
  addStart(start_node);
  addGoal(goal_node);

  ros::WallTime tic = ros::WallTime::now();

  if (!solve(solution, max_iter, max_time))
  {
    ROS_INFO("No solutions found");

    ros::WallTime toc = ros::WallTime::now();
    ROS_INFO_STREAM("time: "<<(toc-tic).toSec()<<" max_t: "<<max_time);

    assert(0);

    return false;
  }

  ros::WallTime toc = ros::WallTime::now();
  ROS_INFO_STREAM("time: "<<(toc-tic).toSec()<<" max_t: "<<max_time);

  return true;
}

bool TreeSolver::setSolution(const PathPtr &solution, const bool& solved)
{
  solution_ = solution;
  solution_->setTree(start_tree_);   //SE SOLUTION HA IL SUO TREE ED E' DIVERSO?

  path_cost_ = solution->cost();
  cost_ = path_cost_+goal_cost_;

  solved_=solved;
  if (solved_)
    clean();
  return true;
}

bool TreeSolver::importFromSolver(const TreeSolverPtr& solver)
{
  ROS_INFO_STREAM("Import from Tree solver");

  config(getNodeHandle());

  solved_        = solver->solved();
  completed_     = solver->completed();
  init_          = solver->init();
  configured_    = solver->configured();
  dof_           = solver->dof();
  goal_cost_fcn_ = solver->getGoalCostFunction();
  path_cost_     = solver->getPathCost();
  goal_cost_     = solver->getGoalCost();
  cost_          = solver->getCost();
  start_tree_    = solver->getStartTree();
  solution_      = solver->getSolution();
  max_distance_  = solver->getMaxDistance();
  best_utopia_   = solver->getUtopia();
  setGoal(solver->getGoal());
  return true;
}

}
