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

bool TreeSolver::config(const YAML::Node &config)
{
<<<<<<< HEAD
  nh_ = nh;
=======
  throw std::invalid_argument("not implemented yet");
  max_distance_ = 1;

  if (!config["max_distance"])
  {
    CNR_WARN(logger_,"max_distance is not set. using 1.0");
    max_distance_=1.0;
  }
  max_distance_=config["max_distance"].as<double>();
>>>>>>> 1dc510815a81597abeb77c2de689d07284069805
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
    ROS_DEBUG("%s/informed is not set. using true (informed set enable)",nh.getNamespace().c_str());
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
<<<<<<< HEAD
  ros::WallTime tic = ros::WallTime::now();
  if(max_time <=0.0)
  {
    return false;
  }
=======
  std::chrono::time_point<std::chrono::system_clock> tic = std::chrono::system_clock::now();


  if(max_time <=0.0) return false;
>>>>>>> 1dc510815a81597abeb77c2de689d07284069805

  for (unsigned int iter = 0; iter < max_iter; iter++)
  {
    if (update(solution))
    {
      solved_ = true;
      return true;
    }

    std::chrono::time_point<std::chrono::system_clock> now = std::chrono::system_clock::now();
    std::chrono::duration<double> difference = now - tic;
    if(difference.count() >= 0.98*max_time) break;
  }

  return false;
}
bool TreeSolver::computePath(const Eigen::VectorXd& start_conf, const Eigen::VectorXd& goal_conf, const ros::NodeHandle& nh, PathPtr &solution, const double &max_time, const unsigned int &max_iter)
{
  NodePtr start_node = std::make_shared<Node>(start_conf);
  NodePtr goal_node  = std::make_shared<Node>(goal_conf);

<<<<<<< HEAD
  return computePath(start_node,goal_node,nh,solution,max_time,max_iter);
}

bool TreeSolver::computePath(const NodePtr &start_node, const NodePtr &goal_node, const ros::NodeHandle& nh, PathPtr &solution, const double &max_time, const unsigned int &max_iter)
=======
#pragma message(Reminder "IS IT NEEDED?")
bool TreeSolver::computePath(const NodePtr &start_node, const NodePtr &goal_node, const YAML::Node& nh, PathPtr &solution, const double &max_time, const unsigned int max_iter)
>>>>>>> 1dc510815a81597abeb77c2de689d07284069805
{
  resetProblem();
  config(nh);
  addStart(start_node);
  addGoal(goal_node);

<<<<<<< HEAD
  ros::WallTime tic = ros::WallTime::now();
  if(!solve(solution, max_iter, max_time))
  {
    ROS_INFO_STREAM("No solutions found. Time: "<<(ros::WallTime::now()-tic).toSec()<<", max time: "<<max_time);
=======

  if (!solve(solution, max_iter, max_time))
  {
    CNR_WARN(logger_,"No solutions found");
>>>>>>> 1dc510815a81597abeb77c2de689d07284069805
    return false;
  }
  return true;
}

bool TreeSolver::setSolution(const PathPtr &solution, const bool& solved)
{
//  solution_->setTree(start_tree_);   //SE SOLUTION HA IL SUO TREE ED E' DIVERSO?
  if(not solution ->getTree())
    return false;

  resetProblem();

  solution_ = solution;
  start_tree_ = solution_->getTree();

  goal_node_ = solution_->getGoalNode();
  goal_cost_ = goal_cost_fcn_->cost(goal_node_);

  best_utopia_ = goal_cost_+(goal_node_->getConfiguration() - start_tree_->getRoot()->getConfiguration()).norm();

  path_cost_ = solution->cost();
  cost_ = path_cost_+goal_cost_;

  sampler_->setCost(path_cost_);

  init_ = true;
  solved_=solved;

  if (solved_)
    clean();
  return true;
}

bool TreeSolver::importFromSolver(const TreeSolverPtr& solver)
{
<<<<<<< HEAD
  //ROS_INFO_STREAM("Import from Tree solver");
=======
  CNR_INFO(logger_,"Import from Tree solver");
>>>>>>> 1dc510815a81597abeb77c2de689d07284069805

  config();

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
