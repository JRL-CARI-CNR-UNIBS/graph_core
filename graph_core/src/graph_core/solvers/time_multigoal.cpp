/*
Copyright (c) 2020, JRL-CARI CNR-STIIMA/UNIBS
Manuel Beschi manuel.beschi@unibs.it
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


#include <graph_core/solvers/time_multigoal.h>


namespace pathplan
{

bool TimeMultigoalSolver::addStart(const NodePtr& start_node, const double &max_time)
{
  if (!configured_)
  {
    ROS_ERROR("Solver is not configured.");
    return false;
  }
  solved_ = false;
  start_tree_ = std::make_shared<Tree>(start_node, Forward, max_distance_, checker_, metrics_);
  setProblem(max_time);
  ROS_DEBUG("Add start goal");
  return true;
}

bool TimeMultigoalSolver::addGoal(const NodePtr& goal_node, const double &max_time)
{
  if (!start_tree_)
  {
    ROS_ERROR("You have to specify first the start goal");
    return false;
  }

  ROS_DEBUG("compute goal cost");
  double goal_cost=goal_cost_fcn_->cost(goal_node);
  ROS_DEBUG("check utopia");
  double utopia=goal_cost+metrics_->utopia(goal_node,start_tree_->getRoot());
  if ((utopia)>cost_)
  {
    ROS_DEBUG("the utopia cost of this goal is already worst than the actual solution, skip it");
    return false;
  }
  if (!checker_->check(goal_node->getConfiguration()))
  {
    ROS_DEBUG("goal collides, skip it");
    return false;
  }

  if (utopia<best_utopia_)
    best_utopia_=utopia;

  PathPtr solution;
  double path_cost=std::numeric_limits<double>::infinity();
  double cost=std::numeric_limits<double>::infinity();
  TreePtr goal_tree;
  GoalStatus status;
  NodePtr new_node;
  ROS_DEBUG("check direct connection");

  unsigned int index=goal_nodes_.size();
  if (start_tree_->connectToNode(goal_node, new_node,max_time))
  {

    ROS_DEBUG("there is direct connection");
    solution = std::make_shared<Path>(start_tree_->getConnectionToNode(goal_node), metrics_, checker_);
    solution->setTree(start_tree_);

    path_cost = solution->cost();
    cost=path_cost+goal_cost;

    solved_ = true;

    if (cost<=(utopia+1e-8))
    {
      ROS_DEBUG("Goal %u reaches its utopia.", index);
      status=GoalStatus::done;
    }
    else
    {
      ROS_DEBUG("Goal %u has a solution with cost %f",index,cost);
      status=GoalStatus::refine;
    }

    PATH_COMMENT_STREAM("A direct solution is found\n" << *solution);
  }
  else
  {
    path_cost=cost = std::numeric_limits<double>::infinity();
    goal_tree = std::make_shared<Tree>(goal_node, Backward, max_distance_, checker_, metrics_);
    status=GoalStatus::search;
  }

  TimeBasedInformedSamplerPtr time_sampler = std::make_shared<TimeBasedInformedSampler>(start_tree_->getRoot()->getConfiguration(),
                                                                           goal_node->getConfiguration(),
                                                                           time_based_sampler_->getLB(),
                                                                           time_based_sampler_->getUB(),
                                                                           time_based_sampler_->getMaxSpeed(),
                                                                           path_cost_+goal_cost_-goal_cost);

  goal_nodes_.push_back(goal_node);
  goal_trees_.push_back(goal_tree);
  path_costs_.push_back(path_cost);
  goal_costs_.push_back(goal_cost);
  costs_.push_back(cost);
  utopias_.push_back(utopia);
  solutions_.push_back(solution);
  time_samplers_.push_back(time_sampler);
  status_.push_back(status);

  if (isBestSolution(goal_nodes_.size()-1))
    ROS_DEBUG_STREAM("Goal "<<goal_node->getConfiguration().transpose() << " is the best one with a cost = "  << cost);

  init_=true;


  return true;
}

bool TimeMultigoalSolver::isBestSolution(const int &index)
{
  assert(status_.at(index)!=GoalStatus::discard);

  if (costs_.at(index)>=(cost_-1e-8))
    return false;
  path_cost_=path_costs_.at(index);
  goal_cost_=goal_costs_.at(index);
  cost_=costs_.at(index);
  best_goal_index=index;
  solution_=solutions_.at(index);

  for (unsigned int igoal=0;igoal<goal_nodes_.size();igoal++)
  {

    if (status_.at(igoal)==GoalStatus::discard)
      continue;
    if (utopias_.at(igoal)>cost_)
    {
      status_.at(igoal)=GoalStatus::discard;
      ROS_DEBUG("Goal %u is discarded. utopia=%f, best cost=%f",igoal,utopias_.at(igoal),cost_);
      cleanTree();
      continue;
    }
    if (status_.at(igoal)==GoalStatus::done)
      continue;
    ROS_DEBUG("Best solution costs. Total cost = %f, Path cost= %f. Goal cost  =%f",cost_,path_cost_,goal_cost_);
    ROS_DEBUG("Goal %u. Total cost = %f, Path cost= %f. Goal cost  =%f",igoal,costs_.at(igoal), path_costs_.at(igoal),goal_costs_.at(igoal));
    ROS_DEBUG("Goal %u. sampler cost = %f",igoal,path_cost_+goal_cost_-goal_costs_.at(igoal));

    time_samplers_.at(igoal)->setCost(path_cost_+goal_cost_-goal_costs_.at(igoal));
  }

  if ((cost_<0.9999*cost_at_last_clean) || start_tree_->needCleaning())
  {
    cost_at_last_clean=cost_;
    cleanTree();
  }

  return true;
}

void TimeMultigoalSolver::resetProblem()
{
  goal_nodes_.clear();
  goal_trees_.clear();
  path_costs_.clear();
  utopias_.clear();
  solutions_.clear();
  time_samplers_.clear();
  status_.clear();
  start_tree_.reset();
  solved_=false;
}

bool TimeMultigoalSolver::setProblem(const double &max_time)   //CHIEDI A MNAUEL->devi metterlo perche in TreeSolver hai dovuto metterlo, ma qua non serve
{
  if (!start_tree_)
    return false;
  if (goal_nodes_.size()==0)
    return false;

  return true;
}

bool TimeMultigoalSolver::config(const ros::NodeHandle& nh)
{
  max_distance_ = 1;
  if (!nh.getParam("max_distance",max_distance_))
  {
    ROS_WARN("%s/max_distance is not set. using 1.0",nh.getNamespace().c_str());
    max_distance_=1.0;
  }
  if (!nh.getParam("extend",extend_))
  {
    ROS_WARN("%s/extend is not set. using false (connect algorithm)",nh.getNamespace().c_str());
    max_distance_=1.0;
  }
  if (!nh.getParam("rewire_radius",r_rewire_))
  {
    ROS_WARN("%s/rewire_radius is not set. using 1.0",nh.getNamespace().c_str());
    r_rewire_=1.0;
  }


  configured_=true;
  return true;
}


bool TimeMultigoalSolver::update(PathPtr& solution)
{
  if (!init_)
  {
    ROS_WARN("TimeMultigoalSolver is not initialized");
    return false;
  }
  if (cost_ <= utopia_tolerance * best_utopia_)
  {
    ROS_DEBUG("Find the final solution");
    solution=solution_;
    completed_=true;
    return false;
  }


  bool global_improvement=false;
  double r_rewire = std::max(start_tree_->getMaximumDistance(),r_rewire_);
  double old_cost=cost_;


  for (unsigned int igoal=0;igoal<goal_nodes_.size();igoal++)
  {
    NodePtr new_start_node, new_goal_node;
    bool add_to_start, add_to_goal;
    Eigen::VectorXd configuration = time_samplers_.at(igoal)->sample();

    double prob=1.0;
    double min_prob=0.2;
    if ((costs_.at(igoal)-cost_)>2.0*cost_)
    {
      prob=min_prob;
    }
    else
    {
      prob=1.0-(1.0-min_prob)*((costs_.at(igoal)-cost_))/(2.0*cost_);
    }
    if (ud_(gen_)>prob)
      continue;
    switch (status_.at(igoal))
    {

    case GoalStatus::search:


      if (extend_)
        add_to_start = start_tree_->extend(configuration, new_start_node);
      else
        add_to_start = start_tree_->connect(configuration, new_start_node);


      if (add_to_start)
      {
        if (extend_)
          add_to_goal = goal_trees_.at(igoal)->extendToNode(new_start_node, new_goal_node);
        else
          add_to_goal = goal_trees_.at(igoal)->connectToNode(new_start_node, new_goal_node);
      }
      else
      {
        if (extend_)
          add_to_goal = goal_trees_.at(igoal)->extend(configuration, new_goal_node);
        else
          add_to_goal = goal_trees_.at(igoal)->connect(configuration, new_goal_node);
      }

      if (add_to_start && add_to_goal && new_goal_node == new_start_node)
      {
        goal_trees_.at(igoal)->keepOnlyThisBranch(goal_trees_.at(igoal)->getConnectionToNode(new_goal_node));
        start_tree_->addBranch(goal_trees_.at(igoal)->getConnectionToNode(new_goal_node));

        solutions_.at(igoal) = std::make_shared<Path>(start_tree_->getConnectionToNode(goal_nodes_.at(igoal)), metrics_, checker_);
        solutions_.at(igoal)->setTree(start_tree_);

        double cost_0=solutions_.at(igoal)->cost();
        size_t waypoints0=solutions_.at(igoal)->getWaypoints().size();
        ros::WallTime tsimpl=ros::WallTime::now();
        solutions_.at(igoal)->simplify(solutions_.at(igoal)->cost());
        size_t waypoints1=solutions_.at(igoal)->getWaypoints().size();
        ROS_DEBUG("simplify: cost from %f to %f (waypoints %zu->%zu) in %f second",cost_0,solutions_.at(igoal)->cost(),waypoints0,waypoints1,(ros::WallTime::now()-tsimpl).toSec());

        double cost_1=solutions_.at(igoal)->cost();
        ros::WallTime twarp=ros::WallTime::now();
        for (int iwarp=0;iwarp<10;iwarp++)
        {
          solutions_.at(igoal)->warp(0.1*solutions_.at(igoal)->cost(),0.01);
        }
        ROS_DEBUG("warp: cost from %f to %f in %f second",cost_1,solutions_.at(igoal)->cost(),(ros::WallTime::now()-twarp).toSec());



        path_costs_.at(igoal) = solutions_.at(igoal)->cost();
        costs_.at(igoal) = path_costs_.at(igoal)+goal_costs_.at(igoal);

        solved_ = true;
        if (path_costs_.at(igoal)<=(utopias_.at(igoal)*utopia_tolerance))
        {
          ROS_DEBUG("Goal %u reaches its utopia. cost = %f, utopia =%f",igoal,path_costs_.at(igoal),utopias_.at(igoal));
          status_.at(igoal)=GoalStatus::done;
        }
        else
        {
          ROS_DEBUG("Goal %u has a solution with cost %f",igoal,path_costs_.at(igoal));
          status_.at(igoal)=GoalStatus::refine;
        }
        global_improvement=isBestSolution(igoal) || global_improvement;
      }

      break;
    case GoalStatus::refine:
      if (start_tree_->rewire(time_samplers_.at(igoal)->sample(), r_rewire)) //TODO add weight goal
      {
        if (start_tree_->costToNode(goal_nodes_.at(igoal)) >= (path_costs_.at(igoal) - 1e-8))
          continue;

        solutions_.at(igoal) = std::make_shared<Path>(start_tree_->getConnectionToNode(goal_nodes_.at(igoal)), metrics_, checker_);
        solutions_.at(igoal)->setTree(start_tree_);

        double cost_0=solutions_.at(igoal)->cost();
        size_t waypoints0=solutions_.at(igoal)->getWaypoints().size();
        ros::WallTime tsimpl=ros::WallTime::now();
        solutions_.at(igoal)->simplify(solutions_.at(igoal)->cost());
        size_t waypoints1=solutions_.at(igoal)->getWaypoints().size();
        ROS_DEBUG("simplify: cost from %f to %f (waypoints %zu->%zu) in %f second",cost_0,solutions_.at(igoal)->cost(),waypoints0,waypoints1,(ros::WallTime::now()-tsimpl).toSec());

        double cost_1=solutions_.at(igoal)->cost();
        ros::WallTime twarp=ros::WallTime::now();
        for (int iwarp=0;iwarp<10;iwarp++)
        {
          solutions_.at(igoal)->warp(0.1*solutions_.at(igoal)->cost(),0.01);
        }
        ROS_DEBUG("warp: cost from %f to %f in %f second",cost_1,solutions_.at(igoal)->cost(),(ros::WallTime::now()-twarp).toSec());


        path_costs_.at(igoal) = solutions_.at(igoal)->cost();
        costs_.at(igoal) = path_costs_.at(igoal)+goal_costs_.at(igoal);
        if (costs_.at(igoal)<=(utopias_.at(igoal)+1e-8))
        {
          ROS_DEBUG("Goal %u reaches its utopia. cost = %f, utopia =%f",igoal,costs_.at(igoal),utopias_.at(igoal));
          cleanTree();
          status_.at(igoal)=GoalStatus::done;
        }
        else
          ROS_DEBUG("Goal %u refines solution with cost %f",igoal,costs_.at(igoal));
        global_improvement=isBestSolution(igoal) || global_improvement;
      }
      else
      {
        double cost_1=solutions_.at(igoal)->cost();
        ros::WallTime twarp=ros::WallTime::now();
        solutions_.at(igoal)->warp(0.1*solutions_.at(igoal)->cost(),0.01);
        ROS_DEBUG_COND(solutions_.at( igoal)->cost()<cost_1,"warp: cost from %f to %f in %f second",cost_1,solutions_.at(igoal)->cost(),(ros::WallTime::now()-twarp).toSec());

        if (solutions_.at(igoal)->cost() >= (path_costs_.at(igoal) - 1e-8))
          continue;
        global_improvement=isBestSolution(igoal) || global_improvement;

      }
      break;
    case GoalStatus::discard:
    case GoalStatus::done:
      break;
    }
  }

  solution = solution_;
  return global_improvement;
}


void TimeMultigoalSolver::printMyself(std::ostream &os) const
{
  os << "Best cost: " << cost_;
  os << " path cost: " << path_cost_;
  os << " goal cost: " << goal_cost_;
  os << ". Nodes of start tree: " << start_tree_->getNumberOfNodes() << "\nGoals:\n";

  for (unsigned int igoal=0;igoal<goal_nodes_.size();igoal++)
  {
    os << igoal <<". Status: ";
    switch (status_.at(igoal))
    {
    case GoalStatus::done:
      os << "done";
      break;
    case GoalStatus::refine:
      os << "refine";
      break;
    case GoalStatus::search:
      os << "search";
      break;
    case GoalStatus::discard:
      os << "discard";
      break;
    }
    os << ". cost = " << costs_.at(igoal);
    os << ". path cost = " << path_costs_.at(igoal);
    os << ". goal cost = " << goal_costs_.at(igoal);
    os << ". utopia = " << utopias_.at(igoal);
    os << ". ellisse = " << path_cost_+goal_cost_-goal_costs_.at(igoal);
    os << ". volume = " << std::scientific <<time_samplers_.at(igoal)->getSpecificVolume()<< std::defaultfloat;

    os<<std::endl;
  }
}

void TimeMultigoalSolver::cleanTree()
{
  std::vector<NodePtr> white_list=goal_nodes_;
  white_list.push_back(start_tree_->getRoot());
  std::vector<SamplerPtr> samplers;
  for (unsigned int igoal=0;igoal<goal_nodes_.size();igoal++)
  {
    if ((status_.at(igoal)==GoalStatus::refine)||(status_.at(igoal)==GoalStatus::search))
      samplers.push_back(time_samplers_.at(igoal));
  }


  for (const PathPtr& sol: solutions_)
  {
    if (sol)
      for (const ConnectionPtr& conn: sol->getConnections())
        white_list.push_back(conn->getChild());
  }
  ROS_DEBUG_STREAM("Removed nodes: " << start_tree_->purgeNodesOutsideEllipsoids(samplers,white_list));
}

TreeSolverPtr TimeMultigoalSolver::clone(const MetricsPtr& metrics, const CollisionCheckerPtr& checker, const SamplerPtr& sampler)
{
  return std::make_shared<TimeMultigoalSolver>(metrics,checker,sampler,time_based_sampler_->getMaxSpeed());
}

}  // namespace pathplan
