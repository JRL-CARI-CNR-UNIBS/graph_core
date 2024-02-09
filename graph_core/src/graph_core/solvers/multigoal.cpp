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


#include <graph_core/solvers/multigoal.h>


namespace pathplan
{

bool MultigoalSolver::addStart(const NodePtr& start_node, const double &max_time)
{
  if (!configured_)
  {
    ROS_ERROR("Solver is not configured.");
    return false;
  }
  start_tree_ = std::make_shared<Tree>(start_node, max_distance_, checker_, metrics_,use_kdtree_);
  solved_ = false;

  setProblem(max_time);
  ROS_DEBUG("Add start goal");
  return true;
}

bool MultigoalSolver::addStartTree(const TreePtr &start_tree, const double &max_time)
{
  assert(start_tree);
  start_tree_ = start_tree;
  solved_ = false;
  setProblem(max_time);
  return true;
}

bool MultigoalSolver::addGoal(const NodePtr& goal_node, const double &max_time)
{
  if (!start_tree_)
  {
    ROS_ERROR("You have to specify first the start goal");
    return false;
  }

  double goal_cost=goal_cost_fcn_->cost(goal_node);

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

  unsigned int index=goal_nodes_.size();
  if (start_tree_->connectToNode(goal_node, new_node,max_time))
  {
    solution = std::make_shared<Path>(start_tree_->getConnectionToNode(goal_node), metrics_, checker_);
    solution->setTree(start_tree_);

    path_cost = solution->cost();
    cost=path_cost+goal_cost;

    solved_ = true;

    if (cost<=(utopia*utopia_tolerance_))
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
    goal_tree = std::make_shared<Tree>(goal_node, max_distance_, checker_, metrics_,use_kdtree_);
    status=GoalStatus::search;
  }



  InformedSamplerPtr sampler = std::make_shared<InformedSampler>(start_tree_->getRoot()->getConfiguration(),goal_node->getConfiguration(),sampler_->getLB(),sampler_->getUB());
  TubeInformedSamplerPtr tube_sampler = std::make_shared<TubeInformedSampler>(start_tree_->getRoot()->getConfiguration(),goal_node->getConfiguration(),sampler,metrics_);
  tube_sampler->setLocalBias(local_bias_);
  tube_sampler->setRadius(tube_radius_);
  if (solution)
    tube_sampler->setPath(solution);

  goal_nodes_.push_back(goal_node);
  goal_trees_.push_back(goal_tree);
  path_costs_.push_back(path_cost);
  goal_costs_.push_back(goal_cost);
  costs_.push_back(cost);
  utopias_.push_back(utopia);
  solutions_.push_back(solution);
  tube_samplers_.push_back(tube_sampler);
  samplers_.push_back(sampler);
  status_.push_back(status);
  were_goals_sampled_.push_back(false);
  goal_probabilities_.push_back(1.0);

  if (isBestSolution(goal_nodes_.size()-1))
    ROS_DEBUG_STREAM("Goal "<<goal_node->getConfiguration().transpose() << " is the best one with a cost = "  << cost);

  init_=true;
  return true;
}

bool MultigoalSolver::isBestSolution(const int &index)
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
    if (mixed_strategy_)
    {
      tube_samplers_.at(igoal)->setCost(path_cost_+goal_cost_-goal_costs_.at(igoal));
      samplers_.at(igoal)->setCost(path_cost_+goal_cost_-goal_costs_.at(igoal));
    }
    else if (informed_)
    {
      samplers_.at(igoal)->setCost(path_cost_+goal_cost_-goal_costs_.at(igoal));
    }
  }

  if ((cost_<0.9999*cost_at_last_clean) || start_tree_->needCleaning())
  {
    cost_at_last_clean=cost_;
    cleanTree();
  }

  return true;
}

void MultigoalSolver::resetProblem()
{
  goal_nodes_.clear();
  goal_trees_.clear();
  path_costs_.clear();
  utopias_.clear();
  were_goals_sampled_.clear();
  goal_probabilities_.clear();
  solutions_.clear();
  tube_samplers_.clear();
  status_.clear();
  start_tree_.reset();
  solved_=false;
}

bool MultigoalSolver::setProblem(const double &max_time)   //CHIEDI A MNAUEL->devi metterlo perche in TreeSolver hai dovuto metterlo, ma qua non serve
{
  if (!start_tree_)
    return false;
  if (goal_nodes_.size()==0)
    return false;

  return true;
}

bool MultigoalSolver::config(const ros::NodeHandle& nh)
{
  if (not TreeSolver::config(nh))
    return false;

  if (!nh.getParam("rewire_radius",r_rewire_))
  {
    ROS_DEBUG("%s/rewire_radius is not set. using 2.0*max_distance",nh.getNamespace().c_str());
    r_rewire_=2.0*max_distance_;
  }

  if (!nh.getParam("mixed_strategy",mixed_strategy_))
  {
    ROS_DEBUG("%s/mixed_strategy is not set. enable it if informed set is enable",nh.getNamespace().c_str());
    mixed_strategy_=informed_;
  }
  if (not informed_ && mixed_strategy_)
  {
    ROS_WARN("you cannot use mixed_strategy without informed set enable, turning it on");
    informed_=true;
  }
  if (!nh.getParam("bidirectional",bidirectional_))
  {
    ROS_DEBUG("%s/bidirectional is not set. using true",nh.getNamespace().c_str());
    bidirectional_=true;
  }

  if (!nh.getParam("k_nearest",knearest_))
  {
    ROS_DEBUG("%s/k_nearest is not set. using false (rewire using nodes in the radius)",nh.getNamespace().c_str());
    knearest_=false;
  }
  if (!nh.getParam("local_bias",local_bias_))
  {
    ROS_WARN("%s/local_bias is not set. using 0.3",nh.getNamespace().c_str());
    local_bias_=0.3;
  }
  if (local_bias_<0)
  {
    ROS_WARN("%s/local_bias cannot be negative, set equal to 0",nh.getNamespace().c_str());
    local_bias_=0.0;
  }
  if (local_bias_>1)
  {
    ROS_WARN("%s/local_bias cannot be greater than 1.0, set equal to 1.0",nh.getNamespace().c_str());
    local_bias_=1.0;
  }

  if (!nh.getParam("tube_radius",tube_radius_))
  {
    ROS_WARN("%s/tube_radius is not set. using 0.01",nh.getNamespace().c_str());
    tube_radius_=0.3;
  }
  if (tube_radius_<=0)
  {
    ROS_WARN("%s/tube_radius cannot be negative, set equal to 0.01",nh.getNamespace().c_str());
    tube_radius_=0.01;
  }
  if (!nh.getParam("forgetting_factor",forgetting_factor_))
  {
    ROS_WARN("%s/forgetting_factor is not set. using 0.01",nh.getNamespace().c_str());
    forgetting_factor_=0.01;
  }
  if (forgetting_factor_<=0)
  {
    ROS_WARN("%s/forgetting_factor cannot be negative, set equal to 0.0",nh.getNamespace().c_str());
    forgetting_factor_=0.0;
  }

  configured_=true;
  return true;
}

bool MultigoalSolver::initGoalSelector()
{
  goal_manager_ = std::make_shared<multi_goal_selection::GoalSelectionManager>(nh_.getNamespace(), goal_nodes_.size(),sampler_->getDimension());
  if (goal_manager_->isWarmStartSet())
    goal_manager_->warmStart(costs_,utopias_,cost_);
  return true;
}


bool MultigoalSolver::update(PathPtr& solution)
{
  if (!init_)
  {
    ROS_WARN("MultigoalSolver is not initialized");
    return false;
  }
  if (cost_ <= utopia_tolerance_ * best_utopia_)
  {
    ROS_DEBUG("Find the final solution");
    solution=solution_;
    completed_=true;
    return false;
  }


//  ROS_INFO("update probabilities");

  if (goal_probabilities_.size()==1)
    goal_probabilities_.at(0)=1.0;
  else
    goal_probabilities_ = goal_manager_->calculateProbabilities(were_goals_sampled_,costs_,utopias_,cost_);

  std::fill(were_goals_sampled_.begin(), were_goals_sampled_.end(), false);

  bool global_improvement=false;
  double old_cost=cost_;

  for (unsigned int igoal=0;igoal<goal_nodes_.size();igoal++)
  {
    if (utopias_.at(igoal)>cost_)
      continue;
    if (goal_probabilities_.at(igoal)<=0.0)
      continue;
    else if (goal_probabilities_.at(igoal)>=1.0);
    else if (ud_(gen_)>goal_probabilities_.at(igoal))
      continue;

    were_goals_sampled_.at(igoal) = true;

    NodePtr new_start_node, new_goal_node;
    bool add_to_start, add_to_goal;
    Eigen::VectorXd configuration;
    if (mixed_strategy_)
      configuration = tube_samplers_.at(igoal)->sample();
    else
      configuration = samplers_.at(igoal)->sample();
    bool is_goal_biased=ud_(gen_)<goal_bias_;

    bool improved=false;

    if (costs_.at(igoal)<utopias_.at(igoal))
    {
      ROS_ERROR("GOAL %d, COST %f, UTOPIA %f",igoal,costs_.at(igoal),utopias_.at(igoal));
      solved_=false;
      return false;
    }

    switch (status_.at(igoal))
    {

    case GoalStatus::search:

      if (not bidirectional_) // if only start tree grows
      {
        if (is_goal_biased) // if it is goal biased try to connect goal
        {
          if (extend_)
            add_to_start = start_tree_->extendToNode(goal_nodes_.at(igoal), new_start_node);
          else
            add_to_start = start_tree_->connectToNode(goal_nodes_.at(igoal), new_start_node);

          if (new_start_node==goal_nodes_.at(igoal))
          {
            ROS_INFO_THROTTLE(0.1,"GOAL BIAS connected!");
            improved=true;
            goal_trees_.at(igoal)->cleanTree();
          }
        }
        else // not is_goal_bias, add a new random node
        {
          if (extend_)
            add_to_start = start_tree_->extend(configuration, new_start_node);
          else
            add_to_start = start_tree_->connect(configuration, new_start_node);
          if (add_to_start) // if a new node is added, check if it near to the goal
          {
            if (metrics_->utopia(new_start_node,goal_nodes_.at(igoal))<max_distance_)
            {
              // if yes, try to extend to the goal
              ROS_INFO_THROTTLE(0.1,"try connect the goal");
              add_to_start = start_tree_->extendToNode(goal_nodes_.at(igoal), new_start_node);
              if (new_start_node==goal_nodes_.at(igoal)) // a solution is found
              {
                improved=true;
                goal_trees_.at(igoal)->cleanTree();
              }
            }
          }
        }
      }
      else  // birectional
      {
        if (is_goal_biased) // if it is goal biased try to connect goal
        {
          if (extend_)
            add_to_start = start_tree_->extendToNode(goal_nodes_.at(igoal), new_start_node);
          else
            add_to_start = start_tree_->connectToNode(goal_nodes_.at(igoal), new_start_node);

          if (new_start_node==goal_nodes_.at(igoal))
          {
            improved=true;
            goal_trees_.at(igoal)->cleanTree();
          }
        }
        else // not is_goal_bias, add a new random node
        {
          // add node to the start tree
          add_to_start = extend_? start_tree_->extend(configuration, new_start_node):
                                  start_tree_->connect(configuration, new_start_node);

          add_to_start = add_to_start?(configuration-new_start_node->getConfiguration()).norm()<1e-6: false;

          add_to_goal = extend_? goal_trees_.at(igoal)->extend(configuration, new_goal_node):
                                 goal_trees_.at(igoal)->connect(configuration, new_goal_node);

          add_to_goal = add_to_goal?(configuration-new_goal_node->getConfiguration()).norm()<1e-6: false;

          if (add_to_start && add_to_goal) // a solution is found
          {
            NodePtr parent=new_goal_node->getParents().at(0);
            double cost_to_parent=new_goal_node->parent_connections_.at(0)->getCost();
            new_goal_node->disconnect();

            //goal_trees_.at(igoal)->keepOnlyThisBranch(goal_trees_.at(igoal)->getConnectionToNode(parent));

            std::vector<ConnectionPtr> connections=goal_trees_.at(igoal)->getConnectionToNode(parent);
            for (ConnectionPtr& conn: connections)
              conn->flip();

            ConnectionPtr conn_to_goal_parent=std::make_shared<Connection>(new_start_node,parent);
            conn_to_goal_parent->add();
            conn_to_goal_parent->setCost(cost_to_parent);
            start_tree_->addNode(parent,false);
            for (ConnectionPtr& conn: connections)
              start_tree_->addNode(conn->getChild(),false);
            improved=true;
          }
        }
      }

      if (improved)
      {
        solutions_.at(igoal) = std::make_shared<Path>(start_tree_->getConnectionToNode(goal_nodes_.at(igoal)), metrics_, checker_);
        solutions_.at(igoal)->setTree(start_tree_);

        double cost_1=solutions_.at(igoal)->cost();

        if (first_warp_)
        {
          ros::WallTime twarp=ros::WallTime::now();
          solutions_.at(igoal)->warp();
          ROS_DEBUG("warp: cost from %f to %f in %f second",cost_1,solutions_.at(igoal)->cost(),(ros::WallTime::now()-twarp).toSec());

          double cost_0=solutions_.at(igoal)->cost();
          ros::WallTime tsimpl=ros::WallTime::now();
          solutions_.at(igoal)->simplify();
          ROS_DEBUG("simplify: cost from %f to %f in %f second",cost_0,solutions_.at(igoal)->cost(),(ros::WallTime::now()-tsimpl).toSec());
        }
        tube_samplers_.at(igoal)->setPath(solutions_.at(igoal));
        tube_samplers_.at(igoal)->setRadius(tube_radius_*solutions_.at(igoal)->cost());
        path_costs_.at(igoal) = solutions_.at(igoal)->cost();
        costs_.at(igoal) = path_costs_.at(igoal)+goal_costs_.at(igoal);

        solved_ = true;
        if (costs_.at(igoal)<=(utopias_.at(igoal)*utopia_tolerance_))
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



      if (not knearest_)
      {
        double dimension_ = (double) dof_;
        double r_rrt=1.1*  std::pow(2.0*(1.0+1.0/dimension_),1.0/dimension_)*std::pow(samplers_.at(igoal)->getSpecificVolume(),1.0/dimension_);
        double cardDbl=start_tree_->getNumberOfNodes()+1.0;
        r_rewire_=r_rrt * std::pow(log(cardDbl) / cardDbl, 1.0 /dimension_);
        //ROS_WARN_THROTTLE(0.1,"dof = %f, r_rrt =%f, cardDbl=%f, spec vol =%f, rewire radius %f", dimension_,r_rrt, cardDbl, samplers_.at(igoal)->getSpecificVolume(), r_rewire_);
        improved=start_tree_->rewire(configuration, r_rewire_,new_start_node);
      }
      else
      {
        improved=start_tree_->rewireK(configuration);
      }
      if (improved)
      {
        if (start_tree_->costToNode(goal_nodes_.at(igoal)) >= (path_costs_.at(igoal) - 1e-8))
          continue;

        // Often breaks here with many collision objcts (getConnectionToNode)
        solutions_.at(igoal) = std::make_shared<Path>(start_tree_->getConnectionToNode(goal_nodes_.at(igoal)), metrics_, checker_);
        solutions_.at(igoal)->setTree(start_tree_);
        double cost_1=solutions_.at(igoal)->cost();
        if (warp_)
        {
          ros::WallTime twarp=ros::WallTime::now();

          solutions_.at(igoal)->warp();

          ROS_DEBUG("warp: cost from %f to %f in %f second",cost_1,solutions_.at(igoal)->cost(),(ros::WallTime::now()-twarp).toSec());

          double cost_0=solutions_.at(igoal)->cost();
          ros::WallTime tsimpl=ros::WallTime::now();
          solutions_.at(igoal)->simplify();
          ROS_DEBUG("simplify: cost from %f to %f in %f second",cost_0,solutions_.at(igoal)->cost(),(ros::WallTime::now()-tsimpl).toSec());
        }

        tube_samplers_.at(igoal)->setPath(solutions_.at(igoal));
        tube_samplers_.at(igoal)->setRadius(tube_radius_*solutions_.at(igoal)->cost());
        path_costs_.at(igoal) = solutions_.at(igoal)->cost();
        costs_.at(igoal) = path_costs_.at(igoal)+goal_costs_.at(igoal);
        if (costs_.at(igoal)<=(utopias_.at(igoal)*utopia_tolerance_))
        {
          ROS_DEBUG("Goal %u reaches its utopia. cost = %f, utopia =%f",igoal,costs_.at(igoal),utopias_.at(igoal));
          cleanTree();
          status_.at(igoal)=GoalStatus::done;
        }
        else
          ROS_DEBUG("Goal %u refines solution with cost %f",igoal,costs_.at(igoal));
        global_improvement=isBestSolution(igoal) || global_improvement;
      }
      else if (new_start_node)  // is an improvement?????
      {
        // try connect with the goal
        double cost_to_goal=metrics_->cost(new_start_node,goal_nodes_.at(igoal));
        if (cost_to_goal<max_distance_ && (start_tree_->costToNode(new_start_node)+cost_to_goal)<path_costs_.at(igoal))
        {
          if (checker_->checkPath(new_start_node->getConfiguration(),
                                  goal_nodes_.at(igoal)->getConfiguration()))
          {
            goal_nodes_.at(igoal)->parent_connections_.at(0)->remove();
            ConnectionPtr conn=std::make_shared<Connection>(new_start_node,goal_nodes_.at(igoal));
            conn->setCost(cost_to_goal);
            conn->add();
            solutions_.at(igoal) = std::make_shared<Path>(start_tree_->getConnectionToNode(goal_nodes_.at(igoal)), metrics_, checker_);
            solutions_.at(igoal)->setTree(start_tree_);
            double cost_1=solutions_.at(igoal)->cost();
            if (warp_)
            {
              ros::WallTime twarp=ros::WallTime::now();
              for (int iwarp=0;iwarp<10;iwarp++)
              {
                solutions_.at(igoal)->warp();
              }
              ROS_DEBUG("warp: cost from %f to %f in %f second",cost_1,solutions_.at(igoal)->cost(),(ros::WallTime::now()-twarp).toSec());

              double cost_0=solutions_.at(igoal)->cost();
              ros::WallTime tsimpl=ros::WallTime::now();
              solutions_.at(igoal)->simplify();
              ROS_DEBUG("simplify: cost from %f to %f in %f second",cost_0,solutions_.at(igoal)->cost(),(ros::WallTime::now()-tsimpl).toSec());
            }

            tube_samplers_.at(igoal)->setPath(solutions_.at(igoal));
            tube_samplers_.at(igoal)->setRadius(tube_radius_*solutions_.at(igoal)->cost());
            path_costs_.at(igoal) = solutions_.at(igoal)->cost();
            costs_.at(igoal) = path_costs_.at(igoal)+goal_costs_.at(igoal);
            if (costs_.at(igoal)<=(utopias_.at(igoal)*utopia_tolerance_))
            {
              ROS_DEBUG("Goal %u reaches its utopia. cost = %f, utopia =%f",igoal,costs_.at(igoal),utopias_.at(igoal));
              cleanTree();
              status_.at(igoal)=GoalStatus::done;
            }
            else
              ROS_DEBUG("Goal %u refines solution with cost %f",igoal,costs_.at(igoal));
            global_improvement=isBestSolution(igoal) || global_improvement;
          }
        }
      }
      break;
    case GoalStatus::discard:
    case GoalStatus::done:
      break;
    }
  }

  if (solved_)
  {
    local_bias_=std::min(forgetting_factor_*local_bias_+reward_*(old_cost-cost_)/(old_cost-best_utopia_),1.0);
    for (TubeInformedSamplerPtr& sampler: tube_samplers_)
    {
      sampler->setLocalBias(local_bias_);
    }
  }
  solution = solution_;
  return global_improvement;
}


void MultigoalSolver::printMyself(std::ostream &os) const
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
    os << ". volume = " << std::scientific <<tube_samplers_.at(igoal)->getSpecificVolume()<< std::defaultfloat;

    os<<std::endl;
  }
}

void MultigoalSolver::cleanTree()
{
  std::vector<NodePtr> white_list=goal_nodes_;
  white_list.push_back(start_tree_->getRoot());
  std::vector<SamplerPtr> samplers;
  for (unsigned int igoal=0;igoal<goal_nodes_.size();igoal++)
  {
    if ((status_.at(igoal)==GoalStatus::refine)||(status_.at(igoal)==GoalStatus::search))
      samplers.push_back(tube_samplers_.at(igoal));
  }


  for (const PathPtr& sol: solutions_)
  {
    if (sol)
      for (const ConnectionPtr& conn: sol->getConnections())
        white_list.push_back(conn->getChild());
  }
  start_tree_->purgeNodesOutsideEllipsoids(samplers,white_list);
}

TreeSolverPtr MultigoalSolver::clone(const MetricsPtr& metrics, const CollisionCheckerPtr& checker, const SamplerPtr& sampler)
{
  MultigoalSolverPtr new_solver = std::make_shared<MultigoalSolver>(metrics,checker,sampler);
  new_solver->config(nh_);
  return new_solver;

}

std::vector<TreePtr> MultigoalSolver::getGoalTrees()
{
  return goal_trees_;
}

}  // namespace pathplan
