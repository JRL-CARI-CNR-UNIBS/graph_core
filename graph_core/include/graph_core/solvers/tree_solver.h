#pragma once
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

#include <graph_core/graph/tree.h>
#include <graph_core/graph/path.h>
#include <graph_core/collision_checker.h>
#include <graph_core/metrics.h>
#include <graph_core/sampler.h>
#include <graph_core/goal_cost_function.h>
#include <ros/ros.h>
#include <ros/duration.h>
namespace pathplan
{


class TreeSolver;
typedef std::shared_ptr<TreeSolver> TreeSolverPtr;
class TreeSolver: public std::enable_shared_from_this<TreeSolver>
{
protected:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  ros::NodeHandle nh_;
  MetricsPtr metrics_;
  CollisionCheckerPtr checker_;
  SamplerPtr sampler_;
  GoalCostFunctionPtr goal_cost_fcn_;
  bool solved_ = false;
  bool completed_=false;
  bool init_ = false;
  bool configured_=false;
  double path_cost_;
  double goal_cost_=0;
  double cost_=0;
  TreePtr start_tree_;
  PathPtr solution_;
  unsigned int dof_;

protected:
  virtual bool setProblem(const double &max_time = std::numeric_limits<double>::infinity())
  {
    return false;
  }
  virtual void clean(){}

  virtual void printMyself(std::ostream& os) const {}

public:
  TreeSolver(const MetricsPtr& metrics,
             const CollisionCheckerPtr& checker,
             const SamplerPtr& sampler):
    metrics_(metrics),
    checker_(checker),
    sampler_(sampler)
  {
    path_cost_ = std::numeric_limits<double>::infinity();
    goal_cost_ = 0.0;
    cost_ = std::numeric_limits<double>::infinity();
    goal_cost_fcn_=std::make_shared<GoalCostFunction>();
  }

  const double& cost() const
  {
    return cost_;
  }

  virtual bool config(const ros::NodeHandle& nh)
  {
    return false;
  }
  virtual bool update(PathPtr& solution) = 0;
  virtual bool update(const Eigen::VectorXd& point, PathPtr& solution){return false;}
  virtual bool update(const NodePtr& n, PathPtr& solution){return false;}

  virtual bool solve(PathPtr& solution, const unsigned int& max_iter = 100, const double &max_time = std::numeric_limits<double>::infinity());
  virtual bool addStart(const NodePtr& start_node, const double &max_time = std::numeric_limits<double>::infinity()) = 0;
  virtual bool addGoal(const NodePtr& goal_node, const double &max_time = std::numeric_limits<double>::infinity()) = 0;
  virtual void resetProblem()=0;
  virtual TreeSolverPtr clone(const MetricsPtr& metrics, const CollisionCheckerPtr& checker, const SamplerPtr& sampler) = 0;

  void setGoalCostFunction(const GoalCostFunctionPtr& goal_cost_fcn)
  {
    goal_cost_fcn_=goal_cost_fcn;
  }

  const bool& completed()const
  {
    return completed_;
  }

  const bool& solved()const
  {
    return solved_;
  }

  virtual bool setSolution(const PathPtr &solution, const bool& solved=false);
  TreePtr getStartTree() const
  {
    return start_tree_;
  }

  PathPtr getSolution() const
  {
    return solution_;
  }

  ros::NodeHandle getNodeHandle()
  {
    return nh_;
  }

  void setSampler(const SamplerPtr& sampler)
  {
    sampler_ = sampler;
  }

  SamplerPtr getSampler() const
  {
    return sampler_;
  }

  void setChecker(const CollisionCheckerPtr& checker)
  {
    checker_ = checker;
  }

  CollisionCheckerPtr getChecker() const
  {
    return checker_;
  }

  void setMetrics(const MetricsPtr& metrics)
  {
    metrics_ = metrics;
  }

  MetricsPtr getMetrics() const
  {
    return metrics_;
  }

  friend std::ostream& operator<<(std::ostream& os, const TreeSolver& solver);

};
inline std::ostream& operator<<(std::ostream& os, const TreeSolver& solver){solver.printMyself(os);return os;}

}  // namespace pathplan

