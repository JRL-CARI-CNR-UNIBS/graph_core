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
#include <ros/ros.h>
namespace pathplan
{


class TreeSolver;
typedef std::shared_ptr<TreeSolver> TreeSolverPtr;
class TreeSolver: public std::enable_shared_from_this<TreeSolver>
{
protected:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  MetricsPtr metrics_;
  CollisionCheckerPtr checker_;
  SamplerPtr sampler_;
  bool solved_ = false;
  bool init_ = false;
  double cost_;
  TreePtr start_tree_;
  PathPtr solution_;
  unsigned int dof_;
protected:
  virtual bool setProblem()
  {
    return false;
  }
  virtual void clean(){}

public:
  TreeSolver(const MetricsPtr& metrics,
             const CollisionCheckerPtr& checker,
             const SamplerPtr& sampler):
    metrics_(metrics),
    checker_(checker),
    sampler_(sampler)
  {
    cost_ = std::numeric_limits<double>::infinity();
  }
  virtual bool config(const ros::NodeHandle& nh)
  {
    return false;
  }
  virtual bool update(PathPtr& solution) = 0;
  virtual bool update(const Eigen::VectorXd& point, PathPtr& solution) = 0;
  virtual bool update(const NodePtr& n, PathPtr& solution)=0;

  virtual bool solve(PathPtr& solution, const unsigned int& max_iter = 100);
  virtual bool addStart(const NodePtr& start_node) = 0;
  virtual bool addGoal(const NodePtr& goal_node) = 0;
  const bool& solved()const
  {
    return solved_;
  }
  const double& cost()const
  {
    return cost_;
  }
  virtual bool setSolution(const PathPtr &solution, const bool& solved=false);
  TreePtr getStartTree() const
  {
    return start_tree_;
  }

  void setSampler(const SamplerPtr& sampler)
  {
    sampler_ = sampler;
  }


};
}
