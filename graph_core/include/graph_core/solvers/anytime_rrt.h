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

#include <graph_core/solvers/rrt.h>

namespace pathplan
{
#define FAILED_ITER 3
class AnytimeRRT;
typedef std::shared_ptr<AnytimeRRT> AnytimeRRTPtr;

class AnytimeRRT: public RRT
{
protected:
  double bias_;
  double delta_; //dist_bias and cost_bias update factor
  double cost_impr_;  //cost improvement factor (new cost < (1-cost_impr_)*path_cost_)
  double cost2beat_;
  TreePtr new_tree_;
  NodePtr tmp_goal_node_;

  bool solveWithRRT(PathPtr& solution,
                    const unsigned int& max_iter = 100,
                    const double &max_time = std::numeric_limits<double>::infinity());

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  AnytimeRRT(const MetricsPtr& metrics,
             const CollisionCheckerPtr& checker,
             const SamplerPtr& sampler): RRT(metrics, checker, sampler)
  {
    setParameters();
  }

  double getBias()
  {
    return bias_;
  }

  double getCost2Beat()
  {
    return cost2beat_;
  }

  double getDelta()
  {
    return delta_;
  }

  double getCostImpr()
  {
    return cost_impr_;
  }

  TreePtr getNewTree()
  {
    return new_tree_;
  }

  void setDelta(const double& delta = 0.1)
  {
    delta_ = delta;
  }

  void setCostImprovementFactor(const double& impr = 0.1)
  {
    cost_impr_ = impr;
  }

  void setParameters(const double& delta = 0.1, const double& impr = 0.1)
  {
    bias_ = 0.9;
    setDelta(delta);
    setCostImprovementFactor(impr);
  }

  bool improve(NodePtr &start_node,
               PathPtr& solution,
               const double &cost2beat,
               const unsigned int& max_iter = 100,
               const double &max_time = std::numeric_limits<double>::infinity());

  bool improve(NodePtr &start_node,
               PathPtr& solution,
               const unsigned int& max_iter = 100,
               const double &max_time = std::numeric_limits<double>::infinity());

  bool improve(NodePtr &start_node,
               NodePtr &goal_node,
               PathPtr& solution,
               const double &cost2beat,
               const unsigned int& max_iter = 100,
               const double &max_time = std::numeric_limits<double>::infinity());

  bool improve(NodePtr &start_node,
               NodePtr &goal_node,
               PathPtr& solution,
               const unsigned int& max_iter = 100,
               const double &max_time = std::numeric_limits<double>::infinity());

  void importFromSolver(const AnytimeRRTPtr& solver);
  void importFromSolver(const TreeSolverPtr& solver);

  virtual bool solve(PathPtr& solution,
                     const unsigned int& max_iter = 100,
                     const double &max_time = std::numeric_limits<double>::infinity()) override;
  virtual bool config(const ros::NodeHandle& nh) override;
  virtual void resetProblem() override;
  virtual bool update(const Eigen::VectorXd& point, PathPtr& solution) override;
  virtual bool update(const NodePtr& n, PathPtr& solution) override;
  virtual bool update(PathPtr& solution) override;
  virtual TreeSolverPtr clone(const MetricsPtr& metrics,
                              const CollisionCheckerPtr& checker,
                              const SamplerPtr& sampler) override;

};

}
