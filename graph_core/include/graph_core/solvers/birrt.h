#pragma once
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

#include <graph_core/solvers/rrt.h>

namespace graph
{
namespace core
{
class BiRRT;
typedef std::shared_ptr<BiRRT> BiRRTPtr;

class BiRRT : public RRT
{
protected:
  TreePtr goal_tree_;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  BiRRT() : RRT()
  {
  }  // set initialized_ false

  BiRRT(const MetricsPtr& metrics, const CollisionCheckerPtr& checker, const SamplerPtr& sampler,
        const GoalCostFunctionPtr& goal_cost_fcn, const cnr_logger::TraceLoggerPtr& logger)
    : RRT(metrics, checker, sampler, goal_cost_fcn, logger)
  {
  }  // set initialized_ true

  BiRRT(const MetricsPtr& metrics, const CollisionCheckerPtr& checker, const SamplerPtr& sampler,
        const cnr_logger::TraceLoggerPtr& logger)
    : RRT(metrics, checker, sampler, logger)
  {
  }  // set initialized_ true

  virtual bool addGoal(const NodePtr& goal_node,
                       const double& max_time = std::numeric_limits<double>::infinity()) override;
  virtual bool update(PathPtr& solution) override;
  virtual bool update(const Eigen::VectorXd& configuration, PathPtr& solution) override;
  virtual bool update(const NodePtr& n, PathPtr& solution) override;

  bool importFromSolver(const BiRRTPtr& solver);
  bool importFromSolver(const TreeSolverPtr& solver) override;
};

}  // end namespace core
}  // end namespace graph
