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

#include <graph_core/solvers/tree_solver.h>

namespace pathplan
{
class RRTConnect;
typedef std::shared_ptr<RRTConnect> RRTConnectPtr;

class RRTConnect: public TreeSolver
{
protected:
  NodePtr goal_node_;
  double max_distance_;
  double utopia_;
  virtual bool setProblem(const double &max_time = std::numeric_limits<double>::infinity());
  virtual void clean(){}

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  RRTConnect(const MetricsPtr& metrics,
             const CollisionCheckerPtr& checker,
             const SamplerPtr& sampler):
    TreeSolver(metrics, checker, sampler) {}
  virtual bool config(const ros::NodeHandle& nh);

  virtual bool addStart(const NodePtr& start_node, const double &max_time = std::numeric_limits<double>::infinity());
  virtual bool addStartTree(const TreePtr& start_tree);
  virtual bool addGoal(const NodePtr& goal_node, const double &max_time = std::numeric_limits<double>::infinity());

  virtual void resetProblem();
  virtual bool update(const Eigen::VectorXd& point, PathPtr& solution);
  virtual bool update(const NodePtr& n, PathPtr& solution);

  virtual bool update(PathPtr& solution);

  virtual TreeSolverPtr clone(const MetricsPtr& metrics, const CollisionCheckerPtr& checker, const SamplerPtr& sampler);

};

}
