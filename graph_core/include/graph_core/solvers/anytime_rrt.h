#pragma once
/*
Copyright (c) 2024, Manuel Beschi and Cesare Tonola, JRL-CARI CNR-STIIMA/UNIBS, manuel.beschi@unibs.it, c.tonola001@unibs.it
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

namespace graph
{
namespace core
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
  SamplerPtr improve_sampler_;
  TreePtr new_tree_;
  NodePtr tmp_goal_node_;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  AnytimeRRT():RRT(){} //set initialized_ false

  AnytimeRRT(const MetricsPtr& metrics,
             const CollisionCheckerPtr& checker,
             const SamplerPtr& sampler,
             const GoalCostFunctionPtr& goal_cost_fcn,
             const cnr_logger::TraceLoggerPtr& logger):
    RRT(metrics, checker, sampler, goal_cost_fcn, logger) //set initialized_ true
  {
    setParameters();
  }

  AnytimeRRT(const MetricsPtr& metrics,
             const CollisionCheckerPtr& checker,
             const SamplerPtr& sampler,
             const cnr_logger::TraceLoggerPtr& logger):
    RRT(metrics, checker, sampler, logger) //set initialized_ true
  {
    setParameters();
  }

  /**
   * @brief init Initialise the object, defining its main attributes. At the end of the function, the flag 'initialized_' is set to true and the object can execute its main functions.
   * @param metrics The metrics used to evaluate paths.
   * @param checker The collision checker for checking collisions.
   * @param sampler The sampler for generating random configurations.
   * @param goal_cost_fcn The function used to assign the cost of the goal. If it is not defined, the default cost function does not assign any cost to the goal.
   * @param logger The logger for logging messages.
   * @return True if correctly initialised, False if already initialised.
   */
  virtual bool init(const MetricsPtr& metrics,
                    const CollisionCheckerPtr& checker,
                    const SamplerPtr& sampler,
                    const GoalCostFunctionPtr& goal_cost_fcn,
                    const cnr_logger::TraceLoggerPtr& logger) override
  {
    if(not RRT::init(metrics,checker,sampler,goal_cost_fcn,logger)) //set initialized_ true
      return false;

    setParameters();

    return true;
  }

  virtual bool init(const MetricsPtr& metrics,
                    const CollisionCheckerPtr& checker,
                    const SamplerPtr& sampler,
                    const cnr_logger::TraceLoggerPtr& logger) override
  {
    if(not RRT::init(metrics,checker,sampler,logger)) //set initialized_ true
      return false;

    setParameters();

    return true;
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

  bool importFromSolver(const AnytimeRRTPtr& solver);
  bool importFromSolver(const TreeSolverPtr& solver) override;

  virtual bool solve(PathPtr& solution,
                     const unsigned int& max_iter = 100,
                     const double &max_time = std::numeric_limits<double>::infinity()) override;
  virtual bool config(const std::string &param_ns) override;
  virtual void resetProblem() override;
  virtual bool improveUpdate(const Eigen::VectorXd& point, PathPtr& solution);
  virtual bool improveUpdate(PathPtr& solution);
  virtual bool update(const NodePtr& n, PathPtr& solution) override;

};

} //end namespace core
} // end namespace graph
