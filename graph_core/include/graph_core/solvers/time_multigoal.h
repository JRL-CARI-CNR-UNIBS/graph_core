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

#pragma once
#include <graph_core/solvers/tree_solver.h>
#include <graph_core/solvers/rrt_star.h>
#include <graph_core/time_sampler.h>
namespace pathplan
{



class TimeMultigoalSolver: public TreeSolver
{
protected:
  enum GoalStatus { search, refine, done, discard};
  std::vector<NodePtr> goal_nodes_;
  std::vector<TreePtr> goal_trees_;
  std::vector<double> path_costs_;
  std::vector<double> goal_costs_;
  std::vector<double> costs_;
  std::vector<double> utopias_;
  std::vector<PathPtr> solutions_;
  std::vector<TimeBasedInformedSamplerPtr> time_samplers_;
  std::vector<GoalStatus> status_;
  TimeBasedInformedSamplerPtr time_based_sampler_;
  std::random_device rd_;
  std::mt19937 gen_;
  std::uniform_real_distribution<double> ud_;

  double cost_at_last_clean=std::numeric_limits<double>::infinity();
  double best_utopia_=std::numeric_limits<double>::infinity();
  int best_goal_index=-1;
  double max_distance_=1.0;
  double utopia_tolerance=1.003;
  double r_rewire_;
  bool extend_ = false;
  virtual bool setProblem(const double &max_time = std::numeric_limits<double>::infinity());
  virtual void clean(){}

  bool isBestSolution(const int& index);

  virtual void printMyself(std::ostream& os) const;
public:

  TimeMultigoalSolver(const MetricsPtr& metrics,
             const CollisionCheckerPtr& checker,
             const SamplerPtr& sampler,
             const Eigen::VectorXd& max_speed):
    TreeSolver(metrics, checker, sampler),
    gen_(time(0))
  {
    ud_ = std::uniform_real_distribution<double>(0, 1);
    time_based_sampler_ =  std::make_shared<TimeBasedInformedSampler>(sampler->getStartConf(),
                                                                      sampler->getStopConf(),
                                                                      sampler->getLB(),
                                                                      sampler->getUB(),
                                                                      max_speed,
                                                                      sampler->getCost());
  }

  virtual bool config(const ros::NodeHandle& nh);
  virtual bool update(PathPtr& solution);

  virtual bool addStart(const NodePtr& start_node, const double &max_time = std::numeric_limits<double>::infinity());
  virtual bool addGoal(const NodePtr& goal_node, const double &max_time = std::numeric_limits<double>::infinity());
  virtual bool addStartTree(const TreePtr& start_tree, const double &max_time = std::numeric_limits<double>::infinity()) override;
  virtual void resetProblem();
  virtual TreeSolverPtr clone(const MetricsPtr& metrics, const CollisionCheckerPtr& checker, const SamplerPtr& sampler);


  void cleanTree();

};

typedef std::shared_ptr<TreeSolver> TreeSolverPtr;


}  // namespace pathplan
