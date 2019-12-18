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

#include <graph_core/graph_core.h>
#include <graph_core/graph.h>
#include <graph_core/path.h>
#include <graph_core/tree.h>
#include <random>

namespace ha_planner {

class Problem
{
protected:
  double m_estimated_cost;
  double m_utopia_cost;
  double m_tolerance=1e-6;
  PathPtr m_best_path;

  GraphPtr m_graph;
  NodeVct m_start_nodes;
  NodeVct m_end_nodes;
  std::vector<TreePtr> m_goal_trees;
  std::vector<TreePtr> m_start_trees;

  std::vector<Combination> m_combinations;
  std::map<Combination,PathPtr> m_best_path_per_combination;
  std::map<Combination,double> m_utopia_cost_per_combination;
  std::map<Combination,Eigen::MatrixXd> m_rot_matrix;


  std::random_device m_rd;
  std::mt19937 m_gen;
  std::uniform_real_distribution<double> m_ud;


  virtual std::vector<double> sample(const Combination& combination);
  bool IsASolution(const PathPtr& path, Combination& combination);

public:
  Problem(const GraphPtr& graph);

  virtual void generateNodesFromStartAndEndPoints(const std::vector<std::vector<double>>& start_points,
                                                  const std::vector<std::vector<double>>& end_points);


  bool solved();
  double getBestCost(){return m_best_path->getCost();}
  bool storeIfImproveCost(const PathPtr& path);
};

}
