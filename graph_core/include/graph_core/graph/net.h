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

#include <graph_core/util.h>
#include <graph_core/graph/subtree.h>
#include <functional>

namespace pathplan
{
class Net;
typedef std::shared_ptr<Net> NetPtr;

class Net: public std::enable_shared_from_this<Net>
{

#define NET_ERROR_TOLERANCE 1e-12

protected:
  TreePtr linked_tree_;
  MetricsPtr metrics_;

  ros::WallTime tic_search_;

  std::vector<double> time_vector_;
  std::vector<NodePtr> black_list_;
  std::vector<NodePtr> visited_nodes_;
  std::vector<ConnectionPtr> connections2parent_;
  std::multimap<double,std::vector<ConnectionPtr>> map_;

  /**
   * @brief cost_evaluation_condition_ is a pointer to a function which return true if net should re-evaluate connection cost, false otherwise.
   * You can set this condition using costReEvaluationCondition. By default, it is a nullptr;
   */
  std::shared_ptr<std::function<bool (const ConnectionPtr& connection)>> cost_evaluation_condition_;

  double time_;
  double max_time_;
  double cost_to_beat_;

  bool verbose_;
  bool search_in_tree_;
  bool search_every_solution_;

  int curse_of_dimensionality_;

  void computeConnectionFromNodeToNode(const NodePtr& start_node, const NodePtr& goal_node);
  void computeConnectionFromNodeToNode(const NodePtr& start_node, const NodePtr& goal_node, const double& cost2here);
  void computeConnectionFromNodeToNode(const NodePtr& start_node, const NodePtr& goal_node, const double& cost2here, const double& cost2beat);

public:
  Net(const TreePtr& tree);

  void setTree(const TreePtr& tree)
  {
    linked_tree_ = tree;
  }

  void setMetrics(const MetricsPtr metrics)
  {
    metrics_ = metrics;
  }

  TreePtr getTree()
  {
    return linked_tree_;
  }

  void setVerbosity(const bool verbose)
  {
    verbose_ = verbose;
  }

  void searchEverySolution(const bool search_every_solution)
  {
    search_every_solution_ = search_every_solution;
  }

  void setCostEvaluationCondition(const std::shared_ptr<std::function<bool (const ConnectionPtr& connection)>>& condition)
  {
    cost_evaluation_condition_ = condition;
  }

  std::multimap<double,std::vector<ConnectionPtr>>& getConnectionToNode(const NodePtr& node, const std::vector<NodePtr>& black_list = {}, const double& max_time = std::numeric_limits<double>::infinity());
  std::multimap<double,std::vector<ConnectionPtr>>& getConnectionToNode(const NodePtr& node, const double& cost2beat, const std::vector<NodePtr>& black_list = {}, const double& max_time = std::numeric_limits<double>::infinity());

  std::multimap<double,std::vector<ConnectionPtr>>& getConnectionBetweenNodes(const NodePtr& start_node, const NodePtr& goal_node, const std::vector<NodePtr> &black_list = {}, const double& max_time = std::numeric_limits<double>::infinity());
  std::multimap<double,std::vector<ConnectionPtr>>& getConnectionBetweenNodes(const NodePtr& start_node, const NodePtr& goal_node, const double& cost2beat, const std::vector<NodePtr> &black_list = {}, const double& max_time = std::numeric_limits<double>::infinity(), const bool &search_in_tree = false);
};

}
