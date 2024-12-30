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

#include <functional>
#include <graph_core/graph/subtree.h>
#include <graph_core/util.h>

namespace graph
{
namespace core
{
/**
 * @class Net
 * @brief Class for defining a net, i.e. a graph that extends the tree allowing
 * for multiple parent connections for the same node.
 */
class Net;
typedef std::shared_ptr<Net> NetPtr;

class Net : public std::enable_shared_from_this<Net>
{
#define NET_ERROR_TOLERANCE 1e-06

protected:
  /**
   * @brief Pointer to the tree linked to the Net.
   */
  TreePtr linked_tree_;

  /**
   * @brief Pointer to the metrics used for the search.
   */
  MetricsPtr metrics_;

  /**
   * @brief Wall time representing the start time of the search.
   */
  std::chrono::time_point<std::chrono::system_clock> tic_search_;

  /**
   * @brief Vector to store the time values during the search.
   */
  std::vector<double> time_vector_;

  /**
   * @brief Vector to store nodes that should be excluded from the search.
   */
  std::vector<NodePtr> black_list_;

  /**
   * @brief Vector to store nodes that have been visited during the search.
   */
  std::vector<NodePtr> visited_nodes_;

  /**
   * @brief Vector to store connections leading to parent nodes during the
   * search.
   */
  std::vector<ConnectionPtr> connections2parent_;

  /**
   * @brief Multimap to paths along with their associated cost, used for sorting
   * solutions.
   */
  std::multimap<double, std::vector<ConnectionPtr>> map_;

  /**
   * @brief Shared pointer to a function representing a condition for evaluating
   * the cost of a connection. It should return true if net should re-evaluate
   * connection cost, false otherwise. You can set this condition using
   * costReEvaluationCondition. By default, it is a nullptr; The function should
   * take a ConnectionPtr as an argument and return a boolean value.
   */
  std::shared_ptr<std::function<bool(const ConnectionPtr& connection)>> cost_evaluation_condition_;

  /**
   * @brief Current time in the search process.
   */
  double time_;

  /**
   * @brief Maximum allowed time for the search.
   */
  double max_time_;

  /**
   * @brief Cost threshold to beat during the search.
   */
  double cost_to_beat_;

  /**
   * @brief Flag indicating whether verbose output is enabled during the search.
   */
  bool verbose_;

  /**
   * @brief Flag indicating whether to check whether nodes belong to the tree.
   */
  bool search_in_tree_;

  /**
   * @brief Flag indicating whether the search continues to find every solution.
   */
  bool search_every_solution_;

  /**
   * @brief Curse of dimensionality counter.
   */
  int curse_of_dimensionality_;

  /**
   * @brief Pointer to a TraceLogger instance for logging.
   *
   * This member variable represents a pointer to a TraceLogger instance,
   * allowing to perform logging operations. TraceLogger is a part of the
   * cnr_logger library. Ensure that the logger is properly configured and
   * available for use.
   */
  cnr_logger::TraceLoggerPtr logger_;

  /**
   * @brief Searches for paths in the net from a start node to a goal node.
   *
   * This function performs a depth-first search to compute paths from a start
   * node to a goal node. When the cost to come + cost to go is greater than
   * cost2beat, a node is not expanded. cost_evaluation_condition_ defines when
   * the cost of a connection should be evaluated.
   *
   * @param start_node The start node for the search.
   * @param goal_node The goal node for the search.
   * @param cost2here The cost to reach the current node.
   */
  void computeConnectionFromNodeToNode(const NodePtr& start_node, const NodePtr& goal_node, const double& cost2here);
  void computeConnectionFromNodeToNode(const NodePtr& start_node, const NodePtr& goal_node);
  void computeConnectionFromNodeToNode(const NodePtr& start_node, const NodePtr& goal_node, const double& cost2here,
                                       const double& cost2beat);

public:
  /**
   * @brief Constructor for the Net class.
   * It gets the metrics from tree_
   *
   * @param tree The tree to be associated with the Net.
   * @param logger The cnr_logger::TraceLoggerPtr logger for logging operations.
   */
  Net(const TreePtr& tree, const cnr_logger::TraceLoggerPtr& logger);

  /**
   * @brief Set the tree associated with the Net.
   *
   * @param tree The tree to be associated with the Net.
   */
  void setTree(const TreePtr& tree)
  {
    linked_tree_ = tree;
  }

  /**
   * @brief Set the metrics for the Net.
   *
   * @param metrics The metrics to be set for the Net.
   */
  void setMetrics(const MetricsPtr metrics)
  {
    metrics_ = metrics;
  }

  /**
   * @brief Get the tree associated with the Net.
   *
   * @return The tree associated with the Net.
   */
  TreePtr getTree()
  {
    return linked_tree_;
  }

  /**
   * @brief Set the verbosity of the Net.
   *
   * @param verbose Flag indicating whether verbose output is enabled.
   */
  void setVerbosity(const bool verbose)
  {
    verbose_ = verbose;
  }

  /**
   * @brief Set whether the Net should search for every solution.
   *
   * @param search_every_solution Flag indicating whether to search for every
   * solution.
   */
  void searchEverySolution(const bool search_every_solution)
  {
    search_every_solution_ = search_every_solution;
  }

  /**
   * @brief Set the cost evaluation condition for the Net.
   *
   * @param condition The cost evaluation condition to be set.
   */
  void
  setCostEvaluationCondition(const std::shared_ptr<std::function<bool(const ConnectionPtr& connection)>>& condition)
  {
    cost_evaluation_condition_ = condition;
  }

  /**
   * @brief Get connections to a node with specific search parameters,
   * considering the root as starting node.
   *
   * This function retrieves connections to a specific node while considering
   * various search parameters such as cost constraint, black list, and maximum
   * time. Exploits the computeConnectionFromNodeToNode method.
   *
   * @param node The target node for the search.
   * @param cost2beat The cost to beat during the search. Nodes with cost to
   * come plus cost to go gretaer than cost2beat are excluded. Solutions with
   * cost greater than this cost2beat are excluded.
   * @param black_list List of nodes to not expand during the search.
   * @param max_time The maximum time allowed for the search.
   * @return A multimap containing the vectors of connections found, sorted by
   * their cost.
   */
  std::multimap<double, std::vector<ConnectionPtr>>&
  getConnectionToNode(const NodePtr& node, const std::vector<NodePtr>& black_list = {},
                      const double& max_time = std::numeric_limits<double>::infinity());
  std::multimap<double, std::vector<ConnectionPtr>>&
  getConnectionToNode(const NodePtr& node, const double& cost2beat, const std::vector<NodePtr>& black_list = {},
                      const double& max_time = std::numeric_limits<double>::infinity());

  /**
   * @brief Get connections between two nodes with specific search parameters.
   *
   * This function retrieves connections between two nodes while considering
   * various search parameters such as cost constraint, black list, maximum
   * time. Exploits the computeConnectionFromNodeToNode method.
   *
   * @param start_node The start node for the search.
   * @param goal_node The goal node for the search.
   * @param cost2beat The cost to beat during the search. Nodes with cost to
   * come plus cost to go gretaer than cost2beat are excluded. Solutions with
   * cost greater than this cost2beat are excluded.
   * @param black_list List of nodes to not expand during the search.
   * @param max_time The maximum time allowed for the search.
   * @param search_in_tree Flag indicating whether to check if nodes belong to
   * the tree.
   * @return A multimap containing the vectors of connections found, sorted by
   * their cost.
   */
  std::multimap<double, std::vector<ConnectionPtr>>& getConnectionBetweenNodes(
      const NodePtr& start_node, const NodePtr& goal_node, const double& cost2beat,
      const std::vector<NodePtr>& black_list = {}, const double& max_time = std::numeric_limits<double>::infinity(),
      const bool& search_in_tree = false);
  std::multimap<double, std::vector<ConnectionPtr>>& getConnectionBetweenNodes(
      const NodePtr& start_node, const NodePtr& goal_node, const std::vector<NodePtr>& black_list = {},
      const double& max_time = std::numeric_limits<double>::infinity());
};

}  // end namespace core
}  // end namespace graph
