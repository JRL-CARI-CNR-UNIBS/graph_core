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

PSEUDO CODE :
- https://www.cs.cmu.edu/~ckingsf/bioinfo-lectures/kdtrees.pdf
- http://andrewd.ces.clemson.edu/courses/cpsc805/references/nearest_search.pdf
*/
#pragma once
#include <graph_core/graph/node.h>
namespace graph
{
namespace core
{
/**
 * @class NearestNeighbors
 * @brief Abstract base class for handling nearest neighbor search in a graph.
 */
class NearestNeighbors : public std::enable_shared_from_this<NearestNeighbors>
{
public:
  /**
   * @brief Constructor for the NearestNeighbors class.
   */
  NearestNeighbors(const cnr_logger::TraceLoggerPtr& logger) : logger_(logger)
  {
    deleted_nodes_ = 0;
    size_ = 0;
  }

  /**
   * @brief Pure virtual function to insert a node into the nearest neighbors
   * data structure.
   *
   * @param node The node to be inserted.
   */
  virtual void insert(const NodePtr& node) = 0;

  /**
   * @brief  Pure virtual function to clear the nearest neighbors data
   * structure. Additionally, it should set size_ and delted_nodes_ to zero.
   * @return True if successful, false otherwise.
   */
  virtual bool clear() = 0;

  /**
   * @brief Pure virtual function to find the nearest neighbor to a given
   * configuration.
   *
   * @param configuration The configuration for which the nearest neighbor needs
   * to be found.
   * @param best Reference to the pointer to the best-matching node.
   * @param best_distance Reference to the distance to the best-matching node.
   */
  virtual void nearestNeighbor(const Eigen::VectorXd& configuration, NodePtr& best, double& best_distance) = 0;

  /**
   * @brief Function to find the nearest neighbor to a given configuration.
   *
   * @param configuration The configuration for which the nearest neighbor needs
   * to be found.
   * @return The nearest neighbor node.
   */
  virtual NodePtr nearestNeighbor(const Eigen::VectorXd& configuration)
  {
    {
      double best_distance = std::numeric_limits<double>::infinity();
      NodePtr best;
      nearestNeighbor(configuration, best, best_distance);
      return best;
    }
  }

  /**
   * @brief Pure virtual function to find nodes within a specified radius of a
   * given configuration.
   *
   * @param configuration The reference configuration.
   * @param radius The search radius.
   * @return A multimap containing nodes and their distances within the
   * specified radius.
   */
  virtual std::multimap<double, NodePtr> near(const Eigen::VectorXd& configuration, const double& radius) = 0;

  /**
   * @brief Pure virtual function to find k nearest neighbors to a given
   * configuration.
   *
   * @param configuration The reference configuration.
   * @param k The number of nearest neighbors to find.
   * @return A multimap containing k nodes and their distances.
   */
  virtual std::multimap<double, NodePtr> kNearestNeighbors(const Eigen::VectorXd& configuration, const size_t& k) = 0;

  /**
   * @brief Pure virtual function to check if a node exists in the nearest
   * neighbors data structure.
   *
   * @param node The node to check.
   * @return True if the node exists, false otherwise.
   */
  virtual bool findNode(const NodePtr& node) = 0;

  /**
   * @brief Pure virtual function to delete a node from the nearest neighbors
   * data structure.
   *
   * @param node The node to delete.
   * @param disconnect_node If true, disconnect the node also from the
   * graph/tree.
   * @return True if the deletion is successful, false otherwise.
   */
  virtual bool deleteNode(const NodePtr& node, const bool& disconnect_node = false) = 0;

  /**
   * @brief Pure virtual function to restore a previously deleted node from the
   * data structure.
   *
   * @param node The node to restore.
   * @return True if the restoration is successful, false otherwise.
   */
  virtual bool restoreNode(const NodePtr& node) = 0;

  /**
   * @brief Function to get the size (number of nodes) in the nearest neighbors
   * data structure.
   *
   * @return The size of the nearest neighbors data structure.
   */
  virtual unsigned int size()
  {
    return size_;
  }

  /**
   * @brief Pure virtual function to get a vector of all nodes in the nearest
   * neighbors data structure.
   *
   * @return A vector containing all nodes in the nearest neighbors data
   * structure.
   */
  virtual std::vector<NodePtr> getNodes() = 0;

  /**
   * @brief Pure virtual function to disconnect nodes in the given white list
   * from the graph.
   *
   * @param white_list A vector of nodes to be excluded from the disconnection
   * process.
   */
  virtual void disconnectNodes(const std::vector<NodePtr>& white_list) = 0;

protected:
  /**
   * @brief size_ Number of nodes in the nearest neighbors data structure.
   */
  unsigned int size_;

  /**
   * @brief delete_nodes_ Number of deleted nodes.
   */
  unsigned int deleted_nodes_;

  /**
   * @brief Pointer to a TraceLogger instance for logging.
   *
   * This member variable represents a pointer to a TraceLogger instance,
   * allowing to perform logging operations. TraceLogger is a part of the
   * cnr_logger library. Ensure that the logger is properly configured and
   * available for use.
   */
  cnr_logger::TraceLoggerPtr logger_;
};

typedef std::shared_ptr<NearestNeighbors> NearestNeighborsPtr;

}  // end namespace core
}  // end namespace graph
