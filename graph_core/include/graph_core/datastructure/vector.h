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

PSEUDO CODE :
- https://www.cs.cmu.edu/~ckingsf/bioinfo-lectures/kdtrees.pdf
- http://andrewd.ces.clemson.edu/courses/cpsc805/references/nearest_search.pdf
*/

#include <graph_core/datastructure/nearest_neighbors.h>
namespace graph_core
{

/**
 * @class Vector
 * @brief NearestNeighbors implementation using a vector data structure for storing nodes.
 */
class Vector: public NearestNeighbors
{
public:

  /**
   * @brief Constructor for the Vector class.
   */
  Vector();

  /**
   * @brief Implementation of the insert function for adding a node to the vector.
   *
   * @param node The node to be inserted.
   */
  virtual void insert(const NodePtr& node) override;

  /**
   * @brief Implementation of the nearestNeighbor function for finding the nearest neighbor in the vector.
   *
   * @param configuration The configuration for which the nearest neighbor needs to be found.
   * @param best Reference to the pointer to the best-matching node.
   * @param best_distance Reference to the distance to the best-matching node.
   */
  virtual void nearestNeighbor(const Eigen::VectorXd& configuration,
                          NodePtr &best,
                          double &best_distance) override;

  /**
   * @brief Implementation of the near function for finding nodes within a specified radius in the vector.
   *
   * @param configuration The reference configuration.
   * @param radius The search radius.
   * @return A multimap containing nodes and their distances within the specified radius.
   */
  virtual std::multimap<double, NodePtr> near(const Eigen::VectorXd& configuration,
                            const double& radius) override;

  /**
   * @brief Implementation of the kNearestNeighbors function for finding k nearest neighbors in the vector.
   *
   * @param configuration The reference configuration.
   * @param k The number of nearest neighbors to find.
   * @return A multimap containing k nodes and their distances.
   */
  virtual std::multimap<double,NodePtr> kNearestNeighbors(const Eigen::VectorXd& configuration,
                                 const size_t& k) override;

  /**
   * @brief Implementation of the findNode function for checking if a node exists in the vector.
   *
   * @param node The node to check.
   * @return True if the node exists, false otherwise.
   */
  virtual bool findNode(const NodePtr& node) override;

  /**
   * @brief Implementation of the deleteNode function for deleting a node from the vector.
   *
   * @param node The node to delete.
   * @param disconnect_node If true, disconnect the node from the graph.
   * @return True if the deletion is successful, false otherwise.
   */
  virtual bool deleteNode(const NodePtr& node,
                  const bool& disconnect_node=false) override;

  /**
   * @brief Implementation of the restoreNode function for restoring a previously deleted node (not implemented).
   *
   * @param node The node to restore.
   * @return Always returns false as restoration is not implemented.
   */
  virtual bool restoreNode(const NodePtr& node) override;

  /**
   * @brief Implementation of the getNodes function for getting all nodes in the vector.
   *
   * @return A vector containing all nodes in the vector.
   */
  virtual std::vector<NodePtr> getNodes() override;

  /**
   * @brief Implementation of the disconnectNodes function for disconnecting nodes in the vector.
   *
   * @param white_list A vector of nodes to be excluded from the disconnection process.
   */
  virtual void disconnectNodes(const std::vector<NodePtr>& white_list) override;
protected:
  std::vector<NodePtr> nodes_;
};

}  // namespace pathplan
