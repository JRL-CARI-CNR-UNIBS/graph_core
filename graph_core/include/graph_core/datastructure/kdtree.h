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
#pragma once

#include <graph_core/datastructure/nearest_neighbors.h>
namespace graph
{
namespace core
{

class KdTree;
typedef std::shared_ptr<KdTree> KdTreePtr;
class KdNode;
typedef std::shared_ptr<KdNode> KdNodePtr;
typedef std::weak_ptr<KdNode> KdNodeWeakPtr;

enum SearchDirection {Left,Right};

/**
 * @brief Node class for the k-d tree data structure.
 *
 * KdNode represents a node in the k-d tree, a spatial partitioning data structure.
 * It stores a node, its dimension, and pointers to its left and right children.
 * The class provides various functions for tree operations such as insertion,
 * searching for the nearest neighbor, finding nodes within a certain radius,
 * and finding k-nearest neighbors.
 */
class KdNode: public std::enable_shared_from_this<KdNode>
{
  friend class KdTree;

public:

  /**
   * @brief Constructor for KdNode.
   *
   * @param node The node to be stored in the KdNode.
   * @param dimension The dimension along which the tree is split.
   */
  KdNode(const NodePtr& node,
         const int& dimension,
         const cnr_logger::TraceLoggerPtr& logger);
  /**
   * @brief Destructor for the KdNode class.
   */
  ~KdNode();

  /**
   * @brief Get the stored node.
   * @return The stored node.
   */
  NodePtr node();

  /**
   * @brief Get the dimension along which the tree is split.
   * @return The dimension value.
   */
  int dimension();

  /**
   * @brief Get a shared pointer to this KdNode.
   * @return A shared pointer to this KdNode.
   */
  KdNodePtr pointer()
  {
    return shared_from_this();
  }

  /**
   * @brief Get the left child of the node.
   * @return The left child of the node.
   */
  KdNodePtr left();

  /**
   * @brief Get the right child of the node.
   * @return The right child of the node.
   */
  KdNodePtr right();

  /**
   * @brief Get the parent of the node.
   * @return The parent of the node.
   */
  KdNodeWeakPtr parent();

  /**
   * @brief Set the left child of the node.
   * @param kdnode The left child to set.
   */
  void left(const KdNodePtr& kdnode);

  /**
   * @brief Set the right child of the node.
   * @param kdnode The right child to set.
   */
  void right(const KdNodePtr& kdnode);

  /**
   * @brief Set the parent of the node.
   * @param kdnode The parent to set.
   */
  void parent(const KdNodeWeakPtr &kdnode);

  /**
   * @brief Delete the node from the tree.
   * @param disconnect_node Flag indicating whether to disconnect the node from the graph/tree.
   */
  void deleteNode(const bool& disconnect_node=false);

  /**
   * @brief Restore the node in the tree setting deleted_ flag false.
   */
  void restoreNode();

  /**
   * @brief Insert a node into the tree.
   * @param node The node to insert.
   */
  void insert(const NodePtr& node);

  /**
   * @brief Find the node with the minimum value in the given dimension.
   * @param dim The dimension along which to find the minimum.
   * @return The KdNode with the minimum value in the specified dimension.
   */
  KdNodePtr findMin(const int& dim);

  /**
   * @brief Find the nearest neighbor to a given configuration.
   * @param configuration The target configuration for finding the nearest neighbor.
   * @param best The node that is the nearest neighbor.
   * @param best_distance The distance to the nearest neighbor.
   */
  void nearestNeighbor(const Eigen::VectorXd& configuration,
                       NodePtr &best,
                       double &best_distance);

  /**
   * @brief Find nodes within a certain radius of a given configuration.
   * @param configuration The target configuration.
   * @param radius The search radius.
   * @param nodes Multimap to store nodes within the specified radius.
   */
  void near(const Eigen::VectorXd& configuration,
            const double& radius,
            std::multimap<double, NodePtr> &nodes);

  /**
   * @brief Find k-nearest neighbors to a given configuration.
   * @param configuration The target configuration.
   * @param k The number of nearest neighbors to find.
   * @param nodes Multimap to store k-nearest neighbors.
   */
  void kNearestNeighbors(const Eigen::VectorXd& configuration,
                         const size_t& k,
                         std::multimap<double,NodePtr>& nodes);

  /**
   * @brief Find a specific node in the tree.
   * @param node The node to search for.
   * @param kdnode A reference to a pointer that will store the found KdNode.
   * @return True if the node is found, false otherwise.
   */  bool findNode(const NodePtr& node,
                     KdNodePtr& kdnode);

  /**
   * @brief Get all nodes in the tree.
   * @param nodes Vector to store all nodes in the tree.
   */
  void getNodes(std::vector<NodePtr>& nodes);

  /**
   * @brief Disconnect nodes not present in a white list.
   * @param white_list Vector of nodes to keep connected.
   */
  void disconnectNodes(const std::vector<NodePtr>& white_list);

  /**
   * @brief Output stream operator for a KdNode.
   *
   * This operator allows streaming a textual representation of the KdNode to an output stream.
   *
   * @param os The output stream where the KdNode information will be printed.
   * @param kdnode The KdNode to be printed.
   * @return A reference to the output stream for chaining.
   */
  friend std::ostream& operator<<(std::ostream& os, const KdNode& kdnode);

  friend std::ostream& operator<<(std::ostream& os, const KdTree& kdtree);

protected:

  /**
   * @brief node_ Shared pointer to the associated Node.
   */
  NodePtr node_;

  /**
   * @brief parent_ Weak pointer to the parent KdNode.
   */
  KdNodeWeakPtr parent_;

  /**
   * @brief left_ Shared pointer to the left child KdNode.
   */
  KdNodePtr left_;

  /**
   * @brief right_ Shared pointer to the right child KdNode.
   */
  KdNodePtr right_;

  /**
   * @brief dimension_ The dimension along which the k-d tree is split at this node.
   */
  int dimension_;

  /**
   * @brief deleted_ Flag indicating whether the node is deleted.
   * Deleted nodes are not actually removed from the tree but excluded in functions
   * like getNodes or nearestNeighbor based on this flag.
   */
  bool deleted_;

  /**
   * @brief Pointer to a TraceLogger instance for logging.
   *
   * This member variable represents a pointer to a TraceLogger instance, allowing
   * to perform logging operations. TraceLogger is a part of the cnr_logger library.
   * Ensure that the logger is properly configured and available for use.
   */
  const cnr_logger::TraceLoggerPtr& logger_;
};

/**
 * @class KdTree
 * @brief NearestNeighbors implementation using a k-d tree data structure for storing nodes.
 */
class KdTree: public NearestNeighbors
{
  friend class KdNode;

public:

  /**
   * @brief print_deleted_nodes_ Flag to decide whether to also consider deleted nodes when the << operator is used.
   */
  bool print_deleted_nodes_;

  /**
   * @brief Constructor for the KdTree class.
   */
  KdTree(const cnr_logger::TraceLoggerPtr &logger);

  /**
   * @brief Destructor for the KdTree class.
   */
  ~KdTree();

  /**
   * @brief Implementation of the insert function for adding a node to the k-d tree.
   *
   * @param node The node to be inserted.
   */
  virtual void insert(const NodePtr& node) override;

  /**
   * @brief Implementation of the clear function to clear the nearest neighbors data structure.
   * Additionally, it sets size_ and delted_nodes_ to zero and root_ to nullptr;
   * @return True if successful, false otherwise.
   */
  virtual bool clear();

  /**
   * @brief Find the node with the minimum value in the specified dimension.
   *
   * This function searches the k-d tree to find the node with the minimum value
   * in the specified dimension.
   *
   * @param dim The dimension to search for the minimum value.
   * @return A pointer to the node with the minimum value in the specified dimension.
   * If the k-d tree is empty, returns nullptr.
   */
  NodePtr findMin(const int& dim);

  /**
   * @brief Implementation of the nearestNeighbor function for finding the nearest neighbor in the k-d tree.
   *
   * @param configuration The configuration for which the nearest neighbor needs to be found.
   * @param best Reference to the pointer to the best-matching node.
   * @param best_distance Reference to the distance to the best-matching node.
   */
  virtual void nearestNeighbor(const Eigen::VectorXd& configuration,
                               NodePtr &best,
                               double &best_distance) override;

  /**
   * @brief Implementation of the near function for finding nodes within a specified radius in the k-d tree.
   *
   * @param configuration The reference configuration.
   * @param radius The search radius.
   * @return A multimap containing nodes and their distances within the specified radius.
   */
  virtual std::multimap<double, NodePtr> near(const Eigen::VectorXd& configuration,
                                              const double& radius) override;

  /**
   * @brief Implementation of the kNearestNeighbors function for finding k nearest neighbors in the k-d tree.
   *
   * @param configuration The reference configuration.
   * @param k The number of nearest neighbors to find.
   * @return A multimap containing k nodes and their distances.
   */
  virtual std::multimap<double,NodePtr> kNearestNeighbors(const Eigen::VectorXd& configuration,
                                                          const size_t& k) override;

  /**
   * @brief Implementation of the findNode function for checking if a node exists in the k-d tree.
   *
   * @param node The node to check.
   * @return True if the node exists, false otherwise.
   */
  virtual bool findNode(const NodePtr& node) override;

  /**
   * @brief Find a specific node in the k-d tree.
   *
   * This function searches the k-d tree to find a specific node.
   *
   * @param node The node to search for.
   * @param kdnode A reference to a pointer that will store the found KdNode.
   * @return True if the node is found, false otherwise. If found, kdnode will point to the corresponding KdNode.
   */
  bool findNode(const NodePtr& node,
                KdNodePtr& kdnode);

  /**
   * @brief Implementation of the deleteNode function for deleting a node from the k-d tree.
   *
   * This function removes the specified node from the KdTree. Optionally, it can disconnect the associate NodePtr.
   * If the number of deleted nodes surpasses the threshold defined by 'deleted_nodes_threshold_',
   * the KdTree is entirely reconstructed. During the reconstruction, all nodes marked with the 'deleted_'
   * flag set to true are excluded, except for the root node, which is retained as is regardless of its 'deleted_' status.
   *
   * @param node The node to delete.
   * @param disconnect_node If true, disconnect the node from the graph.
   * @return True if the deletion is successful, false otherwise.
   */
  virtual bool deleteNode(const NodePtr& node,
                          const bool& disconnect_node=false) override;

  /**
   * @brief Implementation of the restoreNode function for restoring a previously deleted node.
   *
   * @param node The node to restore.
   * @return True if the restoration is successful, false otherwise.
   */
  virtual bool restoreNode(const NodePtr& node) override;

  /**
   * @brief Implementation of the getNodes function for getting all nodes in the k-d tree.
   *
   * @return A vector containing all nodes in the k-d tree.
   */
  virtual std::vector<NodePtr> getNodes() override;

  /**
   * @brief Implementation of the disconnectNodes function for disconnecting nodes in the k-d tree.
   *
   * @param white_list A vector of nodes to be excluded from the disconnection process.
   */
  virtual void disconnectNodes(const std::vector<NodePtr>& white_list) override;

  /**
   * @brief deletedNodesThreshold Returns the deleted_nodes_threshold_,
   * which represents the number of nodes set as deleted beyond which the kdtree is built from scratch.
   * @return deleted_nodes_threshold_
   */
  unsigned int deletedNodesThreshold();

  /**
   * @brief deletedNodesThreshold Sets the value of deleted_nodes_threshold_
   * @param t The value of deleted_nodes_threshold_ to set.
   */
  void deletedNodesThreshold(const unsigned int t);

  /**
   * @brief Output stream operator for a KdTree.
   *
   * This operator allows streaming a textual representation of the KdTree to an output stream.
   *
   * @param os The output stream where the KdTree information will be printed.
   * @param kdtree The KdTree to be printed.
   * @return A reference to the output stream for chaining.
   */
  friend std::ostream& operator<<(std::ostream& os, const KdTree& kdtree);

protected:
  /**
   * @brief root_ Root node of the k-d tree.
   */
  KdNodePtr root_;

  /**
   * @brief deleted_nodes_threshold_ When the number of (nodes for which deleted_ == true) > deleted_nodes_threshold_
   * the KdTree is built from scratch
   */
  unsigned int deleted_nodes_threshold_;
};

} //end namespace core
} // end namespace graph
