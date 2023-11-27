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

#include <graph_core/graph/tree.h>

namespace pathplan
{

class Subtree;
typedef std::shared_ptr<Subtree> SubtreePtr;

class Subtree: public Tree
{
protected:
  /**
   * @brief TreePtr representing the parent tree of the subtree.
   *
   * This member variable holds a TreePtr that represents the parent tree of the subtree.
   */
  TreePtr parent_tree_;

  /**
   * @brief Populates the subtree inside the ellipsoid based on specified criteria.
   *
   * This method populates the subtree starting from the given root node inside an ellipsoid defined by
   * focus points and a cost threshold. It considers criteria such as cost, a blacklist of nodes to be excluded,
   * and performs collision checking if specified. If the root is not initially inside the ellipsoid, it still
   * populates the subtree without the ellipsoid constraints.
   *
   * @param root The root node of the subtree.
   * @param focus1 The first focus point used in the criteria for node inclusion.
   * @param focus2 The second focus point used in the criteria for node inclusion.
   * @param cost The cost threshold for node inclusion.
   * @param black_list A list of nodes to be excluded from the population.
   * @param node_check A flag indicating whether collision checking should be performed for each node.
   */
  void populateSubtreeInsideEllipsoid(const NodePtr& root,
                                      const Eigen::VectorXd& focus1,
                                      const Eigen::VectorXd& focus2,
                                      const double& cost,
                                      const std::vector<NodePtr> &black_list,
                                      const bool node_check = false);
public:

  /**
   * @brief Constructor for the Subtree class.
   *
   * This constructor initializes a Subtree instance with the specified parent tree and root node.
   * It populates the subtree from the given root node.
   *
   * @param parent_tree The parent tree from which the subtree is extracted.
   * @param root The root node of the subtree.
   */
  Subtree(const TreePtr& parent_tree,
          const NodePtr& root);

  /**
   * @brief Constructor for the Subtree class.
   *
   * Initializes a Subtree instance with the specified parent tree, root node, and black list of nodes.
   * Populates the subtree from the given root node, excluding nodes in the black list.
   *
   * @param parent_tree The parent tree from which the subtree is extracted.
   * @param root The root node of the subtree.
   * @param black_list A vector of nodes to be excluded from the subtree.
   */
  Subtree(const TreePtr& parent_tree,
          const NodePtr& root,
          const std::vector<NodePtr>& black_list);

  /**
   * @brief Constructor for the Subtree class.
   *
   * Initializes a Subtree instance with the specified parent tree, root node, ellipsoid foci, and cost.
   * Populates the subtree from the given root node, excluding nodes outside the ellipsoid defined by the foci and cost.
   *
   * @param parent_tree The parent tree from which the subtree is extracted.
   * @param root The root node of the subtree.
   * @param focus1 The first focus of the ellipsoid.
   * @param focus2 The second focus of the ellipsoid.
   * @param cost The cost associated with the ellipsoid.
   */
  Subtree(const TreePtr& parent_tree,
          const NodePtr& root,
          const Eigen::VectorXd& focus1,
          const Eigen::VectorXd& focus2,
          const double& cost);

  /**
   * @brief Constructor for the Subtree class.
   *
   * Initializes a Subtree instance with the specified parent tree, root node, ellipsoid foci, cost, black list, and node checking option.
   * Populates the subtree from the given root node, excluding nodes outside the ellipsoid defined by the foci and cost.
   * Optionally checks collision for each node based on the node_check flag.
   *
   * @param parent_tree The parent tree from which the subtree is extracted.
   * @param root The root node of the subtree.
   * @param focus1 The first focus of the ellipsoid.
   * @param focus2 The second focus of the ellipsoid.
   * @param cost The cost associated with the ellipsoid.
   * @param black_list A list of nodes to be excluded from the subtree.
   * @param node_check Flag to indicate whether collision checking should be performed for each node.
   */
  Subtree(const TreePtr& parent_tree,
          const NodePtr& root,
          const Eigen::VectorXd& focus1,
          const Eigen::VectorXd& focus2,
          const double& cost,
          const std::vector<NodePtr>& black_list,
          const bool node_check = false);

  /**
   * @brief Constructor for the Subtree class.
   *
   * Initializes a Subtree instance with the specified parent tree, root node, goal configuration, cost, black list, and node checking option.
   * Populates the subtree from the given root node, considering the cost to reach the goal and excluding nodes outside the cost bounds.
   * Optionally checks collision for each node based on the node_check flag.
   *
   * @param parent_tree The parent tree from which the subtree is extracted.
   * @param root The root node of the subtree.
   * @param goal The goal configuration for considering the cost.
   * @param cost The cost associated with reaching the goal configuration.
   * @param black_list A list of nodes to be excluded from the subtree.
   * @param node_check Flag to indicate whether collision checking should be performed for each node.
   */
  Subtree(const TreePtr& parent_tree,
          const NodePtr& root,
          const Eigen::VectorXd& goal,
          const double& cost,
          const std::vector<NodePtr>& black_list,
          const bool node_check = false);

  /**
   * @brief Get the parent tree of the Subtree.
   *
   * @return A TreePtr representing the parent tree of the Subtree.
   */
  TreePtr getParentTree()
  {
    return parent_tree_;
  }

  /**
   * @brief Check if the instance is a subtree.
   *
   * @return Returns true, indicating that the instance is a subtree.
   */
  virtual bool isSubtree() override
  {
    return true;
  }

  /**
   * @brief Hide a node and its successors from the subtree.
   *
   * This function hides the specified node and its successors from the subtree.
   * These nodes still belong to the parent tree.
   *
   * @param node The node to hide from the subtree.
   */
  void hideFromSubtree(const NodePtr &node);

  /**
   * @brief Hide invalid branches starting from a node in the subtree.
   *
   * This function hides invalid branches starting from the specified node in the subtree.
   * The branches still belong to the parent tree.
   *
   * @param node The starting node from which to hide invalid branches.
   */
  void hideInvalidBranches(const NodePtr& node);

  /**
   * @brief Purge a node from the subtree or its parent tree if it exists.
   *
   * This function purges the specified node from the subtree if it is present.
   * If the node is not part of the subtree, it is searched in the parent tree and removed from there.
   *
   * @param node The node to be purged.
   * @param removed_nodes The count of nodes removed during the operation.
   */
  void purgeThisNode(NodePtr& node, unsigned int& removed_nodes) override;

  /**
   * @brief Remove a node from the subtree and its parent tree.
   *
   * This function removes the specified node from the subtree and its parent tree.
   *
   * @param node The node to be removed.
   */
  void removeNode(const NodePtr& node) override;

  /**
   * @brief Add a node to the subtree and its parent tree.
   *
   * This function adds the specified node to the subtree and its parent tree.
   *
   * @param node The node to be added.
   * @param check_if_present If true, check if the node is already present in the trees before adding.
   */
  virtual void addNode(const NodePtr& node, const bool& check_if_present = true);

  /**
   * @brief Create a subtree instance.
   *
   * This function creates and returns a Subtree instance with the specified parent tree and root node.
   *
   * @param parent_tree The parent tree from which the subtree is extracted.
   * @param root The root node of the subtree.
   * @return A SubtreePtr to the created subtree instance.
   */
  static SubtreePtr createSubtree(const TreePtr& parent_tree, const NodePtr& root);

  /**
   * @brief Create a Subtree instance with the specified parent tree, root, and black list.
   *
   * This static function creates a SubtreePtr using the provided parent tree, root node, and black list.
   *
   * @param parent_tree The parent tree from which the subtree is extracted.
   * @param root The root node of the subtree.
   * @param black_list The list of nodes to be excluded from the subtree.
   * @return Returns a SubtreePtr created with the specified parameters.
   */
  static SubtreePtr createSubtree(const TreePtr& parent_tree, const NodePtr& root,
                                  const std::vector<NodePtr>& black_list);

  /**
   * @brief Create a Subtree instance with the specified parent tree, root, focus points, and cost.
   *
   * This static function creates a SubtreePtr using the provided parent tree, root node, focus points, and cost.
   *
   * @param parent_tree The parent tree from which the subtree is extracted.
   * @param root The root node of the subtree.
   * @param focus1 The first focus point for defining an ellipsoid.
   * @param focus2 The second focus point for defining an ellipsoid.
   * @param cost The cost associated with the ellipsoid.
   * @return Returns a SubtreePtr created with the specified parameters.
   */
  static SubtreePtr createSubtree(const TreePtr& parent_tree,
                                  const NodePtr& root,
                                  const Eigen::VectorXd& focus1,
                                  const Eigen::VectorXd& focus2,
                                  const double& cost);

  /**
   * @brief Create a Subtree instance with the specified parameters.
   *
   * This static function creates a SubtreePtr using the provided parent tree, root node, focus points, cost, black list, and node check.
   *
   * @param parent_tree The parent tree from which the subtree is extracted.
   * @param root The root node of the subtree.
   * @param focus1 The first focus point for defining an ellipsoid.
   * @param focus2 The second focus point for defining an ellipsoid.
   * @param cost The cost associated with the ellipsoid.
   * @param black_list A vector of nodes to be excluded from the subtree.
   * @param node_check A flag indicating whether to check nodes for validity during the subtree creation.
   * @return Returns a SubtreePtr created with the specified parameters.
   */
  static SubtreePtr createSubtree(const TreePtr& parent_tree,
                                  const NodePtr& root,
                                  const Eigen::VectorXd& focus1,
                                  const Eigen::VectorXd& focus2,
                                  const double& cost,
                                  const std::vector<NodePtr>& black_list,
                                  const bool node_check = false);

  /**
   * @brief Create a Subtree instance with the specified parameters.
   *
   * This static function creates a SubtreePtr using the provided parent tree, root node, goal, cost, black list, and node check.
   *
   * @param parent_tree The parent tree from which the subtree is extracted.
   * @param root The root node of the subtree.
   * @param goal The goal configuration for the subtree.
   * @param cost The cost associated with the subtree.
   * @param black_list A vector of nodes to be excluded from the subtree.
   * @param node_check A flag indicating whether to check nodes for validity during the subtree creation.
   * @return Returns a SubtreePtr created with the specified parameters.
   */
  static SubtreePtr createSubtree(const TreePtr& parent_tree,
                                  const NodePtr& root,
                                  const Eigen::VectorXd& goal,
                                  const double& cost,
                                  const std::vector<NodePtr>& black_list,
                                  const bool node_check = false);

};


}
