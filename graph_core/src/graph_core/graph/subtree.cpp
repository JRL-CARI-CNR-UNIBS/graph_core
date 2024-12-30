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

#include <graph_core/graph/subtree.h>

namespace graph
{
namespace core
{
Subtree::Subtree(const TreePtr& parent_tree, const NodePtr& root)
  : Tree(root, parent_tree->getMaximumDistance(), parent_tree->getChecker(), parent_tree->getMetrics(),
         parent_tree->getLogger(), parent_tree->getUseKdTree())
  , parent_tree_(parent_tree)
{
  populateTreeFromNode(root);
}

Subtree::Subtree(const TreePtr& parent_tree, const NodePtr& root, const std::vector<NodePtr>& black_list)
  : Tree(root, parent_tree->getMaximumDistance(), parent_tree->getChecker(), parent_tree->getMetrics(),
         parent_tree->getLogger(), parent_tree->getUseKdTree())
  , parent_tree_(parent_tree)
{
  double cost = std::numeric_limits<double>::infinity();
  Eigen::VectorXd focus1, focus2;
  focus1 = root->getConfiguration();
  focus2 = root->getConfiguration();

  populateSubtreeInsideEllipsoid(root, focus1, focus2, cost, black_list);
}

Subtree::Subtree(const TreePtr& parent_tree, const NodePtr& root, const Eigen::VectorXd& focus1,
                 const Eigen::VectorXd& focus2, const double& cost)
  : Tree(root, parent_tree->getMaximumDistance(), parent_tree->getChecker(), parent_tree->getMetrics(),
         parent_tree->getLogger(), parent_tree->getUseKdTree())
  , parent_tree_(parent_tree)
{
  std::vector<NodePtr> black_list;
  populateSubtreeInsideEllipsoid(root, focus1, focus2, cost, black_list);
}

Subtree::Subtree(const TreePtr& parent_tree, const NodePtr& root, const Eigen::VectorXd& focus1,
                 const Eigen::VectorXd& focus2, const double& cost, const std::vector<NodePtr>& black_list,
                 const bool node_check)
  : Tree(root, parent_tree->getMaximumDistance(), parent_tree->getChecker(), parent_tree->getMetrics(),
         parent_tree->getLogger(), parent_tree->getUseKdTree())
  , parent_tree_(parent_tree)
{
  populateSubtreeInsideEllipsoid(root, focus1, focus2, cost, black_list, node_check);
}

Subtree::Subtree(const TreePtr& parent_tree, const NodePtr& root, const Eigen::VectorXd& goal, const double& cost,
                 const std::vector<NodePtr>& black_list, const bool node_check)
  : Tree(root, parent_tree->getMaximumDistance(), parent_tree->getChecker(), parent_tree->getMetrics(),
         parent_tree->getLogger(), parent_tree->getUseKdTree())
  , parent_tree_(parent_tree)
{
  populateTreeFromNodeConsideringCost(root, goal, cost, black_list, node_check);
}

void Subtree::populateSubtreeInsideEllipsoid(const NodePtr& root, const Eigen::VectorXd& focus1,
                                             const Eigen::VectorXd& focus2, const double& cost,
                                             const std::vector<NodePtr>& black_list, const bool node_check)
{
  if ((metrics_->utopia(root->getConfiguration(), focus1) + metrics_->utopia(root->getConfiguration(), focus2)) < cost)
    populateTreeFromNode(root, focus1, focus2, cost, black_list, node_check);
  else
  {
    CNR_WARN(logger_, "Root of subtree is not inside the ellipsoid!");
    CNR_INFO(logger_, "Root:\n " << *root << "\nFocus1: " << focus1.transpose() << "\nFocus2: " << focus1.transpose()
                                 << "\nCost: " << cost);

    populateTreeFromNode(root, black_list, node_check);
  }
}

void Subtree::addNode(const NodePtr& node, const bool& check_if_present)
{
  Tree::addNode(node, check_if_present);
  parent_tree_->addNode(node, check_if_present);
}

void Subtree::hideFromSubtree(const NodePtr& node)
{
  assert(node);
  if (nodes_->findNode(node))
  {
    std::vector<NodePtr> successors = node->getChildren();
    for (NodePtr& n : successors)
    {
      assert(n.get() != node.get());
      hideFromSubtree(n);
    }

    if (node != root_)
      nodes_->deleteNode(node, false);
  }
}

void Subtree::hideInvalidBranches(const NodePtr& node)
{
  assert(node);
  if (nodes_->findNode(node))
  {
    for (ConnectionPtr& c : node->getChildConnections())
    {
      assert(c->getChild().get() != node.get());
      if (c->getCost() == std::numeric_limits<double>::infinity())
        hideFromSubtree(c->getChild());
      else
        hideInvalidBranches(c->getChild());
    }

    if (node != root_)
      nodes_->deleteNode(node, false);
  }
}

void Subtree::removeNode(const NodePtr& node)
{
  nodes_->deleteNode(node, true);
  parent_tree_->removeNode(node);
}

void Subtree::purgeThisNode(NodePtr& node, unsigned int& removed_nodes)
{
  /*NB: if the subtree is defined inside an ellipsoid, the node could be part of
  the parent tree but not of the subtree (it is not in nodes_ of subtree but it
  is a successor of a node of the subtree). In this case, if the node is not in
  the subtree, it will be searched in the parent tree.
  */

  assert(node);
  node->disconnect();

  if (nodes_->findNode(node))
  {
    Subtree::removeNode(node);
    removed_nodes++;
  }
  else
  {
    if (parent_tree_->isInTree(node))
    {
      parent_tree_->removeNode(node);
      removed_nodes++;
    }
  }
}

SubtreePtr Subtree::createSubtree(const TreePtr& parent_tree, const NodePtr& root)
{
  return std::make_shared<Subtree>(parent_tree, root);
}

SubtreePtr Subtree::createSubtree(const TreePtr& parent_tree, const NodePtr& root,
                                  const std::vector<NodePtr>& black_list)
{
  return std::make_shared<Subtree>(parent_tree, root, black_list);
}

SubtreePtr Subtree::createSubtree(const TreePtr& parent_tree, const NodePtr& root, const Eigen::VectorXd& focus1,
                                  const Eigen::VectorXd& focus2, const double& cost)
{
  return std::make_shared<Subtree>(parent_tree, root, focus1, focus2, cost);
}

SubtreePtr Subtree::createSubtree(const TreePtr& parent_tree, const NodePtr& root, const Eigen::VectorXd& focus1,
                                  const Eigen::VectorXd& focus2, const double& cost,
                                  const std::vector<NodePtr>& black_list, const bool node_check)
{
  return std::make_shared<Subtree>(parent_tree, root, focus1, focus2, cost, black_list, node_check);
}

SubtreePtr Subtree::createSubtree(const TreePtr& parent_tree, const NodePtr& root, const Eigen::VectorXd& goal,
                                  const double& cost, const std::vector<NodePtr>& black_list, const bool node_check)
{
  return std::make_shared<Subtree>(parent_tree, root, goal, cost, black_list, node_check);
}

}  // end namespace core
}  // end namespace graph
