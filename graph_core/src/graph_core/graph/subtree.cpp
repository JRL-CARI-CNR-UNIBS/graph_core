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

#include <graph_core/graph/subtree.h>

namespace pathplan
{
Subtree::Subtree(const TreePtr& parent_tree,
                 const NodePtr& root):
  parent_tree_(parent_tree),
  Tree(root,parent_tree->getDirection(),parent_tree->getMaximumDistance(),
       parent_tree->getChecker(),parent_tree->getMetrics())
{
  populateTreeFromNode(root);
}

Subtree::Subtree(const TreePtr& parent_tree,
                 const NodePtr& root,
                 const std::vector<NodePtr>& white_list):
  parent_tree_(parent_tree),
  Tree(root,parent_tree->getDirection(),parent_tree->getMaximumDistance(),
       parent_tree->getChecker(),parent_tree->getMetrics())
{
  double cost = std::numeric_limits<double>::infinity();
  Eigen::VectorXd focus1,focus2;
  focus1 = root->getConfiguration();
  focus2 = root->getConfiguration();

  Subtree(parent_tree,root,focus1,focus2,cost,white_list);
}

Subtree::Subtree(const TreePtr& parent_tree,
                 const NodePtr& root,
                 const Eigen::VectorXd& focus1,
                 const Eigen::VectorXd& focus2,
                 const double& cost):
  parent_tree_(parent_tree),
  Tree(root,parent_tree->getDirection(),parent_tree->getMaximumDistance(),
       parent_tree->getChecker(),parent_tree->getMetrics())
{
  std::vector<NodePtr> white_list;
  Subtree(parent_tree,root,focus1,focus2,cost,white_list);
}

Subtree::Subtree(const TreePtr& parent_tree,
                 const NodePtr& root,
                 const Eigen::VectorXd& focus1,
                 const Eigen::VectorXd& focus2,
                 const double& cost,
                 const std::vector<NodePtr>& white_list):
  parent_tree_(parent_tree),
  Tree(root,parent_tree->getDirection(),parent_tree->getMaximumDistance(),
       parent_tree->getChecker(),parent_tree->getMetrics())

{
  if(((root->getConfiguration()-focus1).norm()+(root->getConfiguration()-focus2).norm())<cost)
    populateTreeFromNode(root,focus1,focus2,cost,white_list);
  else
  {
    ROS_WARN("Root of subtree is not inside the ellipsoid!");
    ROS_INFO_STREAM("Root:\n "<<*root<<"\nFocus1: "<<focus1.transpose()<<"\nFocus2: "<<focus1.transpose()<<"\nCost: "<<cost);

    populateTreeFromNode(root,white_list);
  }
}

void Subtree::addNode(const NodePtr& node, const bool& check_if_present)
{
  Tree::addNode(node,check_if_present);
  parent_tree_->addNode(node,check_if_present);
}

void Subtree::removeNode(const std::vector<NodePtr>::iterator& it)
{
  Tree::removeNode(it);
  parent_tree_->removeNode(*it);
}


SubtreePtr Subtree::createSubtree(const TreePtr& parent_tree, const NodePtr& root)
{
  return std::make_shared<Subtree>(parent_tree,root);
}

SubtreePtr Subtree::createSubtree(const TreePtr& parent_tree, const NodePtr& root,
                                  const std::vector<NodePtr>& white_list)
{
  return std::make_shared<Subtree>(parent_tree,root, white_list);
}

SubtreePtr Subtree::createSubtree(const TreePtr& parent_tree, const NodePtr& root,
                                  const Eigen::VectorXd& focus1, const Eigen::VectorXd& focus2,
                                  const double& cost)
{
  return std::make_shared<Subtree>(parent_tree,root,focus1,focus2,cost);
}

SubtreePtr Subtree::createSubtree(const TreePtr& parent_tree, const NodePtr& root,
                                  const Eigen::VectorXd& focus1, const Eigen::VectorXd& focus2,
                                  const double& cost, const std::vector<NodePtr>& white_list)
{
  return std::make_shared<Subtree>(parent_tree,root,focus1,focus2,cost,white_list);
}
}
