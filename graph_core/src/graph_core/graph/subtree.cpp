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
  Tree(root,parent_tree->getMaximumDistance(),
       parent_tree->getChecker(),parent_tree->getMetrics())
{
  populateTreeFromNode(root);
}

Subtree::Subtree(const TreePtr& parent_tree,
                 const NodePtr& root,
                 const std::vector<NodePtr>& black_list):
  parent_tree_(parent_tree),
  Tree(root,parent_tree->getMaximumDistance(),
       parent_tree->getChecker(),parent_tree->getMetrics())
{
  double cost = std::numeric_limits<double>::infinity();
  Eigen::VectorXd focus1,focus2;
  focus1 = root->getConfiguration();
  focus2 = root->getConfiguration();

  populateSubtree(root,focus1,focus2,cost,black_list);
}

Subtree::Subtree(const TreePtr& parent_tree,
                 const NodePtr& root,
                 const Eigen::VectorXd& focus1,
                 const Eigen::VectorXd& focus2,
                 const double& cost):
  parent_tree_(parent_tree),
  Tree(root,parent_tree->getMaximumDistance(),
       parent_tree->getChecker(),parent_tree->getMetrics())
{
  std::vector<NodePtr> black_list;
  populateSubtree(root,focus1,focus2,cost,black_list);
}

Subtree::Subtree(const TreePtr& parent_tree,
                 const NodePtr& root,
                 const Eigen::VectorXd& focus1,
                 const Eigen::VectorXd& focus2,
                 const double& cost,
                 const std::vector<NodePtr>& black_list,
                 const bool node_check):
  parent_tree_(parent_tree),
  Tree(root,parent_tree->getMaximumDistance(),
       parent_tree->getChecker(),parent_tree->getMetrics())

{
  populateSubtree(root,focus1,focus2,cost,black_list,node_check);
}

void Subtree::populateSubtree(const NodePtr& root, const Eigen::VectorXd& focus1, const Eigen::VectorXd& focus2, const double& cost, const std::vector<NodePtr> &black_list, const bool node_check)
{
  if(((root->getConfiguration()-focus1).norm()+(root->getConfiguration()-focus2).norm())<cost)
    populateTreeFromNode(root,focus1,focus2,cost,black_list, node_check);
  else
  {
    ROS_WARN("Root of subtree is not inside the ellipsoid!");
    ROS_INFO_STREAM("Root:\n "<<*root<<"\nFocus1: "<<focus1.transpose()<<"\nFocus2: "<<focus1.transpose()<<"\nCost: "<<cost);

    populateTreeFromNode(root,black_list, node_check);
  }
}

void Subtree::addNode(const NodePtr& node, const bool& check_if_present)
{
  Tree::addNode(node,check_if_present);
  parent_tree_->addNode(node,check_if_present);
}

void Subtree::removeNode(const std::vector<NodePtr>::iterator& it)
{
  // elimina ////////
  NodePtr node = *it;
  //Tree::removeNode(it);
  nodes_.erase(it);

  if(Subtree::isInTree(node))
    assert(0);


  parent_tree_->removeNode(*it);
  // //////

  //  Tree::removeNode(it);
  //  parent_tree_->removeNode(*it);
}

bool Subtree::purgeFromHere(NodePtr& node, const std::vector<NodePtr>& white_list, unsigned int& removed_nodes)
{
  unsigned int subtree_removed_nodes;
  return purgeFromHere(node,white_list,removed_nodes,subtree_removed_nodes);
}

bool Subtree::purgeFromHere(NodePtr& node, const std::vector<NodePtr>& white_list, unsigned int& removed_nodes, unsigned int& subtree_removed_nodes)
{
  // ////////////////elimina////////////////////////////////////////
  struct node_struct
  {
    NodePtr node;
    std::vector<ConnectionPtr> parent_connections;
    int child_size;
    int net_parent_size;
  };

  std::vector<node_struct> struct_nodes;
  std::vector<NodePtr> old_nodes;
  for(const NodePtr& n:nodes_) //elimina
  {
    old_nodes.push_back(n);
    node_struct ns;
    ns.node = n;
    ns.parent_connections = n->parent_connections_;
    ns.child_size = n->child_connections_.size();
    ns.net_parent_size = n->net_parent_connections_.size();

    struct_nodes.push_back(ns);

    if(n->parent_connections_.size() == 0 && n->child_connections_.size() == 0)
      assert(0);
  }
  // /////////////////////////////////////////////////////////////////

  if (std::find(white_list.begin(), white_list.end(), node) != white_list.end())
  {
    ROS_INFO_STREAM("Node in white list: "<<*node);
    return false;
  }
  assert(node);
  std::vector<NodePtr> successors;

  successors = node->getChildren();

  for(NodePtr& n : successors)
  {
    assert(n.get()!=node.get());

    if (!purgeFromHere(n,white_list,removed_nodes,subtree_removed_nodes))
    {
       // elimina
      ROS_INFO_STREAM("not purged node "<<*n);
      for(const NodePtr& nn:white_list)
        if(n==nn)
          ROS_INFO("it is a whitelist member");
      return false;
      // //
    }

    //    if (!purgeFromHere(n,white_list,removed_nodes,subtree_removed_nodes))
    //      return false;
  }

  assert(node);

  /*NB: if the subtree is defined inside an ellipsoid, the node could be part of the parent tree but not of the subtree
  (it is not in nodes_ of subtree but it is a successor of a node of the subtree). In this case, if the node is not in the
  subtree, it will be searched in the parent tree.
  */
  std::vector<NodePtr>::iterator it = std::find(nodes_.begin(), nodes_.end(), node);  //search in the subtree
  NodePtr pare = node->getParents().front(); // elimina
  node->disconnect();
  if(it < nodes_.end())
  {
    ROS_INFO_STREAM("nodo disconnesso dal subtree: "<<node->getConfiguration().transpose());
    ROS_INFO_STREAM("parent: "<<pare->getConfiguration().transpose());

    removeNode(it);
    removed_nodes++;
    subtree_removed_nodes++;
  }
  else
  {
    std::vector<NodePtr> parent_tree_nodes = parent_tree_->getNodes();
    it = std::find(parent_tree_nodes.begin(), parent_tree_nodes.end(), node);

    if(it < parent_tree_nodes.end())
    {
      ROS_INFO_STREAM("nodo disconnesso dal parent tree: "<<node->getConfiguration().transpose());
      ROS_INFO_STREAM("parent: "<<pare->getConfiguration().transpose());

      parent_tree_->removeNode(it);
      removed_nodes++;
    }
  }

  // ////////////////elimina////////////////////////////////////////
  bool rompi = false;
  for(const NodePtr& n:nodes_)
  {
    if(n->parent_connections_.size() == 0 && n->child_connections_.size() == 0)
    {
      for(const node_struct ns:struct_nodes)
      {
        if(n == ns.node)
        {
          bool inSTree = isInTree(n);
          bool inTree = parent_tree_->isInTree(n);
          ROS_INFO_STREAM("node: "<<*ns.node);
          ROS_INFO_STREAM("in sub tree: "<<inSTree);
          ROS_INFO_STREAM("in parent tree: "<<inTree);
          ROS_INFO_STREAM("old parent: "<<ns.parent_connections.front()->getParent()->getConfiguration().transpose()<<" old n children: "<<ns.child_size<<" old n net parent: "<<ns.net_parent_size);
        }
      }
      rompi = true;
    }
  }

  if(rompi)
  {
    ROS_INFO_STREAM("new subtree size: "<<nodes_.size());
    for(const NodePtr& n:nodes_)
      ROS_INFO_STREAM("sn: "<<n->getConfiguration().transpose());

    ROS_INFO_STREAM("old subtree size: "<<old_nodes.size());
    for(const NodePtr& n:old_nodes)
      ROS_INFO_STREAM("old sn: "<<n->getConfiguration().transpose());

    ROS_INFO_STREAM("removed nodes: "<<removed_nodes);
    ROS_INFO_STREAM("subtree removed nodes: "<<subtree_removed_nodes);

  }

  assert(!rompi);
  // /////////////////////////////////////////////////////////////////

  return true;
}


SubtreePtr Subtree::createSubtree(const TreePtr& parent_tree, const NodePtr& root)
{
  return std::make_shared<Subtree>(parent_tree,root);
}

SubtreePtr Subtree::createSubtree(const TreePtr& parent_tree, const NodePtr& root,
                                  const std::vector<NodePtr>& black_list)
{
  return std::make_shared<Subtree>(parent_tree,root, black_list);
}

SubtreePtr Subtree::createSubtree(const TreePtr& parent_tree, const NodePtr& root,
                                  const Eigen::VectorXd& focus1, const Eigen::VectorXd& focus2,
                                  const double& cost)
{
  return std::make_shared<Subtree>(parent_tree,root,focus1,focus2,cost);
}

SubtreePtr Subtree::createSubtree(const TreePtr& parent_tree, const NodePtr& root,
                                  const Eigen::VectorXd& focus1, const Eigen::VectorXd& focus2,
                                  const double& cost, const std::vector<NodePtr>& black_list,
                                  const bool node_check)
{
  return std::make_shared<Subtree>(parent_tree,root,focus1,focus2,cost,black_list,node_check);
}
}
