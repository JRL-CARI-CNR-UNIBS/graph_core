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

namespace pathplan {

Tree::Tree(const NodePtr& root,
           const Direction &direction,
           const double &max_distance,
           const CollisionCheckerPtr &checker,
           const MetricsPtr &metrics):
  checker_(checker),
  metrics_(metrics),
  max_distance_(max_distance),
  direction_(direction),
  root_(root)
{
  nodes_.push_back(root);
}

NodePtr Tree::findClosestNode(const Eigen::VectorXd &configuration)
{
  NodePtr closest_node;
  double min_square_distance=std::numeric_limits<double>::infinity();
  for (const NodePtr& n: nodes_)
  {
    double squared_dist=(n->getConfiguration()-configuration).squaredNorm();
    if (squared_dist<min_square_distance)
    {
      min_square_distance=squared_dist;
      closest_node=n;
    }
  }
  return closest_node;
}

bool Tree::tryExtend(const Eigen::VectorXd &configuration,
                     Eigen::VectorXd &next_configuration,
                     NodePtr &closest_node)
{
  closest_node=findClosestNode(configuration);
  assert(closest_node);

  double distance=(closest_node->getConfiguration()-configuration).norm();
  if (distance<tolerance_)
  {
    next_configuration=configuration;
    return true;
  }
  else if (distance<max_distance_)
  {
    next_configuration=configuration;
  }
  else
  {
    next_configuration=closest_node->getConfiguration()+(configuration-closest_node->getConfiguration())/distance*max_distance_;
  }
  if (checker_->checkPath(closest_node->getConfiguration(),next_configuration))
    return true;

  return false;
}

bool Tree::extend(const Eigen::VectorXd &configuration, NodePtr &new_node)
{
  NodePtr closest_node;
  Eigen::VectorXd next_configuration;
  if (!tryExtend(configuration,
                 next_configuration,
                 closest_node))
  {
    return false;
  }

  new_node=std::make_shared<Node>(next_configuration);
  if (direction_==Forward)
  {
    double cost=metrics_->cost(closest_node,new_node);
    ConnectionPtr conn=std::make_shared<Connection>(closest_node,new_node);
    conn->add();
    conn->setCost(cost);

  }
  else
  {
    double cost=metrics_->cost(new_node,closest_node);
    ConnectionPtr conn=std::make_shared<Connection>(new_node,closest_node);
    conn->add();
    conn->setCost(cost);
  }
  nodes_.push_back(new_node);
  return true;
}

bool Tree::extendToNode(const NodePtr& node,
                        NodePtr& new_node)
{
  NodePtr closest_node;
  Eigen::VectorXd next_configuration;
  if (!tryExtend(node->getConfiguration(),
                 next_configuration,
                 closest_node))
  {
    return false;
  }

  bool attached=false;
  if ((next_configuration-node->getConfiguration()).norm()<tolerance_)
  {
    attached=true;
    new_node=node;
  }
  else
  {
    new_node=std::make_shared<Node>(next_configuration);
    nodes_.push_back(new_node);
  }

  if (direction_==Forward)
  {
    double cost=metrics_->cost(closest_node,new_node);
    ConnectionPtr conn=std::make_shared<Connection>(closest_node,new_node);
    conn->add();
    conn->setCost(cost);

  }
  else
  {
    double cost=metrics_->cost(new_node,closest_node);
    ConnectionPtr conn=std::make_shared<Connection>(new_node,closest_node);
    conn->add();
    conn->setCost(cost);
  }
  return true;
}

bool Tree::connect(const Eigen::VectorXd &configuration, NodePtr &new_node)
{
  bool success=true;
  while (success)
  {
    NodePtr tmp_node;
    success=extend(configuration,tmp_node);
    if (success)
    {
      new_node=tmp_node;
      if ((new_node->getConfiguration()-configuration).norm()<tolerance_)
        return true;
    }
  }
  return false;
}

bool Tree::connectToNode(const NodePtr &node, NodePtr &new_node)
{
  bool success=true;
  while (success)
  {
    NodePtr tmp_node;
    success=extendToNode(node,tmp_node);
    if (success)
    {
      new_node=tmp_node;
      if ((new_node->getConfiguration()-node->getConfiguration()).norm()<tolerance_)
        return true;
    }
  }
  return false;
}

bool Tree::rewire(const Eigen::VectorXd &configuration, double r_rewire)
{
  if (direction_==Backward)
  {
    ROS_ERROR("rewiring is available only on forward tree");
    return false;
  }
  NodePtr new_node;
  if (!extend(configuration,new_node))
  {
    return false;
  }
  std::vector<NodePtr> near_nodes=near(new_node,r_rewire);
  NodePtr nearest_node=new_node->getParents().at(0);
  double cost_to_new=costToNode(new_node);

  bool improved=false;

  ROS_DEBUG("try to find a better parent between %zu nodes",near_nodes.size());
  for (const NodePtr& node: near_nodes)
  {
    if (node==nearest_node)
      continue;
    if (node==new_node)
      continue;

    double cost_to_near=costToNode(node);

    if (cost_to_near>=cost_to_new)
      continue;

    double cost_near_to_new=metrics_->cost(node,new_node);

    if ((cost_to_near+cost_near_to_new)>=cost_to_new)
      continue;

    if (!checker_->checkPath(node->getConfiguration(),new_node->getConfiguration()))
      continue;


    new_node->parent_connections_.at(0)->remove();

    ConnectionPtr conn=std::make_shared<Connection>(node,new_node);
    conn->setCost(cost_near_to_new);
    conn->add();
    nearest_node=node;
    cost_to_new=cost_to_near+cost_near_to_new;
    improved=true;
  }

  ROS_DEBUG("try to find a better child between %zu nodes",near_nodes.size());
  for (NodePtr& n: near_nodes)
  {
    if (n==new_node)
      continue;

    double cost_to_near=costToNode(n);
    if (cost_to_new>=cost_to_near)
      continue;

    double cost_new_to_near=metrics_->cost(new_node->getConfiguration(),n->getConfiguration());
    if ((cost_to_new+cost_new_to_near)>=cost_to_near)
      continue;

    if (!checker_->checkPath(new_node->getConfiguration(),n->getConfiguration()))
      continue;

    n->parent_connections_.at(0)->remove();
    ConnectionPtr conn=std::make_shared<Connection>(new_node,n);
    conn->setCost(cost_new_to_near);
    conn->add();

  }

  return improved;
}

std::vector<NodePtr> Tree::near(const NodePtr &node, const double &r_rewire)
{
  std::vector<NodePtr> nodes;
  for (const NodePtr& n: nodes_)
  {
    double dist=(n->getConfiguration()-node->getConfiguration()).norm();
    if (dist<r_rewire)
    {
      nodes.push_back(n);
    }
  }
  return nodes;
}

double Tree::costToNode( NodePtr node)
{
  double cost=0;
  if (direction_==Forward)
  {
    while (node!=root_)
    {
      if (node->parent_connections_.size()!=1)
      {
        ROS_ERROR("a node of forward-direction tree should have exactly a parent");
        ROS_FATAL_STREAM("node=\n"<<*node);
        assert(0);
      }

      if (node->parent_connections_.at(0)->getParent()==node)
      {
        ROS_FATAL_STREAM("node=\n"<<*node);
        ROS_FATAL_STREAM("to parent\n"<<*(node->parent_connections_.at(0)));
        ROS_FATAL("connection between the same node");
        assert(0);
      }
      cost+=node->parent_connections_.at(0)->getCost();
      node=node->parent_connections_.at(0)->getParent();

    }
  }
  else
  {
    while (node!=root_)
    {
      if (node->child_connections_.size()!=1)
      {
        ROS_ERROR("a node of backward-direction tree should have only a child");
        assert(0);
      }
      cost+=node->child_connections_.at(0)->getCost();
      node=node->child_connections_.at(0)->getChild();
    }
  }
  return cost;
}

std::vector<ConnectionPtr> Tree::getConnectionToNode(NodePtr node)
{
  std::vector<ConnectionPtr> connections;
  if (direction_==Forward)
  {
    while (node!=root_)
    {
      if (node->parent_connections_.size()!=1)
      {
        ROS_ERROR("a node of forward-direction tree should have only a parent");
        ROS_ERROR_STREAM("node \n"<< *node);
        assert(0);
      }
      connections.push_back(node->parent_connections_.at(0));
      node=node->parent_connections_.at(0)->getParent();

    }
    std::reverse(connections.begin(),connections.end());
  }
  else
  {
    while (node!=root_)
    {
      if (node->child_connections_.size()!=1)
      {
        ROS_ERROR("a node of backward-direction tree should have only a child");
        assert(0);
      }

      connections.push_back(node->child_connections_.at(0));
      node=node->child_connections_.at(0)->getChild();
    }
  }
  return connections;
}

void Tree::addNode(const NodePtr& node, const bool& check_if_present)
{
  if (!check_if_present || !isInTree(node))
    nodes_.push_back(node);
}
bool Tree::keepOnlyThisBranch(const std::vector<ConnectionPtr>& connections)
{
  if (connections.size()==0)
    return false;

  std::vector<NodePtr> branch_nodes;
  branch_nodes.push_back(connections.at(0)->getParent());
  for (const ConnectionPtr& conn: connections)
    branch_nodes.push_back(conn->getChild());

  for (NodePtr& n: nodes_)
  {
    std::vector<NodePtr>::iterator it=std::find(branch_nodes.begin(),branch_nodes.end(),n);
    if (it==branch_nodes.end())
    {
      n->disconnect();
      n.reset();
    }
  }
  nodes_=branch_nodes;

  return true;
}

bool Tree::addBranch(const std::vector<ConnectionPtr> &connections)
{
  if (connections.size()==0)
    return false;

  NodePtr start_node=connections.at(0)->getParent();
  if (!isInTree(start_node))
  {
    ROS_ERROR("start node of the branch is not part of the tree");
    return false;
  }

  std::vector<NodePtr> branch_nodes;
  for (const ConnectionPtr& conn: connections)
    branch_nodes.push_back(conn->getChild());

  for (NodePtr& n: branch_nodes)
  {
    std::vector<NodePtr>::iterator it=std::find(nodes_.begin(),nodes_.end(),n);
    if (it==nodes_.end())
      nodes_.push_back(n);
  }
  return true;

}

bool Tree::isInTree(const NodePtr &node)
{
  std::vector<NodePtr>::iterator it;
  return isInTree(node,it);
}

bool Tree::isInTree(const NodePtr &node,std::vector<NodePtr>::iterator& it)
{
  it=std::find(nodes_.begin(),nodes_.end(),node);
  return it!=nodes_.end();
}


void Tree::purgeNodes(const SamplerPtr& sampler, const std::vector<NodePtr>& white_list, const bool check_bounds)
{
  if (nodes_.size()<maximum_nodes_)
    return;
  unsigned int nodes_to_remove=nodes_.size()-maximum_nodes_;

  unsigned int removed_nodes=0;
  unsigned int idx=0;
  while (idx<nodes_.size())
  {
    if (std::find(white_list.begin(),white_list.end(),nodes_.at(idx))!=white_list.end())
    {
      idx++;
      continue;
    }
    if (check_bounds && !sampler->inBounds(nodes_.at(idx)->getConfiguration()))
    {
      purgeFromHere(nodes_.at(idx),white_list,removed_nodes);
      continue;
    }


    if (nodes_to_remove<removed_nodes)
      break;
    if ((direction_==Forward  && nodes_.at(idx)->child_connections_.size()==0 ) ||
        (direction_==Backward && nodes_.at(idx)->parent_connections_.size()==0))
    {
      removed_nodes++;
      nodes_.at(idx)->disconnect();
      nodes_.erase(nodes_.begin()+idx);
      continue;
    }

    idx++;

  }



}


bool Tree::purgeFromHere(NodePtr& node, const std::vector<NodePtr>& white_list, unsigned int& removed_nodes)
{
  if (std::find(white_list.begin(),white_list.end(),node)!=white_list.end())
    return false;


  std::vector<NodePtr> successors;

  if (direction_==Forward)
    successors=node->getChildren();
  else
    successors=node->getParents();

  do
  {
    for (NodePtr& n: successors)
    {

      if (!purgeFromHere(n,white_list,removed_nodes))
        return false;

    }

    if (direction_==Forward)
      successors=node->getChildren();
    else
      successors=node->getParents();

  }
  while (successors.size()>0);

  std::vector<NodePtr>::iterator it=std::find(nodes_.begin(),nodes_.end(),node);
  node->disconnect();
  if(it<nodes_.end())
  {
    nodes_.erase(it);
    removed_nodes++;
  }
  return true;
}

}
