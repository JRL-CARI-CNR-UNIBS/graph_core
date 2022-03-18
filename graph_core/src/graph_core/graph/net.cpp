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

#include <graph_core/graph/net.h>

namespace pathplan
{

bool Net::purgeSuccessors(NodePtr& node, const std::vector<NodePtr>& white_list, unsigned int& removed_nodes)
{
  if (std::find(white_list.begin(), white_list.end(), node) != white_list.end())
  {
    ROS_INFO_STREAM("Node in white list: "<<*node);
    return false;
  }
  assert(node);

  bool purged;
  bool disconnect = true;
  do
  {
    purged = false;

    std::vector<NodePtr> successors = node->getChildren();
    successors.insert(successors.end(),node->getNetChildrenConst().begin(),node->getNetChildrenConst().end());

    for (NodePtr& n : successors)
    {
      assert(n.get()!=node.get());

      if((n->net_parent_connections_.size())>0 || n == linked_tree_->getRoot())
        continue;
      else
      {
        if(not purgeSuccessors(n,white_list,removed_nodes))
          disconnect = false;
        else
          purged = true;
      }
    }
  }while(purged);

  if(disconnect)
  {
    ConnectionPtr conn2convert;
    std::vector<NodePtr> children = node->getChildren();
    for(NodePtr& successor2save: children)
    {
      assert(successor2save->net_parent_connections_.size()>0);
      assert(successor2save->parent_connections_.size() == 1);
      assert(successor2save->parent_connections_.front()->getParent() == node);

      conn2convert = successor2save->net_parent_connections_.front(); //the successor2save must be still part of the tree, so you convert one of its net parent connection into a parent connection
      assert(conn2convert->isNet());
      conn2convert->convertToConnection();

      assert(successor2save->parent_connections_.size() == 1);
    }

    linked_tree_->purgeThisNode(node,removed_nodes);
  }

  return disconnect;
}

bool Net::purgeFromHere(ConnectionPtr& conn2node, const std::vector<NodePtr>& white_list, unsigned int& removed_nodes)
{
  NodePtr node = conn2node->getChild();

  if((node->net_parent_connections_.size())>0 || node == linked_tree_->getRoot())
  {
    if(not conn2node->isNet())
    {
      ConnectionPtr conn2convert = node->net_parent_connections_.front();
      assert(conn2convert->isNet());
      conn2convert->convertToConnection();
      removed_nodes = 0;
    }

    conn2node->remove();
    return false;
  }
  else
    return purgeSuccessors(node,white_list,removed_nodes);
}

std::multimap<double,std::vector<ConnectionPtr>> Net::getNetConnectionBetweenNodes(const NodePtr& start_node, const NodePtr& goal_node)
{
  NodePtr net_parent;
  std::vector<NodePtr> visited_nodes;
  std::multimap<double,std::vector<ConnectionPtr>> map, parent_map;

  for(const ConnectionPtr& net_parent_conn: goal_node->net_parent_connections_)
  {
    net_parent = net_parent_conn->getParent();

    visited_nodes.clear();
    visited_nodes.push_back(goal_node);
    visited_nodes.push_back(net_parent);

    parent_map = computeConnectionFromNodeToNode(start_node,net_parent,visited_nodes);

    if(not parent_map.empty())
      for(std::pair<double,std::vector<ConnectionPtr>> pair:parent_map)
      {
        pair.first = pair.first + net_parent_conn->getCost();
        pair.second.push_back(net_parent_conn);
        map.insert(pair);
      }
  }

  return map;
}

std::multimap<double,std::vector<ConnectionPtr>> Net::getConnectionBetweenNodes(const NodePtr &start_node, const NodePtr& goal_node)
{
  std::vector<NodePtr> visited_nodes;
  visited_nodes.push_back(goal_node);

  return computeConnectionFromNodeToNode(start_node,goal_node,visited_nodes);
}

std::multimap<double,std::vector<ConnectionPtr>> Net::getConnectionToNode(const NodePtr &node)
{
  std::vector<NodePtr> visited_nodes;
  visited_nodes.push_back(node);

  return computeConnectionFromNodeToNode(linked_tree_->getRoot(),node,visited_nodes);
}

std::multimap<double,std::vector<ConnectionPtr>> Net::computeConnectionFromNodeToNode(const NodePtr& start_node, const NodePtr& goal_node, std::vector<NodePtr> &visited_nodes)
{
  std::multimap<double,std::vector<ConnectionPtr>> map;
  std::pair<double,std::vector<ConnectionPtr>> pair;
  NodePtr parent;

  if(goal_node == linked_tree_->getRoot() || goal_node == start_node)
    return map;
  else
  {
    if(goal_node->parent_connections_.size() != 1)
    {
      ROS_ERROR("a node of a tree should have only a parent");
      ROS_ERROR_STREAM("node \n" << *goal_node);

      ROS_INFO_STREAM("current root "<<linked_tree_->getRoot()->getConfiguration().transpose()<< " "<<linked_tree_->getRoot());
      ROS_INFO_STREAM("goal node "<<goal_node->getConfiguration().transpose()<<" "<<goal_node);
      ROS_INFO_STREAM("start node "<<start_node->getConfiguration().transpose()<<" "<<start_node);

      assert(0);
    }

    std::vector<ConnectionPtr> all_parent_connections = goal_node->parent_connections_;
    all_parent_connections.insert(all_parent_connections.end(),goal_node->net_parent_connections_.begin(),goal_node->net_parent_connections_.end());

    for(const ConnectionPtr conn_parent_goal:all_parent_connections)
    {
      parent = conn_parent_goal->getParent();

      if(parent == start_node)
      {
        std::vector<ConnectionPtr> from_start_to_node;
        from_start_to_node.push_back(conn_parent_goal);

        pair.first = conn_parent_goal->getCost();
        pair.second = from_start_to_node;

        map.insert(pair);
      }
      else
      {
        if(std::find(visited_nodes.begin(), visited_nodes.end(), parent) != visited_nodes.end())
          continue;
        else
          visited_nodes.push_back(parent);

        std::multimap<double,std::vector<ConnectionPtr>> map2parent = computeConnectionFromNodeToNode(start_node,parent,visited_nodes);

        visited_nodes.pop_back();

        if(not map2parent.empty())
        {
          for(const std::pair<double,std::vector<ConnectionPtr>>& parent_pair:map2parent)
          {
            std::vector<ConnectionPtr> from_start_to_parent = parent_pair.second;

            if(from_start_to_parent.front()->getParent() != start_node)
              continue;
            else
            {
              from_start_to_parent.push_back(conn_parent_goal);

              pair.first = parent_pair.first + conn_parent_goal->getCost();
              pair.second = from_start_to_parent;

              map.insert(pair);
            }
          }
        }
      }
    }
    return map;
  }
}

}  // end namespace pathplan
