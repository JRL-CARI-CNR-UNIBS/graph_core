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

std::multimap<double,std::vector<ConnectionPtr>> Net::getConnectionBetweenNodes(const NodePtr &start_node, const NodePtr& goal_node, bool& is_net_connected, const bool &get_only_net)
{
  std::vector<NodePtr> goal_vector;
  goal_vector.push_back(goal_node);

  std::vector<bool> is_net_connected_vector;
  is_net_connected_vector.push_back(is_net_connected);

  if(!linked_tree_->isInTree(start_node))
    assert(0);

  return getConnectionBetweenNodes(start_node,goal_vector,is_net_connected_vector,get_only_net);
}

std::multimap<double,std::vector<ConnectionPtr>> Net::getConnectionBetweenNodes(const NodePtr &start_node, const std::vector<NodePtr>& goal_vector, std::vector<bool>& is_net_connected_vector, const bool &get_only_net)
{
  TreePtr linked_tree = linked_tree_;
  ROS_INFO("Creating subtree");
  SubtreePtr subtree = std::make_shared<Subtree>(linked_tree_,start_node);
  ROS_INFO("Subtree created");

  linked_tree_ = subtree; //temporary substitute the linked tree with the subtree to get the connections to the goals

  std::multimap<double,std::vector<ConnectionPtr>> only_net_map, map;

  for(unsigned int i=0;i<goal_vector.size();i++)
  {
    ROS_INFO_STREAM("Goal number "<<i);

    is_net_connected_vector.at(i) = false;
    map = getConnectionToNode(goal_vector.at(i));

    ROS_INFO("Map computed");

    if(!map.empty())
    {
      ROS_INFO("Map NOT empty");

      for(const std::pair<double,std::vector<ConnectionPtr>>& pair:map)
      {
        ROS_INFO("Map pair");

        for(const ConnectionPtr& conn:pair.second)
        {
          if(conn->isNet())
          {
            if(get_only_net)
            {
              ROS_INFO("Inserting net pair");
              only_net_map.insert(pair);
            }

            is_net_connected_vector.at(i) = true;
            break;
          }
        }

        if(is_net_connected_vector.at(i) && !get_only_net)
        {
          ROS_INFO("Break");
          break;
        }
      }
    }
    else
      ROS_INFO("Map empty");
  }

  linked_tree_ = linked_tree;

  if(get_only_net)
    return only_net_map;
  else
    return map;
}

std::multimap<double,std::vector<ConnectionPtr>> Net::getConnectionToNode(const NodePtr &node)
{
  std::vector<NodePtr> visited_nodes;
  visited_nodes.push_back(node);

  return computeConnectionToNode(node,visited_nodes);
}

std::multimap<double,std::vector<ConnectionPtr>> Net::computeConnectionToNode(const NodePtr& node, std::vector<NodePtr> &visited_nodes)
{
  std::multimap<double,std::vector<ConnectionPtr>> map;
  std::pair<double,std::vector<ConnectionPtr>> pair;
  NodePtr parent;

  if(node == linked_tree_->getRoot())
    return map;
  else
  {
    if (node->parent_connections_.size() != 1)
    {
      ROS_ERROR("a node of a tree should have only a parent");
      ROS_ERROR_STREAM("node \n" << *node);

      ROS_INFO_STREAM("current root "<<linked_tree_->getRoot());
      ROS_INFO_STREAM("node "<<node);

      assert(0);
    }

    std::vector<ConnectionPtr> all_parent_connections = node->parent_connections_;
    all_parent_connections.insert(all_parent_connections.end(),node->net_parent_connections_.begin(),node->net_parent_connections_.end());

    for(const ConnectionPtr conn_parent_node:all_parent_connections)
    {
      parent = conn_parent_node->getParent();
      if(disp_)
      {
        disp_->displayNode(parent,123456);
        disp_->nextButton();
      }

      if(std::find(visited_nodes.begin(), visited_nodes.end(), parent) != visited_nodes.end())
        continue;
      else
        visited_nodes.push_back(parent);

      std::multimap<double,std::vector<ConnectionPtr>> map2parent = computeConnectionToNode(parent,visited_nodes);

      visited_nodes.pop_back();

      if(!map2parent.empty())
      {
        for(const std::pair<double,std::vector<ConnectionPtr>>& parent_pair:map2parent)
        {
          std::vector<ConnectionPtr> from_start_to_node = parent_pair.second;

          if(from_start_to_node.front()->getParent() != linked_tree_->getRoot())
            continue;
          else
          {
            from_start_to_node.push_back(conn_parent_node);

            pair.first = parent_pair.first + conn_parent_node->getCost();
            pair.second = from_start_to_node;

            map.insert(pair);
          }
        }
      }
      else
      {
        std::vector<ConnectionPtr> from_start_to_node;
        from_start_to_node.push_back(conn_parent_node);

        pair.first = conn_parent_node->getCost();
        pair.second = from_start_to_node;

        map.insert(pair);
      }
    }
    return map;
  }
}

}  // end namespace pathplan
