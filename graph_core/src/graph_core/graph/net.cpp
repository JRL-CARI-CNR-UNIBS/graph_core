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

std::multimap<double,std::vector<ConnectionPtr>> Net::getConnectionBetweenNodes(const NodePtr &start_node, const NodePtr& goal_node, bool& is_net_connected)
{
  NodePtr root = linked_tree_->getRoot();
  linked_tree_->changeRoot(start_node);

  std::multimap<double,std::vector<ConnectionPtr>> map = getConnectionToNode(goal_node);

  is_net_connected = false;

  if(!map.empty())
  {
    for(const std::pair<double,std::vector<ConnectionPtr>>& pair:map)
    {
      for(const ConnectionPtr& conn:pair.second)
      {
        if(conn->isNet())
        {
          is_net_connected = true;
          break;
        }
      }

      if(is_net_connected)
        break;
    }
  }

  linked_tree_->changeRoot(root);

  return map;
}

std::multimap<double,std::vector<ConnectionPtr>> Net::getConnectionToNode(const NodePtr &node)
{
  visited_nodes_.clear();
  visited_nodes_.push_back(node);

  return computeConnectionToNode(node);
}

std::multimap<double,std::vector<ConnectionPtr>> Net::computeConnectionToNode(const NodePtr& node)
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

      if(std::find(visited_nodes_.begin(), visited_nodes_.end(), parent) != visited_nodes_.end())
        continue;
      else
        visited_nodes_.push_back(parent);

      std::multimap<double,std::vector<ConnectionPtr>> map2parent = computeConnectionToNode(parent);

      visited_nodes_.pop_back();

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
