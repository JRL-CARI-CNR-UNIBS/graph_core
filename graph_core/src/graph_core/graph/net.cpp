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

      if((n->getNetParentConnectionsSize())>0 || n == linked_tree_->getRoot())
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
      assert(successor2save->getNetParentConnectionsSize()>0);
      assert(successor2save->getParentConnectionsSize() == 1);
      assert(successor2save->parentConnection(0)->getParent() == node);

      conn2convert = successor2save->netParentConnection(0); //the successor2save must be still part of the tree, so you convert one of its net parent connection into a parent connection
      assert(conn2convert->isNet());
      conn2convert->convertToConnection();

      assert(successor2save->getParentConnectionsSize() == 1);
    }

    linked_tree_->purgeThisNode(node,removed_nodes);
  }

  return disconnect;
}

bool Net::purgeFromHere(ConnectionPtr& conn2node, const std::vector<NodePtr>& white_list, unsigned int& removed_nodes)
{
  NodePtr node = conn2node->getChild();

  if((node->getNetParentConnectionsSize())>0 || node == linked_tree_->getRoot())
  {
    if(not conn2node->isNet())
    {
      ConnectionPtr conn2convert = node->netParentConnection(0);
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

std::multimap<double,std::vector<ConnectionPtr>> Net::getNetConnectionBetweenNodes(const NodePtr& start_node, const NodePtr& goal_node, const std::vector<NodePtr>& black_list)
{
  NodePtr net_parent;
  ConnectionPtr net_parent_conn;
  std::vector<NodePtr> visited_nodes;
  std::multimap<double,std::vector<ConnectionPtr>> map, parent_map;

  for(unsigned int i=0;i<goal_node->getNetParentConnectionsSize();i++)
  {
    net_parent_conn = goal_node->netParentConnection(i);
    net_parent = net_parent_conn->getParent();

    visited_nodes.clear();
    visited_nodes.push_back(goal_node);
    visited_nodes.push_back(net_parent);

    parent_map = computeConnectionFromNodeToNode(start_node,net_parent,black_list,visited_nodes);

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

std::multimap<double,std::vector<ConnectionPtr>> Net::getConnectionBetweenNodes(const NodePtr &start_node, const NodePtr& goal_node, const std::vector<NodePtr>& black_list)
{
  std::vector<NodePtr> visited_nodes;
  visited_nodes.push_back(goal_node);

  return computeConnectionFromNodeToNode(start_node,goal_node,black_list,visited_nodes);
}

std::multimap<double,std::vector<ConnectionPtr>> Net::getConnectionToNode(const NodePtr &node, const double& cost2beat, const std::vector<NodePtr>& black_list)
{
  std::vector<NodePtr> visited_nodes;
  visited_nodes.push_back(node);

  double cost2here = 0.0;

  return computeConnectionFromNodeToNode(linked_tree_->getRoot(),node,cost2here,cost2beat,black_list,visited_nodes);
}

std::multimap<double,std::vector<ConnectionPtr>> Net::getConnectionToNode(const NodePtr &node, const std::vector<NodePtr>& black_list)
{
  double cost2beat = std::numeric_limits<double>::infinity();
  return computeConnectionFromNodeToNode(linked_tree_->getRoot(),node,cost2beat,black_list);
}

std::multimap<double,std::vector<ConnectionPtr>> Net::computeConnectionFromNodeToNode(const NodePtr& start_node, const NodePtr& goal_node, std::vector<NodePtr> &visited_nodes)
{
  std::vector<NodePtr> black_list;
  return computeConnectionFromNodeToNode(start_node,goal_node,black_list,visited_nodes);
}

std::multimap<double,std::vector<ConnectionPtr>> Net::computeConnectionFromNodeToNode(const NodePtr& start_node, const NodePtr& goal_node, const std::vector<NodePtr> &black_list, std::vector<NodePtr> &visited_nodes)
{
  double cost2here = 0.0;
  double cost2beat = std::numeric_limits<double>::infinity();
  return computeConnectionFromNodeToNode(start_node, goal_node, cost2here, cost2beat, black_list, visited_nodes);
}

std::multimap<double,std::vector<ConnectionPtr>> Net::computeConnectionFromNodeToNode(const NodePtr& start_node, const NodePtr& goal_node, const double& cost2here, const double& cost2beat, const std::vector<NodePtr> &black_list, std::vector<NodePtr> &visited_nodes)
{
  std::vector<ConnectionPtr> connections2here;
  return computeConnectionFromNodeToNode(start_node,goal_node,cost2here,cost2beat,black_list,visited_nodes,connections2here);
}

std::multimap<double,std::vector<ConnectionPtr>> Net::computeConnectionFromNodeToNode(const NodePtr& start_node, const NodePtr& goal_node, const double& cost2here, const double& cost2beat, const std::vector<NodePtr> &black_list, std::vector<NodePtr> &visited_nodes, std::vector<ConnectionPtr>& connections2here)
{
  //Depth-first search

  NodePtr parent;
  std::multimap<double,std::vector<ConnectionPtr>> map;

  if(goal_node == linked_tree_->getRoot() || goal_node == start_node)
    return map;
  else
  {
    assert([&]() ->bool{
             if(goal_node->getParentConnectionsSize() != 1)
             {
               ROS_ERROR("a node of a tree should have only a parent");
               ROS_ERROR_STREAM("goal node \n" <<*goal_node);

               ROS_INFO_STREAM("current root "<<linked_tree_->getRoot()->getConfiguration().transpose()<< " "<<linked_tree_->getRoot());
               ROS_INFO_STREAM("goal node "<<goal_node->getConfiguration().transpose()<<" "<<goal_node);
               ROS_INFO_STREAM("start node "<<start_node->getConfiguration().transpose()<<" "<<start_node);

               if(goal_node->getNetChildConnectionsSize()>0);
               ROS_INFO_STREAM("the child "<<*goal_node->netChildConnection(0)->getChild()<<" "<<goal_node->netChildConnection(0)->getChild());

               int count = 0;
               for(const NodePtr& n:linked_tree_->getNodes())
               {
                 if(n->getConfiguration() == goal_node->getConfiguration())
                 {
                   count++;
                   ROS_INFO_STREAM(*n<<"\n"<<n<<"\n count "<<count);
                 }
               }
               ROS_INFO_STREAM("EQUAL NODES "<<count);

               ROS_INFO_STREAM("in tree: "<<linked_tree_->isInTree(goal_node));

               return false;
             }
             return true;
           }());

    std::vector<ConnectionPtr> all_parent_connections = goal_node->getParentConnections();
    std::vector<ConnectionPtr> net_parent_connections = goal_node->getNetParentConnections();
    all_parent_connections.insert(all_parent_connections.end(),net_parent_connections.begin(),net_parent_connections.end());

    double cost2parent;
    for(const ConnectionPtr& conn2parent:all_parent_connections)
    {
      parent = conn2parent->getParent();
      cost2parent = cost2here+conn2parent->getCost();

      if(cost2parent>=cost2beat)
      {
        if(verbose_)
          ROS_INFO("cost up to now %f, cost to beat %f -> don't follow this branch!",cost2parent,cost2beat);

        continue;
      }

      std::vector<ConnectionPtr> connections2parent = connections2here;
      if(parent == start_node)
      {
        //When the start node is reached, a solution is found -> reverse the connections vector and insert into the map
        connections2parent.push_back(conn2parent);
        std::reverse(connections2parent.begin(),connections2parent.end());

        std::pair<double,std::vector<ConnectionPtr>> pair;
        pair.first = cost2parent;
        pair.second = connections2parent;

        if(verbose_)
        {
          ROS_INFO_STREAM("New conn inserted: "<<conn2parent<<" "<<*conn2parent<<" cost up to now: "<<cost2parent<<" cost to beat: "<<cost2beat);
          ROS_INFO_STREAM("Start node reached! Cost: "<<cost2parent);
        }

        map.insert(pair);
      }
      else
      {
        if(std::find(black_list.begin(), black_list.end(), parent) != black_list.end())
        {
          if(verbose_)
            ROS_INFO_STREAM("parent belongs to black list, skipping..");

          continue;
        }

        if(std::find(visited_nodes.begin(), visited_nodes.end(), parent) != visited_nodes.end())
        {
          if(verbose_)
            ROS_INFO_STREAM("avoiding cycles...");

          continue;
        }
        else
          visited_nodes.push_back(parent);

        connections2parent.push_back(conn2parent);

        if(verbose_)
          ROS_INFO_STREAM("New conn inserted: "<<conn2parent<<" "<<*conn2parent<<" cost up to now: "<<cost2parent<<" cost to beat: "<<cost2beat);

        std::multimap<double,std::vector<ConnectionPtr>> map_to_start_through_parent;
        map_to_start_through_parent= computeConnectionFromNodeToNode(start_node,parent,cost2parent,cost2beat,
                                                                     black_list,visited_nodes,connections2parent);
        visited_nodes.pop_back();

        if(not map_to_start_through_parent.empty())
          map.insert(map_to_start_through_parent.begin(),map_to_start_through_parent.end());
      }
    }
    return map;
  }
}


}  // end namespace pathplan
