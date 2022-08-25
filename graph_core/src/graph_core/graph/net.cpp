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

std::multimap<double,std::vector<ConnectionPtr>>& Net::getConnectionBetweenNodes(const NodePtr &start_node, const NodePtr& goal_node, const std::vector<NodePtr>& black_list)
{
  double cost2beat = std::numeric_limits<double>::infinity();
  return getConnectionBetweenNodes(start_node,goal_node,cost2beat,black_list);
}

std::multimap<double,std::vector<ConnectionPtr>>& Net::getConnectionBetweenNodes(const NodePtr &start_node, const NodePtr& goal_node, const double& cost2beat, const std::vector<NodePtr>& black_list)
{
  ros::WallTime tic = ros::WallTime::now();

  std::vector<NodePtr> visited_nodes;
  visited_nodes.push_back(goal_node);

  double cost2here = 0.0;

  map_.clear();
  time_vector_.clear();
  curse_of_dimensionality_ = 0;

  tic_fcn_call_ = ros::WallTime::now();
  computeConnectionFromNodeToNode(start_node,goal_node,cost2here,cost2beat,black_list,visited_nodes);

  double time_mean = 0;
  time_mean = std::accumulate(time_vector_.begin(),time_vector_.end(),time_mean)/((double) time_vector_.size());

  ROS_INFO("TIME VECTOR LENGTH %i, nodes length %i",time_vector_.size(), curse_of_dimensionality_);

  ROS_WARN_STREAM("NET TIME: "<<(ros::WallTime::now()-tic).toSec()<<" NODE TIME MEAN: "<<time_mean);
  double calc = curse_of_dimensionality_*time_mean;
  ROS_INFO_STREAM("calc: "<<calc<<" diff: "<<((ros::WallTime::now()-tic).toSec()-calc));

  return map_;
}

std::multimap<double,std::vector<ConnectionPtr>>& Net::getConnectionToNode(const NodePtr &node, const std::vector<NodePtr>& black_list)
{
  double cost2beat = std::numeric_limits<double>::infinity();
  return getConnectionToNode(node,cost2beat,black_list);
}

std::multimap<double,std::vector<ConnectionPtr>>& Net::getConnectionToNode(const NodePtr &node, const double& cost2beat, const std::vector<NodePtr>& black_list)
{

  std::vector<NodePtr> visited_nodes;
  visited_nodes.push_back(node);

  double cost2here = 0.0;

  map_.clear();
  time_vector_.clear();
  curse_of_dimensionality_ = 0;

  computeConnectionFromNodeToNode(linked_tree_->getRoot(),node,cost2here,cost2beat,black_list,visited_nodes);

  return map_;
}

void Net::computeConnectionFromNodeToNode(const NodePtr& start_node, const NodePtr& goal_node, std::vector<NodePtr> &visited_nodes)
{
  std::vector<NodePtr> black_list;
  return computeConnectionFromNodeToNode(start_node,goal_node,black_list,visited_nodes);
}

void Net::computeConnectionFromNodeToNode(const NodePtr& start_node, const NodePtr& goal_node, const std::vector<NodePtr> &black_list, std::vector<NodePtr> &visited_nodes)
{
  double cost2here = 0.0;
  double cost2beat = std::numeric_limits<double>::infinity();
  return computeConnectionFromNodeToNode(start_node, goal_node, cost2here, cost2beat, black_list, visited_nodes);
}

void Net::computeConnectionFromNodeToNode(const NodePtr& start_node, const NodePtr& goal_node, const double& cost2here, const double& cost2beat, const std::vector<NodePtr> &black_list, std::vector<NodePtr> &visited_nodes)
{
  cost_to_beat_ = cost2beat;
  std::vector<ConnectionPtr> connections2here;
  return computeConnectionFromNodeToNode(start_node,goal_node,cost2here,black_list,visited_nodes,connections2here);
}

void Net::computeConnectionFromNodeToNode(const NodePtr& start_node, const NodePtr& goal_node, const double& cost2here, const std::vector<NodePtr> &black_list, std::vector<NodePtr> &visited_nodes, std::vector<ConnectionPtr>& connections2here)
{
  long double fcn_time = (ros::WallTime::now()-tic_fcn_call_).toSec();
  assert([&]() ->bool{
           if(fcn_time>1e-03)
           {
           ROS_WARN_STREAM("fcn_time "<<fcn_time);
             return false;
           }
           return true;
         }());
  //ROS_INFO_STREAM("time fcn call "<<fcn_time);

  //Depth-first search
  curse_of_dimensionality_++;

//  ros::WallTime tic = ros::WallTime::now();
  ros::WallTime tic_tot = ros::WallTime::now();

  long double time_tot = 0.0;
//  long double time_if = 0.0;
//  long double time_init = 0.0;
//  long double time_insert = 0.0;
//  long double time_cost = 0.0;
//  long double time_rev = 0.0;
//  long double time_bl = 0.0;
//  long double time_vn = 0.0;
//  long double time_map = 0.0;

  NodePtr parent;

//  time_init = (ros::WallTime::now()-tic).toSec();
//  assert([&]() ->bool{
//           if(time_init>1e-03)
//           {
//           ROS_WARN_STREAM("time_init "<<time_init);
//             return false;
//           }
//           return true;
//         }());
//  tic = ros::WallTime::now();

  if(goal_node == linked_tree_->getRoot() || goal_node == start_node)
  {
    //ROS_INFO("node %i, time_if %lf, time_init %lf, time_insert %lf, time_cost %lf, time_rev %lf, time_bl %lf, time_vn %lf",curse_of_dimensionality_,time_if, time_init, time_insert, time_cost, time_rev, time_bl, time_vn);

    time_tot = (ros::WallTime::now()-tic_tot).toSec();
//    ROS_INFO_STREAM("node "<<curse_of_dimensionality_<<" time "<<time_tot);
    return;
  }
  else
  {
//    time_if = (ros::WallTime::now()-tic).toSec();
//    assert([&]() ->bool{
//             if(time_if>1e-03)
//             {
//             ROS_WARN_STREAM("time_if "<<time_if);
//               return false;
//             }
//             return true;
//           }());

//    assert([&]() ->bool{
//             if(goal_node->getParentConnectionsSize() != 1)
//             {
//               ROS_ERROR("a node of a tree should have only a parent");
//               ROS_ERROR_STREAM("goal node \n" <<*goal_node);

//               ROS_INFO_STREAM("current root "<<linked_tree_->getRoot()->getConfiguration().transpose()<< " "<<linked_tree_->getRoot());
//               ROS_INFO_STREAM("goal node "<<goal_node->getConfiguration().transpose()<<" "<<goal_node);
//               ROS_INFO_STREAM("start node "<<start_node->getConfiguration().transpose()<<" "<<start_node);

//               if(goal_node->getNetChildConnectionsSize()>0);
//               ROS_INFO_STREAM("the child "<<*goal_node->netChildConnection(0)->getChild()<<" "<<goal_node->netChildConnection(0)->getChild());

//               int count = 0;
//               for(const NodePtr& n:linked_tree_->getNodes())
//               {
//                 if(n->getConfiguration() == goal_node->getConfiguration())
//                 {
//                   count++;
//                   ROS_INFO_STREAM(*n<<"\n"<<n<<"\n count "<<count);
//                 }
//               }
//               ROS_INFO_STREAM("EQUAL NODES "<<count);

//               ROS_INFO_STREAM("in tree: "<<linked_tree_->isInTree(goal_node));

//               return false;
//             }
//             return true;
//           }());

//    tic = ros::WallTime::now();

    std::vector<ConnectionPtr> all_parent_connections = goal_node->getParentConnections();
    std::vector<ConnectionPtr> net_parent_connections = goal_node->getNetParentConnections();
    all_parent_connections.insert(all_parent_connections.end(),net_parent_connections.begin(),net_parent_connections.end());

//    time_insert = (ros::WallTime::now()-tic).toSec();
//    assert([&]() ->bool{
//             if(time_insert>1e-03)
//             {
//             ROS_WARN_STREAM("time_insert "<<time_insert);
//               return false;
//             }
//             return true;
//           }());

    ros::WallTime tic_cycle;
    time_tot = (ros::WallTime::now()-tic_tot).toSec();

    double cost2parent;
    for(const ConnectionPtr& conn2parent:all_parent_connections)
    {
      tic_cycle = ros::WallTime::now();

      parent = conn2parent->getParent();
      cost2parent = cost2here+conn2parent->getCost();

      if(cost2parent == std::numeric_limits<double>::infinity() || cost2parent>=cost_to_beat_ || std::abs(cost2parent-cost_to_beat_)<=NET_ERROR_TOLERANCE) //NET_ERROR_TOLERANCE to cope with machine errors
      {
        if(verbose_)
          ROS_INFO("cost up to now %lf, cost to beat %f -> don't follow this branch!",cost2parent,cost_to_beat_);

//        time_cost = (ros::WallTime::now()-tic).toSec();
//        assert([&]() ->bool{
//                 if(time_cost>1e-03)
//                 {
//                 ROS_WARN_STREAM("time_cost "<<time_cost);
//                   return false;
//                 }
//                 return true;
//               }());

        time_tot += (ros::WallTime::now()-tic_cycle).toSec();

        continue;
      }

//      time_cost = (ros::WallTime::now()-tic).toSec();
//      assert([&]() ->bool{
//               if(time_cost>1e-03)
//               {
//               ROS_WARN_STREAM("time_cost "<<time_cost);
//                 return false;
//               }
//               return true;
//             }());

      assert([&]() ->bool{
               if(not(cost2parent<cost_to_beat_))
               {
                 ROS_INFO("cost to parent %f, cost to beat %f ",cost2parent,cost_to_beat_);
                 return false;
               }
               return true;
             }());

      double cost_heuristics = cost2parent+(parent->getConfiguration()-start_node->getConfiguration()).norm();
      if(cost_heuristics>=cost_to_beat_ || std::abs(cost_heuristics-cost_to_beat_)<=NET_ERROR_TOLERANCE )
      {
        if(verbose_)
          ROS_INFO("cost heuristic through this node %lf, cost to beat %f -> don't follow this branch!",cost_heuristics,cost_to_beat_);

        time_tot += (ros::WallTime::now()-tic_cycle).toSec();

        continue;
      }

      assert([&]() ->bool{
               if(not(cost_heuristics<cost_to_beat_))
               {
                 ROS_INFO("cost heuristics %f, cost to beat %f ",cost_heuristics,cost_to_beat_);
                 return false;
               }
               return true;
             }());

      if(parent == start_node)
      {
        //When the start node is reached, a solution is found -> insert into the map

//        tic = ros::WallTime::now();

        std::vector<ConnectionPtr> connections2parent = connections2here;
        connections2parent.push_back(conn2parent);

        std::reverse(connections2parent.begin(),connections2parent.end());

//        time_rev = (ros::WallTime::now()-tic).toSec();
//        assert([&]() ->bool{
//                 if(time_rev>1e-03)
//                 {
//                 ROS_WARN_STREAM("time_rev "<<time_rev);
//                   return false;
//                 }
//                 return true;
//               }());
//        tic = ros::WallTime::now();

        std::pair<double,std::vector<ConnectionPtr>> pair;
        pair.first = cost2parent;
        pair.second = connections2parent;

        if(not search_every_solution_) //update cost_to_beat_ -> search only for better solutions than this one
          cost_to_beat_ = cost2parent;

        if(verbose_)
        {
          ROS_INFO_STREAM("New conn inserted: "<<conn2parent<<" "<<*conn2parent<<" cost up to now: "<<cost2parent<<" cost to beat: "<<cost_to_beat_);
          ROS_INFO_STREAM("Start node reached! Cost: "<<cost2parent<<" (cost to beat updated)");
        }

        map_.insert(pair);

        time_tot += (ros::WallTime::now()-tic_cycle).toSec();

//        time_map= (ros::WallTime::now()-tic).toSec();
//        assert([&]() ->bool{
//                 if(time_map>1e-03)
//                 {
//                 ROS_WARN_STREAM("time_map "<<time_map);
//                   return false;
//                 }
//                 return true;
//               }());
//        ROS_INFO_STREAM("1node "<<curse_of_dimensionality_<<" time_if "<<time_if<<" time_init "<<time_init<<" time_insert "<<time_insert<<" time_cost "<<time_cost<<" time_rev "<< time_rev <<" time_bl "<<time_bl<<" time_vn"<< time_vn<<" time_map"<<time_map);

      }
      else
      {
//        tic = ros::WallTime::now();

        if(std::find(black_list.begin(), black_list.end(), parent) != black_list.end())
        {
          if(verbose_)
            ROS_INFO_STREAM("parent belongs to black list, skipping..");

          time_tot += (ros::WallTime::now()-tic_cycle).toSec();

          continue;
        }

//        time_bl = (ros::WallTime::now()-tic).toSec();
//        assert([&]() ->bool{
//                 if(time_bl>1e-03)
//                 {
//                 ROS_WARN_STREAM("time_bl "<<time_bl);
//                   return false;
//                 }
//                 return true;
//               }());

//        tic = ros::WallTime::now();

        if(std::find(visited_nodes.begin(), visited_nodes.end(), parent) != visited_nodes.end())
        {
          if(verbose_)
            ROS_INFO_STREAM("avoiding cycles...");

//          time_vn = (ros::WallTime::now()-tic).toSec();
//          assert([&]() ->bool{
//                   if(time_vn>1e-03)
//                   {
//                   ROS_WARN_STREAM("time_vn "<<time_vn);
//                     return false;
//                   }
//                   return true;
//                 }());

          time_tot += (ros::WallTime::now()-tic_cycle).toSec();

          continue;
        }
        else
          visited_nodes.push_back(parent);

//        time_vn = (ros::WallTime::now()-tic).toSec();
//        assert([&]() ->bool{
//                 if(time_vn>1e-03)
//                 {
//                 ROS_WARN_STREAM("time_vn "<<time_vn);
//                   return false;
//                 }
//                 return true;
//               }());

        std::vector<ConnectionPtr> connections2parent = connections2here;
        connections2parent.push_back(conn2parent);

        if(verbose_)
          ROS_INFO_STREAM("New conn inserted: "<<conn2parent<<" "<<*conn2parent<<" cost up to now: "<<cost2parent<<" cost to beat: "<<cost_to_beat_);


        //ROS_INFO("node %i, time_if %lf, time_init %lf, time_insert %lf, time_cost %lf, time_rev %lf, time_bl %lf, time_vn %lf",curse_of_dimensionality_,time_if, time_init, time_insert, time_cost, time_rev, time_bl, time_vn);
        //ROS_INFO_STREAM("2node "<<curse_of_dimensionality_<<" time_if "<<time_if<<" time_init "<<time_init<<" time_insert "<<time_insert<<" time_cost "<<time_cost<<" time_rev "<< time_rev <<" time_bl "<<time_bl<<" time_vn"<< time_vn);

        time_tot += (ros::WallTime::now()-tic_cycle).toSec();

        tic_fcn_call_ = ros::WallTime::now();
        computeConnectionFromNodeToNode(start_node,parent,cost2parent,black_list,
                                        visited_nodes,connections2parent);

        ros::WallTime tic_cycle2 = ros::WallTime::now();
//        tic = ros::WallTime::now();
        visited_nodes.pop_back();

        time_tot += (ros::WallTime::now()-tic_cycle2).toSec();

//        long double time_map = (ros::WallTime::now()-tic).toSec();
//        assert([&]() ->bool{
//                 if(time_map>1e-03)
//                 {
//                 ROS_WARN_STREAM("time_map "<<time_map);
//                   return false;
//                 }
//                 return true;
//               }());
//        ROS_INFO("map time %lf",time_map);
      }
    }

//    ROS_INFO_STREAM("node "<<curse_of_dimensionality_<<" time "<<time_tot);

    time_vector_.push_back(time_tot);
    return;
  }
}

}  // end namespace pathplan
