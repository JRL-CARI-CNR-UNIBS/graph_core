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
Net::Net(const TreePtr& tree, const cnr_logger::TraceLoggerPtr& logger):
{
  logger_ = logger;
  verbose_ = false;
  search_every_solution_ = true;

  setTree(tree);
  setMetrics(tree->getMetrics());

  cost_evaluation_condition_ = nullptr;
}

std::multimap<double,std::vector<ConnectionPtr>>& Net::getConnectionBetweenNodes(const NodePtr &start_node, const NodePtr& goal_node, const std::vector<NodePtr>& black_list, const double& max_time)
{
  double cost2beat = std::numeric_limits<double>::infinity();
  return getConnectionBetweenNodes(start_node,goal_node,cost2beat,black_list,max_time);
}

std::multimap<double,std::vector<ConnectionPtr>>& Net::getConnectionBetweenNodes(const NodePtr &start_node, const NodePtr& goal_node, const double& cost2beat, const std::vector<NodePtr>& black_list, const double& max_time, const bool& search_in_tree)
{
  double cost2here = 0.0;

  black_list_.clear();
  black_list_ = black_list;

  visited_nodes_.clear();
  visited_nodes_.push_back(goal_node);

  map_.clear();
  connections2parent_.clear();

  max_time_ = max_time;

  time_vector_.clear();
  curse_of_dimensionality_ = 0;
  search_in_tree_ = search_in_tree;

  if(max_time_<=0.0)
    return map_;

  tic_search_ = std::chrono::system_clock::now();
  computeConnectionFromNodeToNode(start_node,goal_node,cost2here,cost2beat);

  return map_;
}

std::multimap<double,std::vector<ConnectionPtr>>& Net::getConnectionToNode(const NodePtr &node, const std::vector<NodePtr>& black_list, const double& max_time)
{
  double cost2beat = std::numeric_limits<double>::infinity();
  return getConnectionToNode(node,cost2beat,black_list,max_time);
}

std::multimap<double,std::vector<ConnectionPtr>>& Net::getConnectionToNode(const NodePtr &node, const double& cost2beat, const std::vector<NodePtr>& black_list, const double& max_time)
{
  double cost2here = 0.0;

  black_list_.clear();
  black_list_ = black_list;

  visited_nodes_.clear();
  visited_nodes_.push_back(node);

  map_.clear();
  connections2parent_.clear();

  max_time_ = max_time;

  time_vector_.clear();
  curse_of_dimensionality_ = 0;

  if(max_time_<=0.0)
    return map_;

  tic_search_ = std::chrono::system_clock::now();
  computeConnectionFromNodeToNode(linked_tree_->getRoot(),node,cost2here,cost2beat);

  return map_;
}

void Net::computeConnectionFromNodeToNode(const NodePtr& start_node, const NodePtr& goal_node)
{
  double cost2here = 0.0;
  double cost2beat = std::numeric_limits<double>::infinity();
  return computeConnectionFromNodeToNode(start_node, goal_node, cost2here, cost2beat);
}

void Net::computeConnectionFromNodeToNode(const NodePtr& start_node, const NodePtr& goal_node, const double& cost2here, const double& cost2beat)
{
  cost_to_beat_ = cost2beat;
  return computeConnectionFromNodeToNode(start_node,goal_node,cost2here);
}

void Net::computeConnectionFromNodeToNode(const NodePtr& start_node, const NodePtr& goal_node, const double& cost2here)
{
  std::chrono::time_point<std::chrono::system_clock> time_black_list_check, time_visited_list_check;
  std::chrono::time_point<std::chrono::system_clock> now = std::chrono::system_clock::now();
  std::chrono::time_point<std::chrono::system_clock> tic_tot = now;

  if(verbose_)
    CNR_INFO(logger_,"time in: "<<(std::chrono::duration<double>(now-tic_search_)).count());

  //Depth-first search
  curse_of_dimensionality_++;

  std::chrono::duration<double> time_tot;

  NodePtr parent;

  if(goal_node == linked_tree_->getRoot() || goal_node == start_node)
  {
    now = std::chrono::system_clock::now();
    time_tot = now-tic_tot;

    if(verbose_)
      CNR_INFO(logger_,"time return: "<<time_tot.count());

    return;
  }
  else
  {
    std::vector<ConnectionPtr> all_parent_connections = goal_node->getParentConnections();
    std::vector<ConnectionPtr> net_parent_connections = goal_node->getNetParentConnections();
    all_parent_connections.insert(all_parent_connections.end(),net_parent_connections.begin(),net_parent_connections.end());

    std::chrono::time_point<std::chrono::system_clock> tic_cycle;

    now = std::chrono::system_clock::now();
    time_tot = now-tic_tot;

    double time2now;
    double cost2parent;
    for(const ConnectionPtr& conn2parent:all_parent_connections)
    {
      tic_cycle = std::chrono::system_clock::now();
      time2now =  (std::chrono::duration<double>(tic_cycle-tic_search_)).count();

      if(verbose_)
        CNR_INFO(logger_,"Available time: "<<(std::chrono::duration<double>(max_time_-time2now)).count());

      if(time2now>0.9*max_time_)
      {
        if(verbose_)
        {
          now = std::chrono::system_clock::now();
          CNR_INFO(logger_,"Net max time exceeded! Time: "<<time2now<<" max time: "<<max_time_);
          CNR_INFO(logger_,"time return: "<<(std::chrono::duration<double>(now-tic_cycle)).count());
        }
        return;
      }

      parent = conn2parent->getParent();

      if(search_in_tree_)
      {
        if(not linked_tree_->isInTree(parent))
          continue;
      }

      if(cost_evaluation_condition_ && (*cost_evaluation_condition_)(conn2parent)) //if a condition exists and it is met, re-evaluate the connection cost
        conn2parent->setCost(metrics_->cost(conn2parent->getParent(),conn2parent->getChild()));

      cost2parent = cost2here+conn2parent->getCost();

      if(cost2parent == std::numeric_limits<double>::infinity() || cost2parent>=cost_to_beat_ || std::abs(cost2parent-cost_to_beat_)<=NET_ERROR_TOLERANCE) //NET_ERROR_TOLERANCE to cope with machine errors
      {
        now = std::chrono::system_clock::now();
        time_tot = time_tot+(now-tic_cycle);

        if(verbose_)
        {
          CNR_INFO(logger_,"cost up to now %lf, cost to beat %f -> don't follow this branch!",cost2parent,cost_to_beat_);
          CNR_INFO(logger_,"time don't follow branch: "<<(std::chrono::duration<double>(now-tic_cycle)).count());
        }
        continue;
      }

      assert([&]() ->bool{
               if(not(cost2parent<cost_to_beat_))
               {
                 CNR_INFO(logger_,"cost to parent %f, cost to beat %f ",cost2parent,cost_to_beat_);
                 return false;
               }
               return true;
             }());

      double cost_heuristics = cost2parent+metrics_->utopia(parent->getConfiguration(),start_node->getConfiguration());
      if(cost_heuristics>=cost_to_beat_ || std::abs(cost_heuristics-cost_to_beat_)<=NET_ERROR_TOLERANCE )
      {
        now = std::chrono::system_clock::now();
        time_tot = time_tot+(now-tic_cycle);

        if(verbose_)
        {
          CNR_INFO(logger_,"cost heuristic through this node %lf, cost to beat %f -> don't follow this branch!",cost_heuristics,cost_to_beat_);
          CNR_INFO(logger_,"time cost heuristics: "<<(std::chrono::duration<double>(now-tic_cycle)).count());
        }
        continue;
      }

      assert([&]() ->bool{
               if(not(cost_heuristics<cost_to_beat_))
               {
                 CNR_INFO(logger_,"cost heuristics %f, cost to beat %f ",cost_heuristics,cost_to_beat_);
                 return false;
               }
               return true;
             }());

      if(parent == start_node)
      {
        //When the start node is reached, a solution is found -> insert into the map

        std::vector<ConnectionPtr> connections2start = connections2parent_;
        connections2start.push_back(conn2parent);

        std::reverse(connections2start.begin(),connections2start.end());

        std::pair<double,std::vector<ConnectionPtr>> pair;
        pair.first = cost2parent;
        pair.second = connections2start;


        if(not search_every_solution_) //update cost_to_beat_ -> search only for better solutions than this one
          cost_to_beat_ = cost2parent;

        if(verbose_)
        {
          CNR_INFO(logger_,"New conn inserted: "<<conn2parent<<" "<<*conn2parent<<" cost up to now: "<<cost2parent<<" cost to beat: "<<cost_to_beat_);
          CNR_INFO(logger_,"Start node reached! Cost: "<<cost2parent<<" (cost to beat updated)");
        }

        map_.insert(pair);

        time_tot = time_tot+(std::chrono::system_clock::now()-tic_cycle);
      }
      else
      {
        time_black_list_check = std::chrono::system_clock::now();
        if(std::find(black_list_.begin(),black_list_.end(),parent)<black_list_.end())
        {
          now = std::chrono::system_clock::now();
          time_tot = time_tot + (now-tic_cycle);

          if(verbose_)
          {
            CNR_INFO(logger_,"parent belongs to black list, skipping..");
            CNR_INFO(logger_,"time black list: "<<(std::chrono::duration<double>(now-tic_cycle)).count()<<" check: "
                     <<(std::chrono::duration<double>(now-time_black_list_check)).count());
          }
          continue;
        }

        now = std::chrono::system_clock::now();
        if(verbose_)
          CNR_INFO(logger_,"time black list check: "<<((std::chrono::duration<double>(now-time_black_list_check)).count()));

        time_visited_list_check = std::chrono::system_clock::now();
        if(std::find(visited_nodes_.begin(),visited_nodes_.end(),parent)<visited_nodes_.end())
        {
          now = std::chrono::system_clock::now();
          time_tot = time_tot+(now-tic_cycle);

          if(verbose_)
          {
            CNR_INFO(logger_,"avoiding cycles...");
            CNR_INFO(logger_,"time visited nodes: "<<(std::chrono::duration<double>(now-tic_cycle)).count()<<" check: "<<(std::chrono::duration<double>(now-time_visited_list_check)).count());
          }

          continue;
        }
        else
          visited_nodes_.push_back(parent);

        now = std::chrono::system_clock::now();
        if(verbose_)
          CNR_INFO(logger_,"time visited list check: "<<(std::chrono::duration<double>(now-time_visited_list_check)).count());

        connections2parent_.push_back(conn2parent);

        now = std::chrono::system_clock::now();
        time_tot = time_tot+(now-tic_cycle);

        if(verbose_)
        {
          CNR_INFO(logger_,"New conn inserted: "<<conn2parent<<" "<<*conn2parent<<" cost up to now: "<<cost2parent<<" cost to beat: "<<cost_to_beat_);
          CNR_INFO(logger_,"time before: "<<(std::chrono::duration<double>(now-tic_search_)).count()<<" time cycle "<<(std::chrono::duration<double>(now-tic_cycle)).count());
        }

        computeConnectionFromNodeToNode(start_node,parent,cost2parent);

        std::chrono::time_point<std::chrono::system_clock> tic_cycle2 = std::chrono::system_clock::now();
        visited_nodes_.pop_back();
        connections2parent_.pop_back();

        now = std::chrono::system_clock::now();
        time_tot = time_tot+(now-tic_cycle2);
      }
    }

    time_vector_.push_back(time_tot);
    return;
  }
}

}  // end namespace pathplan
