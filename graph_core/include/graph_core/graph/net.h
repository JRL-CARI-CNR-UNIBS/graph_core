#pragma once
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

#include <graph_core/util.h>
#include <graph_core/graph/subtree.h>

namespace pathplan
{
class Net;
typedef std::shared_ptr<Net> NetPtr;

class Net: public std::enable_shared_from_this<Net>
{

#define NET_ERROR_TOLERANCE 1e-12

protected:
  TreePtr linked_tree_;
  std::multimap<double,std::vector<ConnectionPtr>> map_;
  double cost_to_beat_;
  bool verbose_;
  bool search_every_solution_;
  bool search_in_tree_;
  int curse_of_dimensionality_;
  ros::WallTime tic_fcn_call_;
  ros::WallTime tic_search_;
  double max_time_;
  std::vector<double> time_vector_;

  void computeConnectionFromNodeToNode(const NodePtr &start_node, const NodePtr &goal_node, std::vector<NodePtr>& visited_nodes);
  void computeConnectionFromNodeToNode(const NodePtr &start_node, const NodePtr &goal_node, const std::vector<NodePtr> &black_list, std::vector<NodePtr>& visited_nodes);
  void computeConnectionFromNodeToNode(const NodePtr& start_node, const NodePtr& goal_node, const double& cost2here, const double& cost2beat, const std::vector<NodePtr> &black_list, std::vector<NodePtr> &visited_nodes);
  void computeConnectionFromNodeToNode(const NodePtr& start_node, const NodePtr& goal_node, const double& cost2here, const std::vector<NodePtr> &black_list, std::vector<NodePtr> &visited_nodes, std::vector<ConnectionPtr>& connections2here);
  bool purgeSuccessors(NodePtr& node, const std::vector<NodePtr>& white_list, unsigned int& removed_nodes);  //VEDI CON MANUEL

public:
  Net(const TreePtr& tree)
  {
    verbose_ = false;
    search_every_solution_ = true;
    setTree(tree);
  }

  void setTree(const TreePtr& tree)
  {
//    if(tree->isSubtree())
//    {
//      SubtreePtr subtree = std::static_pointer_cast<Subtree>(tree);
//      linked_tree_ = subtree->getParentTree();
//    }
//    else
//      linked_tree_ = tree;
    linked_tree_ = tree;
  }

  TreePtr getTree()
  {
    return linked_tree_;
  }

  void setVerbosity(const bool verbose)
  {
    verbose_ = verbose;
  }

  void searchEverySolution(const bool search_every_solution)
  {
    search_every_solution_ = search_every_solution;
  }

  bool purgeFromHere(ConnectionPtr& conn2node, const std::vector<NodePtr>& white_list, unsigned int& removed_nodes); //VEDI CON MANUEL

  std::multimap<double,std::vector<ConnectionPtr>>& getConnectionToNode(const NodePtr& node, const std::vector<NodePtr>& black_list = {}, const double& max_time = std::numeric_limits<double>::infinity());
  std::multimap<double,std::vector<ConnectionPtr>>& getConnectionToNode(const NodePtr& node, const double& cost2beat, const std::vector<NodePtr>& black_list = {}, const double& max_time = std::numeric_limits<double>::infinity());

  std::multimap<double,std::vector<ConnectionPtr>>& getConnectionBetweenNodes(const NodePtr& start_node, const NodePtr& goal_node, const std::vector<NodePtr> &black_list = {}, const double& max_time = std::numeric_limits<double>::infinity());
  std::multimap<double,std::vector<ConnectionPtr>>& getConnectionBetweenNodes(const NodePtr& start_node, const NodePtr& goal_node, const double& cost2beat, const std::vector<NodePtr> &black_list = {}, const double& max_time = std::numeric_limits<double>::infinity(), const bool &search_in_tree = false);
};

}
