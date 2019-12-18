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

#include <graph_core/graph_core.h>
#include <graph_core/node.h>
#include <graph_core/connection.h>
namespace ha_planner
{

class Graph
{
protected:

  std::vector<NodePtr> m_nodes;
  unsigned int m_dof;

  std::vector<double> m_lb;
  std::vector<double> m_ub;
  Eigen::VectorXd m_unscaling;
  NodeParams m_node_params;
  planning_scene::PlanningSceneConstPtr m_planning_scene;


  friend Problem;
public:
  Graph(const unsigned int& dof,
      const std::string& group_name,
      const planning_scene::PlanningSceneConstPtr &planning_scene,
      const std::vector<double> scaling,
      const std::vector<double> lb,
      const std::vector<double> ub);

  NodePtr searchClosestNode(const std::vector<double>& q);
  NodePtr addNode(const std::vector<double>& q);
  NodePtr addChildNode(const NodePtr& parent_node, const std::vector<double>& q);
  NodePtr addParentNode(const NodePtr& child_node, const std::vector<double>& q);
  NodePtr addConnectedNode(const NodePtr& node, const std::vector<double>& q, Direction dir);
  ConnectionPtr addConnection(const NodePtr& parent, const NodePtr& child);

  bool checkNodeCollision(const NodePtr& node);
  bool checkConnectionCollision(const ConnectionPtr& conn);
  void removeNodeWithConnections(NodePtr& node);
  void removeConnection(ConnectionPtr& conn);

  unsigned int getNodeNumber()
  {
    return m_nodes.size();
  }

  void setPlanningScene( const planning_scene::PlanningSceneConstPtr& planning_scene )
  {
    m_planning_scene=planning_scene;
  }

  // node removal functions
  unsigned int removeUnconnectedNodes();
};


}

