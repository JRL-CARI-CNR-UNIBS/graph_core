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
#include <graph_core/utils.h>
#include <graph_core/connection.h>

namespace ha_planner
{

class Node: public std::enable_shared_from_this<Node>
{

protected:
  double m_cost=0;
  std::vector<double> m_q;

  bool m_is_collision_checked;
  bool m_is_in_collision;
  //reference member is not stored in memory, but you need to use it carefully: life cycle is not automatically managed. In this library there are no issues because Nodes and Connections are members of a Net.
  const NodeParams& m_params;

public:
  std::vector<ConnectionPtr> m_connections;
  std::vector<ConnectionPtr> m_parent_connections;
  std::vector<ConnectionPtr> m_child_connections;

  Node(const std::vector<double>& q,
       const NodeParams& node_parameters);
  std::shared_ptr<ha_planner::Node> pointer(){return shared_from_this();}

  const unsigned int getConnectionsNumber(){return m_connections.size();}

  void setCost(const double& cost){m_cost=cost;};
  double getCost(){return m_cost;}

  std::vector<double> getJoints();
  const std::vector<double>& getScaledJoints() const {return m_q;}
  const bool& isCollisionChecked(){return m_is_collision_checked;}
  bool isInCollision(const planning_scene::PlanningSceneConstPtr &planning_scene);

  bool isUnconnected(const planning_scene::PlanningSceneConstPtr &planning_scene);
  void addConnection(const ConnectionPtr& connection);
  bool removeConnection(ConnectionPtr& connection);

  std::vector<NodePtr> getParents();
  std::vector<NodePtr> getChilds();
  bool checkIfConnectedWith(const NodePtr& node, ConnectionPtr& connection);

  void print();
};

}

