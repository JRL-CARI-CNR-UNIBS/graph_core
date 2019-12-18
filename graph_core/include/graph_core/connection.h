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
#include <graph_core/node.h>

namespace ha_planner
{

class Connection: public std::enable_shared_from_this<ha_planner::Connection>
{
protected:
  NodePtr m_parent;
  NodePtr m_child;
  double m_square_length;
  double m_length;

  bool m_is_collision_checked;
  bool m_is_in_collision;
  virtual void checkCollisionNew(const planning_scene::PlanningSceneConstPtr& planning_scene);
  virtual bool checkCollisionIteration(const planning_scene::PlanningSceneConstPtr& planning_scene,
                                       const std::vector<double>& q1,
                                       const std::vector<double>& q2,
                                       robot_state::RobotState& state);
  virtual void computeLength();
  const NodeParams& m_params;
public:
  Connection(const NodePtr& parent,
             const NodePtr& child,
             const NodeParams& parameters); // excepetion if not valid

  std::shared_ptr<ha_planner::Connection> pointer(){return shared_from_this();}

  const bool& isCollisionChecked(){return m_is_collision_checked;}
  bool isInCollision(const planning_scene::PlanningSceneConstPtr &planning_scene);
  void forceNotCollision(){m_is_collision_checked=true;m_is_in_collision=false;}
  void forceCollision(){m_is_collision_checked=true;m_is_in_collision=true;}

  const NodePtr& getOtherNode(const NodePtr& node);
  double getLength(){return m_length;}
  double getSquareLength(){return m_square_length;}
  double getWeigthedLength();


  void print();
  bool isMember(const NodePtr& node){return (m_parent==node || m_child==node);}
  void getNodes(NodePtr& parent, NodePtr& child);
  const NodePtr& getParent(){return m_parent;}
  const NodePtr& getChild(){return m_child;}


  Eigen::VectorXd versor();
  double dotProduct(const ConnectionPtr& conn);
  void flipDirection();
  ConnectionPtr createFlippedConnection();
  void registerConnection();

};


}

