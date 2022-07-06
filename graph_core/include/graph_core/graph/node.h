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

#include <eigen3/Eigen/Core>
#include <ros/ros.h>
#include <graph_core/util.h>
#include <graph_core/graph/connection.h>

namespace pathplan
{
typedef std::vector<ConnectionWeakPtr> WeakPtrVector;

class Node: public std::enable_shared_from_this<Node>
{
protected:
  Eigen::VectorXd configuration_;
  unsigned int ndof_;
  bool analyzed_;
  bool non_optimal_;

  std::vector<ConnectionWeakPtr> parent_connections_;     //Weak ptr to avoid pointers cycles
  std::vector<ConnectionWeakPtr> net_parent_connections_; //Weak ptr to avoid pointers cycles

  std::vector<ConnectionPtr> child_connections_;
  std::vector<ConnectionPtr> net_child_connections_;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Node(const Eigen::VectorXd& configuration);
  NodePtr pointer()
  {
    return shared_from_this();
  }
  void setAnalyzed(const bool& analyzed);
  bool getAnalyzed();
  void setNonOptimal(const bool& nonOptimal);
  bool getNonOptimal();
  void addParentConnection(const ConnectionPtr& connection);
  void addChildConnection(const ConnectionPtr& connection);
  void addNetParentConnection(const ConnectionPtr& connection);
  void addNetChildConnection(const ConnectionPtr& connection);
  const int getParentConnectionsSize() const;
  const int getNetParentConnectionsSize() const;
  const int getChildConnectionsSize() const;
  const int getNetChildConnectionsSize() const;
  ConnectionPtr parentConnection(const int& i) const;
  ConnectionPtr netParentConnection(const int& i) const;
  ConnectionPtr childConnection(const int& i) const;
  ConnectionPtr netChildConnection(const int& i) const;
  std::vector<NodePtr> getChildren() const;
  std::vector<NodePtr> getParents() const;
  std::vector<NodePtr> getNetParents() const;
  std::vector<NodePtr> getNetChildren() const;
  const std::vector<NodePtr> getChildrenConst() const;
  const std::vector<NodePtr> getParentsConst() const;
  const std::vector<NodePtr> getNetParentsConst() const;
  const std::vector<NodePtr> getNetChildrenConst() const;
  std::vector<ConnectionPtr> getParentConnections() const;
  std::vector<ConnectionPtr> getNetParentConnections() const;
  std::vector<ConnectionPtr> getChildConnections() const;
  std::vector<ConnectionPtr> getNetChildConnections() const;
  const std::vector<ConnectionPtr> getParentConnectionsConst() const;
  const std::vector<ConnectionPtr> getNetParentConnectionsConst() const;
  const std::vector<ConnectionPtr> getChildConnectionsConst() const;
  const std::vector<ConnectionPtr> getNetChildConnectionsConst() const;

  void disconnect();
  void disconnectChildConnections();
  void disconnectParentConnections();
  void disconnectNetParentConnections();
  void disconnectNetChildConnections();
  void removeParentConnection(const ConnectionPtr& connection);
  void removeChildConnection(const ConnectionPtr& connection);
  void removeNetParentConnection(const ConnectionPtr& connection);
  void removeNetChildConnection(const ConnectionPtr& connection);
  bool switchParentConnection(const ConnectionPtr& net_connection);

  const Eigen::VectorXd& getConfiguration()
  {
    return configuration_;
  }
  ~Node();

  XmlRpc::XmlRpcValue toXmlRpcValue() const;
  friend std::ostream& operator<<(std::ostream& os, const Node& path);

  static NodePtr fromXmlRpcValue(const XmlRpc::XmlRpcValue& x);
};

std::ostream& operator<<(std::ostream& os, const Node& node);

}
