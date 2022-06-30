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

#include <graph_core/graph/node.h>

namespace pathplan
{

Node::Node(const Eigen::VectorXd &configuration)
{
  configuration_ = configuration;
  ndof_ = configuration_.size();
  analyzed_ = 0;
  non_optimal_ = 0;
}

void Node::setAnalyzed(const bool &analyzed)
{
  analyzed_ = analyzed;
}
bool Node::getAnalyzed()
{
  return analyzed_;
}

void Node::setNonOptimal(const bool &nonOptimal)
{
  non_optimal_ = nonOptimal;
}
bool Node::getNonOptimal()
{
  return non_optimal_;
}

void Node::addParentConnection(const ConnectionPtr &connection)
{
  assert(connection->getChild() == pointer());
  parent_connections_.push_back(connection);
  assert(parent_connections_.back().lock() == connection);
}

void Node::addChildConnection(const ConnectionPtr &connection)
{
  assert(connection->getParent() == pointer());
  child_connections_.push_back(connection);
  assert(child_connections_.back().lock() == connection);
}

void Node::addNetParentConnection(const ConnectionPtr &connection)
{
  assert(connection->getChild() == pointer());
  net_parent_connections_.push_back(connection);
  assert(net_parent_connections_.back().lock() == connection);
}

void Node::addNetChildConnection(const ConnectionPtr &connection)
{
  assert(connection->getParent() == pointer());
  net_child_connections_.push_back(connection);
  assert(net_child_connections_.back().lock() == connection);
}

void Node::remoteParentConnection(const ConnectionPtr &connection)
{
  std::vector<ConnectionWeakPtr>::iterator it =
      std::find_if(parent_connections_.begin(), parent_connections_.end(),
                   [&connection](const ConnectionWeakPtr& conn){return connection == conn.lock();});

  if (it == parent_connections_.end())
  {
    ROS_FATAL("connection is not in the parent vector");
    //    throw std::invalid_argument("connection is not in the parent vector");
  }
  else
  {
    parent_connections_.erase(it);
  }
}

void Node::remoteNetParentConnection(const ConnectionPtr &connection)
{
  std::vector<ConnectionWeakPtr>::iterator it =
      std::find_if(net_parent_connections_.begin(), net_parent_connections_.end(),
                   [&connection](const ConnectionWeakPtr& conn){return connection == conn.lock();});

  if (it == net_parent_connections_.end())
  {
    ROS_FATAL("connection is not in the net parent vector");
  }
  else
  {
    net_parent_connections_.erase(it);
  }
}

void Node::remoteChildConnection(const ConnectionPtr &connection)
{
  std::vector<ConnectionWeakPtr>::iterator it =
      std::find_if(child_connections_.begin(), child_connections_.end(),
                   [&connection](const ConnectionWeakPtr& conn){return connection == conn.lock();});

  if (it == child_connections_.end())
  {
    ROS_FATAL("connection is not in the child vector");
    //  throw std::invalid_argument("connection is not in the child vector");
  }
  else
  {
    child_connections_.erase(it);
  }
}

void Node::remoteNetChildConnection(const ConnectionPtr &connection)
{
  std::vector<ConnectionWeakPtr>::iterator it =
      std::find_if(net_child_connections_.begin(), net_child_connections_.end(),
                   [&connection](const ConnectionWeakPtr& conn){return connection == conn.lock();});

  if (it == net_child_connections_.end())
  {
    ROS_FATAL("connection is not in the child vector");
    //  throw std::invalid_argument("connection is not in the child vector");
  }
  else
  {
    net_child_connections_.erase(it);
  }
}

void Node::disconnect()
{
  disconnectParentConnections();
  disconnectNetParentConnections();
  disconnectChildConnections();
  disconnectNetChildConnections();
}

void Node::disconnectParentConnections()
{
  ConnectionPtr conn;
  for (unsigned int i=0;i<parent_connections_.size();i++)
  {
    conn = parent_connections_[i].lock();
    if(conn)
      if (conn->getParent())
        conn->getParent()->remoteChildConnection(conn);
  }
  parent_connections_.clear();
}

void Node::disconnectNetParentConnections()
{
  ConnectionPtr conn;
  for (unsigned int i=0;i<net_parent_connections_.size();i++)
  {
    conn = net_parent_connections_[i].lock();
    if (conn)
      if (conn->getParent())
        conn->getParent()->remoteNetChildConnection(conn);
  }
  net_parent_connections_.clear();
}

void Node::disconnectChildConnections()
{
  ConnectionPtr conn;
  for (unsigned int i=0;i<child_connections_.size();i++)
  {
    conn = child_connections_[i].lock();
    if (conn)
      if (conn->getChild())
        conn->getChild()->remoteParentConnection(conn);
  }
  child_connections_.clear();
}

void Node::disconnectNetChildConnections()
{
  ConnectionPtr conn;
  for (unsigned int i=0;i<net_child_connections_.size();i++)
  {
    conn = net_child_connections_[i].lock();
    if (conn)
      if (conn->getChild())
        conn->getChild()->remoteNetParentConnection(conn);
  }
  net_child_connections_.clear();
}

bool Node::switchParentConnection(const ConnectionPtr& net_connection)
{
  if(std::find_if(net_parent_connections_.begin(),net_parent_connections_.end(),
                  [&net_connection](const ConnectionWeakPtr& conn){return net_connection == conn.lock();})
     >=net_parent_connections_.end())
  {
    ROS_ERROR("it is not a parent net connection of the node!");
    return false;
  }

  assert(net_connection->getChild() == pointer());
  assert(parent_connections_.size() == 1);

  ConnectionPtr parent_connection = parent_connections_.front().lock();
  if(not parent_connection->convertToNetConnection())
  {
    ROS_ERROR("parent connection can't be converted into parent net connection!");
    return false;
  }

  if(not net_connection->convertToConnection())
  {
    assert(parent_connections_.size() == 0);
    parent_connection->convertToConnection();
    assert(parent_connections_.size() == 1);

    ROS_ERROR("parent net connection can't be converted into parent connection!");
    return false;
  }

  assert(parent_connections_.size() == 1);
  return true;
}

Node::~Node()
{
  disconnect();
}
const int Node::getParentConnectionsSize() const
{
  return parent_connections_.size();
}
const int Node::getNetParentConnectionsSize() const
{
  return net_parent_connections_.size();
}
const int Node::getChildConnectionsSize() const
{
  return child_connections_.size();
}
const int Node::getNetChildConnectionsSize() const
{
  return net_child_connections_.size();
}

ConnectionPtr Node::parentConnection(const int& i) const
{
  return parent_connections_.at(i).lock();
}

ConnectionPtr Node::netParentConnection(const int& i) const
{
  return net_parent_connections_.at(i).lock();
}

ConnectionPtr Node::childConnection(const int& i) const
{
  return child_connections_.at(i).lock();
}

ConnectionPtr Node::netChildConnection(const int& i) const
{
  return net_child_connections_.at(i).lock();
}

std::vector<ConnectionPtr> Node::getParentConnections() const
{
  std::vector<ConnectionPtr> v(parent_connections_.size());

  //transform into a shared_ptr vector
  std::transform(parent_connections_.begin(),parent_connections_.end(),v.begin(),
                 [](const ConnectionWeakPtr& weakPtr){return weakPtr.lock();});
  return v;
}

std::vector<ConnectionPtr> Node::getNetParentConnections() const
{
  std::vector<ConnectionPtr> v(net_parent_connections_.size());

  //transform into a shared_ptr vector
  std::transform(net_parent_connections_.begin(),net_parent_connections_.end(),v.begin(),
                 [](const ConnectionWeakPtr& weakPtr){return weakPtr.lock();});
  return v;
}

std::vector<ConnectionPtr> Node::getChildConnections() const
{
  std::vector<ConnectionPtr> v(child_connections_.size());

  //transform into a shared_ptr vector
  std::transform(child_connections_.begin(),child_connections_.end(),v.begin(),
                 [](const ConnectionWeakPtr& weakPtr){return weakPtr.lock();});
  return v;
}

std::vector<ConnectionPtr> Node::getNetChildConnections() const
{
  std::vector<ConnectionPtr> v(net_child_connections_.size());

  //transform into a shared_ptr vector
  std::transform(net_child_connections_.begin(),net_child_connections_.end(),v.begin(),
                 [](const ConnectionWeakPtr& weakPtr){return weakPtr.lock();});
  return v;
}

const std::vector<ConnectionPtr> Node::getParentConnectionsConst() const
{
  return getParentConnections();
}

const std::vector<ConnectionPtr> Node::getNetParentConnectionsConst() const
{
  return getNetParentConnections();
}

const std::vector<ConnectionPtr> Node::getChildConnectionsConst() const
{
  return getChildConnections();
}

const std::vector<ConnectionPtr> Node::getNetChildConnectionsConst() const
{
  return getNetChildConnections();
}

std::vector<NodePtr> Node::getChildren() const
{
  std::vector<NodePtr> children;
  if (child_connections_.size()==0)
    return children;

  ConnectionPtr conn;
  for(unsigned int i=0; i<child_connections_.size();i++)
  {
    conn = child_connections_[i].lock();
    assert(conn);
    children.push_back(conn->getChild());
  }
  return children;
}

const std::vector<NodePtr> Node::getChildrenConst() const
{
  return getChildren();
}

std::vector<NodePtr> Node::getNetChildren() const
{
  std::vector<NodePtr> children;
  if (net_child_connections_.size()==0)
    return children;

  ConnectionPtr conn;
  for(unsigned int i=0; i<net_child_connections_.size();i++)
  {
    conn = net_child_connections_[i].lock();
    assert(conn);
    children.push_back(conn->getChild());
  }
  return children;
}

const std::vector<NodePtr> Node::getNetChildrenConst() const
{
  return getNetChildren();
}

std::vector<NodePtr> Node::getParents() const
{
  std::vector<NodePtr> parents;
  if (parent_connections_.size()==0)
    return parents;

  ConnectionPtr conn;
  for(unsigned int i=0; i<parent_connections_.size();i++)
  {
    conn = parent_connections_[i].lock();
    assert(conn);
    parents.push_back(conn->getParent());
  }
  return parents;
}

const std::vector<NodePtr> Node::getParentsConst() const
{
  return getParents();
}

std::vector<NodePtr> Node::getNetParents() const
{
  std::vector<NodePtr> parents;
  if (net_parent_connections_.size()==0)
    return parents;

  ConnectionPtr conn;
  for(unsigned int i=0; i<net_parent_connections_.size();i++)
  {
    conn = net_parent_connections_[i].lock();
    assert(conn);
    parents.push_back(conn->getParent());
  }
  return parents;
}

const std::vector<NodePtr> Node::getNetParentsConst() const
{
  return getNetParents();
}

XmlRpc::XmlRpcValue Node::toXmlRpcValue() const
{
  XmlRpc::XmlRpcValue x;
  x.setSize(configuration_.size());
  for (int idx=0;idx<configuration_.size();idx++)
    x[idx]=configuration_(idx);
  return x;
}

NodePtr Node::fromXmlRpcValue(const XmlRpc::XmlRpcValue& x)
{
  if (x.getType()!= XmlRpc::XmlRpcValue::Type::TypeArray)
  {
    ROS_ERROR("loading from XmlRpcValue a node, but XmlRpcValue is not an array");
    return NULL;
  }
  Eigen::VectorXd conf(x.size());
  for (int idx=0;idx<x.size();idx++)
  {
    conf(idx)=x[idx];
  }
  return std::make_shared<Node>(conf);
}

std::ostream& operator<<(std::ostream& os, const Node& node)
{
  os << "configuration = " << node.configuration_.transpose() << std::endl;
  os << "parent connections = " << node.parent_connections_.size() << std::endl;
  os << "child connections = " << node.child_connections_.size() << std::endl;
  os << "net parent connections = " << node.net_parent_connections_.size() << std::endl;
  os << "net child connections = " << node.net_child_connections_.size() << std::endl;

  return os;
}
}
