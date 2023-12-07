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

Node::Node(const Eigen::VectorXd &configuration, const cnr_logger::TraceLoggerPtr &logger):
  logger_(logger)
{
  configuration_ = configuration;
  ndof_ = configuration_.size();

  /*insert the defaults in flags_ here*/
}

unsigned int Node::setFlag(const bool flag)
{
  unsigned int idx = flags_.size();
  setFlag(flag,idx);

  return idx;
}

bool Node::setFlag(const int& idx, const bool flag)
{
  if(idx == flags_.size()) //new flag to add
    flags_.push_back(flag);
  else if(idx<flags_.size())  //overwrite an already existing flag
  {
    if(idx<number_reserved_flags_)
    {
      CNR_ERROR(logger_,"can't overwrite a default flag");
      return false;
    }
    else
      flags_[idx] = flag;
  }
  else  //the flag should already exist or you should ask to create a flag at idx = flags_.size()
  {
    CNR_ERROR(logger_,"flags size "<<flags_.size()<<" and you want to set a flag in position "<<idx);
    return false;
  }

  return true;
}

bool Node::getFlag(const int& idx, const bool default_value)
{
  if(idx<flags_.size())
    return flags_[idx];
  else
    return default_value;  //if the value has not been set, return the default value
}

void Node::addParentConnection(const ConnectionPtr &connection)
{
  if(connection->getChild() != pointer())
  {
    CNR_FATAL(logger_,"child of connection is not this node");
    throw std::runtime_error("child of connection is not this node");
  }

  parent_connections_.push_back(connection);
}

void Node::addChildConnection(const ConnectionPtr &connection)
{
  if(connection->getParent() != pointer())
  {
    CNR_FATAL(logger_,"parent of connection is not this node");
    throw std::runtime_error("parent of connection is not this node");
  }

  child_connections_.push_back(connection);
}

void Node::addNetParentConnection(const ConnectionPtr &connection)
{
  if(connection->getChild() != pointer())
  {
    CNR_FATAL(logger_,"child of net connection is not this node");
    throw std::runtime_error("child of net connection is not this node");
  }

  net_parent_connections_.push_back(connection);
}

void Node::addNetChildConnection(const ConnectionPtr &connection)
{
  if(connection->getParent() != pointer())
  {
    CNR_FATAL(logger_,"parent of net connection is not this node");
    throw std::runtime_error("parent of net connection is not this node");
  }

  net_child_connections_.push_back(connection);
}

void Node::removeParentConnection(const ConnectionPtr &connection)
{
  std::vector<ConnectionWeakPtr>::iterator it_conn =
      std::find_if(parent_connections_.begin(), parent_connections_.end(),
                   [&connection](const ConnectionWeakPtr& conn){return connection == conn.lock();});

  if (it_conn == parent_connections_.end())
  {
    CNR_FATAL(logger_,"connection is not in the parent vector");
    throw std::invalid_argument("connection is not in the parent vector");
  }
  else
  {
    ConnectionPtr conn = (*it_conn).lock();

    //Remove connection from this node's parent connections vector
    parent_connections_.erase(it_conn);

    //Set connection as not valid
    conn->flags_[Connection::idx_valid_] = false;

    //Remove connection from parent's child_connections vector
    NodePtr parent = conn->getParent();
    if(parent)
    {
      std::vector<ConnectionPtr>::iterator it_parent = std::find(parent->child_connections_.begin(),parent->child_connections_.end(),conn);

      if(it_parent == parent->child_connections_.end())
      {
       CNR_FATAL(logger_,"connection is not in the child vector");
       throw std::runtime_error("connection is not in the child vector");
      }
      else
      {
        parent->child_connections_.erase(it_parent);
      }
    }
  }
}

void Node::removeNetParentConnection(const ConnectionPtr &connection)
{
  std::vector<ConnectionWeakPtr>::iterator it_conn =
      std::find_if(net_parent_connections_.begin(), net_parent_connections_.end(),
                   [&connection](const ConnectionWeakPtr& conn){return connection == conn.lock();});

  if (it_conn == net_parent_connections_.end())
  {
    CNR_FATAL(logger_,"connection is not in the net parent vector");
  }
  else
  {
    ConnectionPtr conn = (*it_conn).lock();

    //Remove connection from this node's net parent connections vector
    net_parent_connections_.erase(it_conn);

    //Set connection as not valid
    conn->flags_[Connection::idx_valid_] = false;

    //Remove connection from parent's net child connections vector
    NodePtr parent = conn->getParent();
    if(parent)
    {
      std::vector<ConnectionPtr>::iterator it_parent = std::find(parent->net_child_connections_.begin(),parent->net_child_connections_.end(),conn);

      if(it_parent == parent->net_child_connections_.end())
      {
        CNR_FATAL(logger_,"connection is not in the net child vector");
      }
      else
      {
        parent->net_child_connections_.erase(it_parent);
      }
    }
  }
}

void Node::removeChildConnection(const ConnectionPtr &connection)
{
  std::vector<ConnectionPtr>::iterator it_conn = std::find(child_connections_.begin(),child_connections_.end(),connection);

  if (it_conn == child_connections_.end())
  {
    CNR_FATAL(logger_,"connection is not in the child vector");
    throw std::invalid_argument("connection is not in the child vector");
  }
  else
  {
    ConnectionPtr conn = *it_conn;

    //Remove connection from child's parent connections vector
    NodePtr child = conn->getChild();
    if(child)
    {
      std::vector<ConnectionWeakPtr>::iterator it_child = std::find_if(child->parent_connections_.begin(), child->parent_connections_.end(),
                                                                       [&connection](const ConnectionWeakPtr& conn){return connection == conn.lock();});

      if(it_child == child->parent_connections_.end())
      {
        CNR_FATAL(logger_,"connection is not in the parent vector");
      }
      else
      {
        child->parent_connections_.erase(it_child);
      }
    }

    //Remove connection from this node's child connections vector
    child_connections_.erase(it_conn);

    //Set connection as not valid
    conn->flags_[Connection::idx_valid_] = false;
  }
}

void Node::removeNetChildConnection(const ConnectionPtr &connection)
{
  std::vector<ConnectionPtr>::iterator it_conn = std::find(net_child_connections_.begin(),net_child_connections_.end(),connection);

  if (it_conn == net_child_connections_.end())
  {
    CNR_FATAL(logger_,"connection is not in the child vector");
    throw std::invalid_argument("connection is not in the child vector");
  }
  else
  {
    ConnectionPtr conn = *it_conn;

    //Remove connection from child's net parent connections vector
    NodePtr child = conn->getChild();
    if(child)
    {
      std::vector<ConnectionWeakPtr>::iterator it_child = std::find_if(child->net_parent_connections_.begin(), child->net_parent_connections_.end(),
                                                                       [&connection](const ConnectionWeakPtr& conn){return connection == conn.lock();});

      if(it_child == child->net_parent_connections_.end())
      {
        CNR_FATAL(logger_,"connection is not in the net parent vector");
      }
      else
      {
        child->net_parent_connections_.erase(it_child);
      }
    }

    //Remove connection from this node's net child connections vector
    net_child_connections_.erase(it_conn);

    //Set connection as not valid
    conn->flags_[Connection::idx_valid_] = false;
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
      if(conn->getParent())
        conn->getParent()->removeChildConnection(conn);
      else
        throw std::runtime_error("connection has no parent");
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
        conn->getParent()->removeNetChildConnection(conn);
      else
        throw std::runtime_error("connection has no parent");
  }
  net_parent_connections_.clear();
}

void Node::disconnectChildConnections()
{
  for(const ConnectionPtr& conn:child_connections_)
  {
    if (conn)
      if (conn->getChild())
        conn->getChild()->removeParentConnection(conn);
      else
        throw std::runtime_error("connection has no child");
  }
  child_connections_.clear();
}

void Node::disconnectNetChildConnections()
{
  for(const ConnectionPtr& conn:net_child_connections_)
  {
    if (conn)
      if (conn->getChild())
        conn->getChild()->removeNetParentConnection(conn);
      else
        throw std::runtime_error("connection has no child");
  }
  net_child_connections_.clear();
}

bool Node::switchParentConnection(const ConnectionPtr& net_connection)
{
  if(std::find_if(net_parent_connections_.begin(),net_parent_connections_.end(),
                  [&net_connection](const ConnectionWeakPtr& conn){return net_connection == conn.lock();})
     >=net_parent_connections_.end())
  {
    CNR_FATAL(logger_,"it is not a parent net connection of the node!");
    return false;
  }

  assert(net_connection->getChild() == pointer());
  assert(parent_connections_.size() == 1);

  ConnectionPtr parent_connection = parent_connections_.front().lock();
  if(not parent_connection->convertToNetConnection())
  {
    CNR_FATAL(logger_,"parent connection can't be converted into parent net connection!");
    return false;
  }

  if(not net_connection->convertToConnection())
  {
    assert(parent_connections_.size() == 0);
    parent_connection->convertToConnection();
    assert(parent_connections_.size() == 1);

    CNR_FATAL(logger_,"parent net connection can't be converted into parent connection!");
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
  return child_connections_.at(i);
}

ConnectionPtr Node::netChildConnection(const int& i) const
{
  return net_child_connections_.at(i);
}

std::vector<ConnectionPtr> Node::getParentConnections() const
{
  std::vector<ConnectionPtr> v(parent_connections_.size());
  if(v.empty())
    return v;

  //transform into a shared_ptr vector
  std::transform(parent_connections_.begin(),parent_connections_.end(),v.begin(),
                 [](const ConnectionWeakPtr& weakPtr){return weakPtr.lock();});
  return v;
}

std::vector<ConnectionPtr> Node::getNetParentConnections() const
{
  std::vector<ConnectionPtr> v(net_parent_connections_.size());
  if(v.empty())
    return v;

  //transform into a shared_ptr vector
  std::transform(net_parent_connections_.begin(),net_parent_connections_.end(),v.begin(),
                 [](const ConnectionWeakPtr& weakPtr){return weakPtr.lock();});
  return v;
}

std::vector<ConnectionPtr> Node::getChildConnections() const
{
  return child_connections_;
}

std::vector<ConnectionPtr> Node::getNetChildConnections() const
{
  return net_child_connections_;
}

//const std::vector<ConnectionPtr>& Node::getParentConnectionsConst() const
//{
//  return getParentConnections();
//}

//const std::vector<ConnectionPtr>& Node::getNetParentConnectionsConst() const
//{
//  return getNetParentConnections();
//}

//const std::vector<ConnectionPtr>& Node::getChildConnectionsConst() const
//{
//  return getChildConnections();
//}

//const std::vector<ConnectionPtr>& Node::getNetChildConnectionsConst() const
//{
//  return getNetChildConnections();
//}

std::vector<NodePtr> Node::getChildren() const
{
  std::vector<NodePtr> children;
  if (child_connections_.size()==0)
    return children;

  for(const ConnectionPtr& conn:child_connections_)
  {
    assert(conn);
    children.push_back(conn->getChild());
  }
  return children;
}

//const std::vector<NodePtr> Node::getChildrenConst() const
//{
//  return getChildren();
//}

std::vector<NodePtr> Node::getNetChildren() const
{
  std::vector<NodePtr> children;
  if (net_child_connections_.size()==0)
    return children;

  for(const ConnectionPtr& conn:net_child_connections_)
  {
    assert(conn);
    children.push_back(conn->getChild());
  }
  return children;
}

//const std::vector<NodePtr> Node::getNetChildrenConst() const
//{
//  return getNetChildren();
//}

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

//const std::vector<NodePtr> Node::getParentsConst() const
//{
//  return getParents();
//}

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

//XmlRpc::XmlRpcValue Node::toXmlRpcValue() const
//{
//  XmlRpc::XmlRpcValue x;
//  x.setSize(configuration_.size());
//  for (int idx=0;idx<configuration_.size();idx++)
//    x[idx]=configuration_(idx);
//  return x;
//}

//NodePtr Node::fromXmlRpcValue(const XmlRpc::XmlRpcValue& x)
//{
//  if (x.getType()!= XmlRpc::XmlRpcValue::Type::TypeArray)
//  {
//    ROS_ERROR("loading from XmlRpcValue a node, but XmlRpcValue is not an array");
//    return NULL;
//  }
//  Eigen::VectorXd conf(x.size());
//  for (int idx=0;idx<x.size();idx++)
//  {
//    conf(idx)=x[idx];
//  }
//  return std::make_shared<Node>(conf);
//}

unsigned int Node::getReservedFlagsNumber()
{
  return number_reserved_flags_;
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
