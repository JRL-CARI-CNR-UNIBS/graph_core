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

namespace graph
{
namespace core
{
Node::Node(const Eigen::VectorXd& configuration):logger_(nullptr)
{
  configuration_ = configuration;
  ndof_ = configuration_.size();

  /*insert the defaults in flags_ here*/
}

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

bool Node::setFlag(const size_t& idx, const bool flag)
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

bool Node::getFlag(const size_t& idx, const bool default_value)
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

  //Set connection's child as valid
  connection->flags_[Connection::idx_child_valid_] = true;
}

void Node::addChildConnection(const ConnectionPtr &connection)
{
  if(connection->getParent() != pointer())
  {
    CNR_FATAL(logger_,"parent of connection is not this node");
    throw std::runtime_error("parent of connection is not this node");
  }

  child_connections_.push_back(connection);

  //Set connection's parent as valid
  connection->flags_[Connection::idx_parent_valid_] = true;
}

void Node::addNetParentConnection(const ConnectionPtr &connection)
{
  if(connection->getChild() != pointer())
  {
    CNR_FATAL(logger_,"child of net connection is not this node");
    throw std::runtime_error("child of net connection is not this node");
  }

  net_parent_connections_.push_back(connection);

  //Set connection's child as valid
  connection->flags_[Connection::idx_child_valid_] = true;
}

void Node::addNetChildConnection(const ConnectionPtr &connection)
{
  if(connection->getParent() != pointer())
  {
    CNR_FATAL(logger_,"parent of net connection is not this node");
    throw std::runtime_error("parent of net connection is not this node");
  }

  net_child_connections_.push_back(connection);

  //Set connection's parent as valid
  connection->flags_[Connection::idx_parent_valid_] = true;
}

void Node::removeParentConnection(const ConnectionPtr &connection)
{
  std::vector<ConnectionWeakPtr>::iterator it_conn =
      std::find_if(parent_connections_.begin(), parent_connections_.end(),
                   [&connection](const ConnectionWeakPtr& conn){return connection == conn.lock();});

  return removeParentConnection(it_conn);
}

void Node::removeParentConnection(const std::vector<ConnectionWeakPtr>::iterator& it_conn)
{
  if (it_conn == parent_connections_.end())
  {
    CNR_FATAL(logger_,"connection is not in the parent vector");
    throw std::invalid_argument("connection is not in the parent vector");
  }
  else
  {
    //Set connection's child as not valid (before erasing)
    (*it_conn).lock()->flags_[Connection::idx_child_valid_] = false;

    //Remove connection from this node's parent connections vector
    parent_connections_.erase(it_conn);
  }
}

void Node::removeNetParentConnection(const ConnectionPtr &connection)
{
  std::vector<ConnectionWeakPtr>::iterator it_conn =
      std::find_if(net_parent_connections_.begin(), net_parent_connections_.end(),
                   [&connection](const ConnectionWeakPtr& conn){return connection == conn.lock();});

  return removeNetParentConnection(it_conn);
}

void Node::removeNetParentConnection(const std::vector<ConnectionWeakPtr>::iterator &it_conn)
{
  if (it_conn == net_parent_connections_.end())
  {
    CNR_FATAL(logger_,"connection is not in the net parent vector");
    throw std::invalid_argument("connection is not in the net parent vector");
  }
  else
  {
    //Set connection's child as not valid (before erasing)
    (*it_conn).lock()->flags_[Connection::idx_child_valid_] = false;

    //Remove connection from this node's net parent connections vector
    net_parent_connections_.erase(it_conn);
  }
}

void Node::removeChildConnection(const ConnectionPtr &connection)
{
  std::vector<ConnectionPtr>::iterator it_conn = std::find(child_connections_.begin(),child_connections_.end(),connection);

  return removeChildConnection(it_conn);
}

void Node::removeChildConnection(const std::vector<ConnectionPtr>::iterator &it_conn)
{
  if (it_conn == child_connections_.end())
  {
    CNR_FATAL(logger_,"connection is not in the child vector");
    throw std::invalid_argument("connection is not in the child vector");
  }
  else
  {
    //Set connection's parent as not valid (before erasing)
    (*it_conn)->flags_[Connection::idx_parent_valid_] = false;

    //Remove connection from this node's child connections vector
    child_connections_.erase(it_conn);
  }
}

void Node::removeNetChildConnection(const ConnectionPtr &connection)
{
  std::vector<ConnectionPtr>::iterator it_conn = std::find(net_child_connections_.begin(),net_child_connections_.end(),connection);

  return removeNetChildConnection(it_conn);
}

void Node::removeNetChildConnection(const std::vector<ConnectionPtr>::iterator& it_conn)
{
  if (it_conn == net_child_connections_.end())
  {
    CNR_FATAL(logger_,"connection is not in the net child vector");
    throw std::invalid_argument("connection is not in the net child vector");
  }
  else
  {
    //Set connection's parent as not valid (before erasing)
    (*it_conn)->flags_[Connection::idx_parent_valid_] = false;

    //Remove connection from this node's child connections vector
    net_child_connections_.erase(it_conn);
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

  // Use reverse iterator because during the process the vector size decreases
  for(std::vector<ConnectionWeakPtr>::reverse_iterator it = parent_connections_.rbegin(); it != parent_connections_.rend(); ++it)
  {
    conn = (*it).lock();
    if(conn)
      conn->remove();
  }

  assert(parent_connections_.empty());
  //  parent_connections_.clear();
}

void Node::disconnectNetParentConnections()
{
  ConnectionPtr conn;

  // Use reverse iterator because during the process the vector size decreases
  for(std::vector<ConnectionWeakPtr>::reverse_iterator it = net_parent_connections_.rbegin(); it != net_parent_connections_.rend(); ++it)
  {
    conn = (*it).lock();
    if(conn)
      conn->remove();
  }

  assert(net_parent_connections_.empty());
  //  net_parent_connections_.clear();
}

void Node::disconnectChildConnections()
{
  for(std::vector<ConnectionPtr>::reverse_iterator it = child_connections_.rbegin(); it != child_connections_.rend(); ++it)
  {
    if(*it != nullptr)
      (*it)->remove();
  }

  assert(child_connections_.empty());
  //  child_connections_.clear();
}

void Node::disconnectNetChildConnections()
{ 
  for(std::vector<ConnectionPtr>::reverse_iterator it = net_child_connections_.rbegin(); it != net_child_connections_.rend(); ++it)
  {
    if(*it != nullptr)
      (*it)->remove();
  }

  assert(net_child_connections_.empty());
  //  net_child_connections_.clear();
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
  /* Note: cannot use disconnect() because it calls connection->remove()
   * which needs the object's pointer and it is not available at this point
   * since the object is being destroyed. */
  ConnectionPtr conn;
  for(std::vector<ConnectionWeakPtr>::reverse_iterator it = parent_connections_.rbegin(); it != parent_connections_.rend(); ++it)
  {
    conn = (*it).lock();
    this->removeParentConnection(std::next(it).base()); //detach connection. Note: std::next(it).base() to convert reverse iterator into iterator
    conn->getParent()->removeChildConnection(conn); //update connection's parent node

    assert(not conn->flags_[Connection::idx_parent_valid_] && not conn->flags_[Connection::idx_child_valid_]);
  }

  for(std::vector<ConnectionWeakPtr>::reverse_iterator it = net_parent_connections_.rbegin(); it != net_parent_connections_.rend(); ++it)
  {
    conn = (*it).lock();
    this->removeNetParentConnection(std::next(it).base()); //detach connection
    conn->getParent()->removeNetChildConnection(conn); //update connection's parent node

    assert(not conn->flags_[Connection::idx_parent_valid_] && not conn->flags_[Connection::idx_child_valid_]);
  }

  for(std::vector<ConnectionPtr>::reverse_iterator it = child_connections_.rbegin(); it != child_connections_.rend(); ++it)
  {
    conn = *it;
    conn->getChild()->removeParentConnection(conn); //update connection's child node
    this->removeChildConnection(std::next(it).base()); //detach connection

    assert(not conn->flags_[Connection::idx_parent_valid_] && not conn->flags_[Connection::idx_child_valid_]);
  }

  for(std::vector<ConnectionPtr>::reverse_iterator it = net_child_connections_.rbegin(); it != net_child_connections_.rend(); ++it)
  {
    conn = *it;
    conn->getChild()->removeNetParentConnection(conn); //update connection's child node
    this->removeNetChildConnection(std::next(it).base()); //detach connection

    assert(not conn->flags_[Connection::idx_parent_valid_] && not conn->flags_[Connection::idx_child_valid_]);
  }

  assert(parent_connections_.empty() && net_parent_connections_.empty() &&
         child_connections_ .empty() && net_child_connections_ .empty());
}

const size_t Node::getParentConnectionsSize() const
{
  return parent_connections_.size();
}
const size_t Node::getNetParentConnectionsSize() const
{
  return net_parent_connections_.size();
}
const size_t Node::getChildConnectionsSize() const
{
  return child_connections_.size();
}
const size_t Node::getNetChildConnectionsSize() const
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

YAML::Node Node::toYAML() const
{
  YAML::Node yaml;
  yaml.SetStyle(YAML::EmitterStyle::Flow); // Set the style to flow style for a more compact representation

  for (int idx = 0; idx < configuration_.size(); ++idx)
    yaml.push_back(configuration_(idx));

  return yaml;
}

NodePtr Node::fromYAML(const YAML::Node& yaml, const cnr_logger::TraceLoggerPtr& logger)
{
  if (!yaml.IsSequence())
  {
    CNR_ERROR(logger,"Cannot load a node from YAML::Node which does not contain a sequence");
    return nullptr;
  }

  Eigen::VectorXd conf(yaml.size());
  for (size_t idx=0;idx<yaml.size();idx++)
  {
    conf(idx)=yaml[idx].as<double>();
  }

  return std::make_shared<Node>(conf,logger);
}

NodePtr Node::fromYAML(const YAML::Node& yaml, std::string& what)
{
  if (!yaml.IsSequence())
  {
    what += "\nCannot load a node from YAML::Node which does not contain a sequence";
    return nullptr;
  }

  Eigen::VectorXd conf(yaml.size());
  for (size_t idx=0;idx<yaml.size();idx++)
  {
    conf(idx)=yaml[idx].as<double>();
  }

  return std::make_shared<Node>(conf,nullptr);
}

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

template<>
bool get_param<NodePtr>(const cnr_logger::TraceLoggerPtr& logger, const std::string param_ns, const std::string param_name, NodePtr& param)
{
  std::string what, full_param_name = param_ns+"/"+param_name;
  if(cnr::param::has(full_param_name, what))
  {
    YAML::Node yaml_node;
    if(not cnr::param::mapped_file::recover(full_param_name, yaml_node, what))
    {
      CNR_ERROR(logger, "Cannot load " << full_param_name + " parameter.\n"<<what);
      throw std::invalid_argument("Cannot load " + full_param_name + " parameter.");
    }
    else
      param = graph::core::Node::fromYAML(yaml_node,logger);
  }
  else
  {
    CNR_WARN(logger, full_param_name + " parameter not available.\n"<<what);
    return false;
  }
  return true;
}

} //end namespace core
} // end namespace graph
