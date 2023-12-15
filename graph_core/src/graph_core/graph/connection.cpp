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

#include <graph_core/graph/connection.h>

namespace graph_core
{

Connection::Connection(const NodePtr& parent, const NodePtr& child, const cnr_logger::TraceLoggerPtr &logger, const bool is_net):
  parent_(parent),
  child_(child),
  logger_(logger)
{
  euclidean_norm_ = (child->getConfiguration() - parent->getConfiguration()).norm();
  likelihood_ = 1.0;

  flags_ = {false, is_net, false}; //valid, is_net, recently_checked
  assert(number_reserved_flags_ == flags_.size());
}

unsigned int Connection::setFlag(const bool flag)
{
  unsigned int idx = flags_.size();
  setFlag(flag,idx);

  return idx;
}

bool Connection::setFlag(const int& idx, const bool flag)
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

bool Connection::getFlag(const int& idx, const bool default_value)
{
  if(idx<flags_.size())
    return flags_[idx];
  else
    return default_value;  //if the value has not been set, return the default value
}

void Connection::add(const bool is_net)
{
  flags_[idx_net_] = is_net;
  add();
}

void Connection::add()
{
  if(flags_[idx_valid_])
  {
    CNR_INFO(logger_,"connection already added");
    return;
  }
  else
    flags_[idx_valid_] = true;

  if(flags_[idx_net_])
  {
    getParent()->addNetChildConnection(pointer());
    getChild()->addNetParentConnection(pointer());
  }
  else
  {
    getParent()->addChildConnection(pointer());
    getChild()->addParentConnection(pointer());
  }
}

void Connection::remove()
{
  if(not flags_[idx_valid_])
    return;

  flags_[idx_valid_] = false;
  if(child_)
  {
    if(flags_[idx_net_])
      getChild()->removeNetParentConnection(pointer()); //notifies the connection's parent too
    else
      getChild()->removeParentConnection(pointer());    //notifies the connection's parent too
  }
  else //if the connection can't be removed by the child node, try using the parent node
  {
    CNR_ERROR(logger_,"child already destroied");
    if(not (parent_.expired()))
    {
      if(flags_[idx_net_])
        getParent()->removeNetChildConnection(pointer());
      else
        getParent()->removeChildConnection(pointer());
    }
    else
      CNR_ERROR(logger_,"parent already destroied");
  }
}

Eigen::VectorXd Connection::projectOnConnection(const Eigen::VectorXd& point, double& distance, bool& in_conn, const bool& verbose)
{
  Eigen::VectorXd parent = getParent()->getConfiguration();
  Eigen::VectorXd child  = getChild() ->getConfiguration();

  if(point == parent)
  {
    if(verbose)
      CNR_INFO(logger_,"point is parent");

    in_conn = true;
    distance = 0.0;
    return parent;
  }
  else if(point == child)
  {
    if(verbose)
      CNR_INFO(logger_,"point is child");

    in_conn = true;
    distance = 0.0;
    return child;
  }
  else
  {
    Eigen::VectorXd conn_vector  = child-parent;
    Eigen::VectorXd point_vector = point-parent;

    double conn_length = (conn_vector).norm();
    assert(conn_length>0.0);

    double point_length = (point_vector).norm();
    assert(point_length>0);

    Eigen::VectorXd conn_versor = conn_vector/conn_length;
    double s = point_vector.dot(conn_versor);

    Eigen::VectorXd projection = parent + s*conn_versor;

    distance = (point-projection).norm();
    assert(not std::isnan(distance));
    assert((point-projection).dot(conn_vector)<TOLERANCE);

    ((s>=0.0) && (s<=conn_length))? (in_conn = true):
                                    (in_conn = false);

    if(verbose)
      CNR_INFO(logger_,"in_conn: "<<in_conn<<" dist: "<<distance<<" s: "<<s<<" point_length: "<<point_length<<" conn_length: "<<euclidean_norm_<< " projection: "<<projection.transpose()<<" parent: "<<parent.transpose()<<" child: "<<child.transpose());

    return projection;
  }
}

void Connection::flip()
{
  remove(); // remove connection from parent and child
  NodePtr tmp = child_;
  child_ = parent_.lock();
  parent_ = tmp;
  add();   // add new connection from new parent and child
}

Connection::~Connection()
{
  if(flags_[idx_valid_])
    remove();
}

bool Connection::isParallel(const ConnectionPtr& conn, const double& toll)
{
  if(euclidean_norm_ == 0.0 || conn->norm() == 0.0)
  {
    CNR_ERROR(logger_,"A connection has norm zero");
    throw std::invalid_argument("A connection has norm zero");
    return false;
  }
  // v1 dot v2 = norm(v1)*norm(v2)*cos(angle) if v1 not // v2
  // v1 dot v2 = norm(v1)*norm(v2) if v1 // v2

  double scalar= std::abs((getChild()->getConfiguration()-getParent()->getConfiguration()).dot(
                            conn->getChild()->getConfiguration()-conn->getParent()->getConfiguration()));

  assert(std::abs(euclidean_norm_-(getChild()->getConfiguration()-getParent()->getConfiguration()).norm())<TOLERANCE);

  return (std::abs(scalar-(euclidean_norm_*conn->norm()))<toll);
}

bool Connection::convertToConnection()
{
  if(flags_[idx_net_])
  {
    remove();
    flags_[idx_net_] = false;
    add();
    return true;
  }
  return false;
}

bool Connection::convertToNetConnection()
{
  if(not flags_[idx_net_])
  {
    remove();
    flags_[idx_net_] = true;
    add();
    return true;
  }
  return false;
}

void Connection::changeConnectionType()
{
  if(not convertToConnection())
    convertToNetConnection();
}

std::ostream& operator<<(std::ostream& os, const Connection& connection)
{
  assert(connection.getParent()!= nullptr);
  assert(connection.getChild() != nullptr);

  os << connection.getParent()->getConfiguration().transpose()<<" ("<<
        connection.getParent()<<") --> "<<connection.getChild()->getConfiguration().transpose()<<
        " ("<<connection.getChild()<<")"<<"\ncost: "<<connection.cost_<<" | length: "<<
        connection.euclidean_norm_<<"\nvalid: "<<connection.isValid()<<" | net: "<<
        connection.isNet()<<" | r.c.: "<<connection.isRecentlyChecked();

  if(connection.flags_.size()>Connection::getReservedFlagsNumber())
  {
    os<<"\n";
    for(unsigned int i=Connection::getReservedFlagsNumber();i<connection.flags_.size();i++)
      os<<" | flag"<<(i-Connection::getReservedFlagsNumber())<<": "<<connection.flags_[i];
  }

  return os;
}

}
