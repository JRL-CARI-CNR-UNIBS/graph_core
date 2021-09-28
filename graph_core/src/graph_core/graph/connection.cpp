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

#include <graph_core/graph/connection.h>

namespace pathplan
{

Connection::Connection(const NodePtr &parent, const NodePtr &child):
  parent_(parent),
  child_(child)
{
  euclidean_norm_ = (child->getConfiguration() - parent->getConfiguration()).norm();
}

ConnectionPtr Connection::clone()
{
  NodePtr new_parent = std::make_shared<Node>(parent_->getConfiguration());
  NodePtr new_child = std::make_shared<Node>(child_->getConfiguration());

  ConnectionPtr new_connection = std::make_shared<Connection>(new_parent,new_child);
  new_connection->setCost(cost_);
  new_connection->add();
  return new_connection;
}

void Connection::add()
{
  valid = true;
  parent_->addChildConnection(pointer());
  child_->addParentConnection(pointer());
}
void Connection::remove()
{
  if (!valid)
    return;

  valid = false;
  if (parent_)
  {
    parent_->remoteChildConnection(pointer());
  }
  else
    ROS_FATAL("parent already destroied");

  if (child_)
  {
    child_->remoteParentConnection(pointer());
  }
  else
    ROS_FATAL("child already destroied");

}

void Connection::flip()
{
  remove(); // remove connection from parent and child
  parent_.swap(child_);
  add(); // add new connection from new parent and child
}
Connection::~Connection()
{
}

std::ostream& operator<<(std::ostream& os, const Connection& connection)
{
  os << connection.parent_->getConfiguration().transpose() << " -->" << std::endl;
  os << "-->" << connection.child_->getConfiguration().transpose() << std::endl;

  return os;
}

}
