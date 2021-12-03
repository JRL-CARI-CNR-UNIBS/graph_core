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
#include <graph_core/graph/tree.h>
#include <graph_core/graph/graph_display.h>

namespace pathplan
{
class Net;
typedef std::shared_ptr<Net> NetPtr;

class Net: public std::enable_shared_from_this<Net>
{
protected:
  TreePtr linked_tree_;
  DisplayPtr disp_;

  std::multimap<double,std::vector<ConnectionPtr>> computeConnectionFromNodeToNode(const NodePtr &start_node, const NodePtr &goal_node, std::vector<NodePtr>& visited_nodes);

public:
  Net(const TreePtr& tree): linked_tree_(tree){}

  void setDisp(const DisplayPtr& disp)
  {
    disp_ = disp;
  }

  void setTree(const TreePtr& tree)
  {
    linked_tree_ = tree;
  }

  TreePtr getTree()
  {
    return linked_tree_;
  }

  std::multimap<double,std::vector<ConnectionPtr>> getConnectionToNode(const NodePtr& node);
  std::multimap<double,std::vector<ConnectionPtr>> getConnectionBetweenNodes(const NodePtr& start_node, const NodePtr& goal_node);
  std::multimap<double,std::vector<ConnectionPtr>> getNetConnectionBetweenNodes(const NodePtr& start_node, const NodePtr& goal_node);
};

}
