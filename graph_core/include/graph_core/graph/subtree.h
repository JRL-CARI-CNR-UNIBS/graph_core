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

#include <graph_core/graph/tree.h>

namespace pathplan
{

class Subtree;
typedef std::shared_ptr<Subtree> SubtreePtr;

class Subtree: public Tree
{
protected:
  TreePtr parent_tree_;

  //Populate subtree with only those node that are inside the ellipsoid defined by focus1, focus2 and cost
  void populateSubtreeInsideEllipsoid(const NodePtr& root,
                                      const Eigen::VectorXd& focus1,
                                      const Eigen::VectorXd& focus2,
                                      const double& cost,
                                      const std::vector<NodePtr> &black_list,
                                      const bool node_check = false);

public:

  Subtree(const TreePtr& parent_tree,
          const NodePtr& root);

  Subtree(const TreePtr& parent_tree,
          const NodePtr& root,
          const std::vector<NodePtr>& black_list);

  Subtree(const TreePtr& parent_tree,
          const NodePtr& root,
          const Eigen::VectorXd& focus1,
          const Eigen::VectorXd& focus2,
          const double& cost);

  Subtree(const TreePtr& parent_tree,
          const NodePtr& root,
          const Eigen::VectorXd& focus1,
          const Eigen::VectorXd& focus2,
          const double& cost,
          const std::vector<NodePtr>& black_list,
          const bool node_check = false);

  Subtree(const TreePtr& parent_tree,
          const NodePtr& root,
          const Eigen::VectorXd& goal,
          const double& cost,
          const std::vector<NodePtr>& black_list,
          const bool node_check = false);

  TreePtr getParentTree()
  {
    return parent_tree_;
  }

  virtual bool isSubtree() override
  {
    return true;
  }

  void purgeThisNode(NodePtr& node, unsigned int& removed_nodes) override;
  void addNode(const NodePtr& node, const bool& check_if_present = true);
  void removeNode(const std::vector<NodePtr>::iterator& it) override;
  void removeNode(const NodePtr& node);

  static SubtreePtr createSubtree(const TreePtr& parent_tree, const NodePtr& root);
  static SubtreePtr createSubtree(const TreePtr& parent_tree, const NodePtr& root,
                                  const std::vector<NodePtr>& black_list);
  static SubtreePtr createSubtree(const TreePtr& parent_tree,
                                  const NodePtr& root,
                                  const Eigen::VectorXd& focus1,
                                  const Eigen::VectorXd& focus2,
                                  const double& cost);
  static SubtreePtr createSubtree(const TreePtr& parent_tree,
                                  const NodePtr& root,
                                  const Eigen::VectorXd& focus1,
                                  const Eigen::VectorXd& focus2,
                                  const double& cost,
                                  const std::vector<NodePtr>& black_list,
                                  const bool node_check = false);
  static SubtreePtr createSubtree(const TreePtr& parent_tree,
                                  const NodePtr& root,
                                  const Eigen::VectorXd& goal,
                                  const double& cost,
                                  const std::vector<NodePtr>& black_list,
                                  const bool node_check = false);

};


}
