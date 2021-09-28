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
#include <graph_core/collision_checker.h>
#include <graph_core/sampler.h>
#include <graph_core/metrics.h>

namespace pathplan
{

class Tree;
typedef std::shared_ptr<Tree> TreePtr;

class Tree: public std::enable_shared_from_this<Tree>
{
protected:
  NodePtr root_;
  Direction direction_;
  double max_distance_=1;
  double tolerance_ = 1e-6;
  double k_rrt_;
  unsigned int maximum_nodes_ = 5000; // legare il massimo numero di punti al volume????
  CollisionCheckerPtr checker_;
  MetricsPtr metrics_;

  std::vector<NodePtr> nodes_;

  void purgeNodeOutsideEllipsoid(NodePtr& node,
                                 const SamplerPtr& sampler,
                                 const std::vector<NodePtr>& white_list,
                                 unsigned int& removed_nodes);

  void purgeNodeOutsideEllipsoids(NodePtr& node,
                                  const std::vector<SamplerPtr>& samplers,
                                  const std::vector<NodePtr>& white_list,
                                  unsigned int& removed_nodes);

  // add children(Forward direction) or parents (Backforward direction) to the tree. node is not added (throw exception if it is not member of the tree)
  void populateTreeFromNode(const NodePtr& node);
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Tree(const NodePtr& root,
       const Direction& direction,
       const double& max_distance,
       const CollisionCheckerPtr& checker,
       const MetricsPtr& metrics);

  const NodePtr& getRoot()
  {
    return root_;
  }
  std::vector<NodePtr> getNodes()
  {
    return nodes_;
  }

  bool changeRoot(const NodePtr& node);

  virtual void addNode(const NodePtr& node, const bool& check_if_present = true);
  virtual void removeNode(const std::vector<NodePtr>::iterator& it);
  virtual void removeNode(const NodePtr& node);

  bool tryExtend(const Eigen::VectorXd& configuration,
                 Eigen::VectorXd& next_configuration,
                 NodePtr& closest_node);

  bool extend(const Eigen::VectorXd& configuration,
              NodePtr& new_node);

  bool extendToNode(const NodePtr& node,
                    NodePtr& new_node);

  bool connect(const Eigen::VectorXd& configuration,
               NodePtr& new_node);

  bool connectToNode(const NodePtr& node,
                     NodePtr& new_node,
                     const double &max_time = std::numeric_limits<double>::infinity());

  bool rewire(const Eigen::VectorXd& configuration,
              double r_rewire);

  bool rewireK(const Eigen::VectorXd& configuration);
  bool rewire(const Eigen::VectorXd& configuration,
              double r_rewire,
              NodePtr& new_node);

  bool rewireToNode(const NodePtr& n,
                    double r_rewire);

  bool rewireToNode(const NodePtr& n,
                    double r_rewire,
                    NodePtr& new_node);

  //if what_rewire is 1 it searches for the best parent for node inside the radius r_rewire, if 2 it verifies if node is a better parent for the other nodes inside r_rewire, if 0 it does both
  bool rewireOnly(NodePtr& node, double r_rewire, const int &what_rewire = 0);

  NodePtr findClosestNode(const Eigen::VectorXd& configuration);

  double costToNode(NodePtr node);

  std::vector<ConnectionPtr> getConnectionToNode(NodePtr node);

  bool keepOnlyThisBranch(const std::vector<ConnectionPtr>& connections);

  bool addBranch(const std::vector<ConnectionPtr>& connections);
  bool addTree(TreePtr& additional_tree, const double &max_time = std::numeric_limits<double>::infinity());
  void cleanTree();
  std::vector<NodePtr> near(const NodePtr& node, const double& r_rewire);
  std::map<double, NodePtr> nearK(const NodePtr& node);

  bool isInTree(const NodePtr& node);
  bool isInTree(const NodePtr& node, std::vector<NodePtr>::iterator& it);
  unsigned int getNumberOfNodes()const
  {
    return nodes_.size();
  }

  unsigned int purgeNodesOutsideEllipsoid(const SamplerPtr& sampler, const std::vector<NodePtr>& white_list);
  unsigned int purgeNodesOutsideEllipsoids(const std::vector<SamplerPtr>& samplers, const std::vector<NodePtr>& white_list);
  unsigned int purgeNodes(const SamplerPtr& sampler, const std::vector<NodePtr>& white_list, const bool check_bounds = true);
  bool purgeFromHere(NodePtr& node, const std::vector<NodePtr>& white_list, unsigned int& removed_nodes);
  bool needCleaning(){return nodes_.size()>maximum_nodes_;}

  bool recheckCollision(); //return true if there are no collisions
  bool recheckCollisionFromNode(NodePtr &n); //return true if there are no collisions

  const double& getMaximumDistance() const {return max_distance_;}
  const Direction& getDirection() const {return direction_;}
  MetricsPtr& getMetrics() {return metrics_;}
  CollisionCheckerPtr& getChecker() {return checker_;}

  XmlRpc::XmlRpcValue toXmlRpcValue() const;
  friend std::ostream& operator<<(std::ostream& os, const Tree& tree);

  static TreePtr fromXmlRpcValue(const XmlRpc::XmlRpcValue& x,
                                 const Direction& direction,
                                 const double& max_distance,
                                 const CollisionCheckerPtr& checker,
                                 const MetricsPtr& metrics,
                                 const bool& lazy=false);
};

std::ostream& operator<<(std::ostream& os, const Tree& tree);


}
