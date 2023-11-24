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
#include <graph_core/informed_sampler.h>
#include <graph_core/metrics.h>
#include <graph_core/datastructure/nearest_neighbors.h>
#include <graph_core/datastructure/kdtree.h>
#include <graph_core/datastructure/vector.h>

namespace pathplan
{

class Tree;
typedef std::shared_ptr<Tree> TreePtr;

class Tree: public std::enable_shared_from_this<Tree>
{
protected:
  NodePtr root_;
  double k_rrt_;
  bool use_kdtree_;
  double max_distance_ = 1;
  unsigned int maximum_nodes_ = 5000;

  MetricsPtr metrics_;
  NearestNeighborsPtr nodes_;
  CollisionCheckerPtr checker_;

  /**
   * @brief purgeNodeOutsideEllipsoid checks whether a given node falls within the bounds defined by the sampler. If not, it removes the node and its successors.
   * @param node The node to be checked for inclusion within the bounds defined by the sampler.
   * @param sampler It determines whether the node is within the bounds using the inBounds() function.
   * @param white_list A list of nodes exempt from removal. If a node belongs to this vector, it will not be removed, even if it is outside the bounds. If one of its successors is in the white list, it and its predecessors will also be spared from removal.
   * @param removed_nodes The number of successfully removed nodes.
   */
  void purgeNodeOutsideEllipsoid(NodePtr& node,
                                 const InformedSamplerPtr& sampler,
                                 const std::vector<NodePtr>& white_list,
                                 unsigned int& removed_nodes);

  /**
   * @brief purgeNodeOutsideEllipsoid checks whether a given node falls within the bounds defined by a set of samplers. If not, it removes the node and its successors.
   * @param node The node to be checked for inclusion within the bounds defined by the samplers.
   * @param samplers A vector of samplers which determine whether the node is within the bounds using the inBounds() function.
   * @param white_list A list of nodes exempt from removal. If a node belongs to this vector, it will not be removed, even if it is outside the bounds. If one of its successors is in the white list, it and its predecessors will also be spared from removal.
   * @param removed_nodes The number of successfully removed nodes.
   */
  void purgeNodeOutsideEllipsoids(NodePtr& node,
                                  const std::vector<InformedSamplerPtr>& samplers,
                                  const std::vector<NodePtr>& white_list,
                                  unsigned int& removed_nodes);

  /**
   * @brief populateTreeFromNode adds the successors of a given node to the tree. The specified node, although not added itself, must already belong to the tree (e.g., it serves as the root).
   * Successors are included only if the following conditions hold:
   *  1) They do not belong to the black_list.
   *  2) If node_check is true, they are collision-checked; if in collision, they are excluded.
   *  3) They satisfy the condition metrics_->utopia(n->getConfiguration(), focus1) + metrics_->utopia(n->getConfiguration(), focus2) < cost, where n is the considered successor.
   *     In the case of using the Euclidean distance as metrics, this condition ensures that the node belongs to the ellipsoid defined by focus1, focus2 and cost.
   * @param node The successors of this node are added to the tree.
   * @param focus1 The first focus of condition 3).
   * @param focus2 The second focus of condition 3).
   * @param cost The cost of condition 3).
   * @param black_list A list of nodes to be excluded from the tree.
   * @param node_check If true, each successor undergoes a collision check and is added to the tree only if valid.
   */
  void populateTreeFromNode(const NodePtr& node, const Eigen::VectorXd& focus1, const Eigen::VectorXd& focus2, const double& cost, const std::vector<NodePtr> &black_list, const bool node_check = false);
  void populateTreeFromNode(const NodePtr& node, const Eigen::VectorXd& focus1, const Eigen::VectorXd& focus2, const double& cost, const bool node_check = false);
  void populateTreeFromNode(const NodePtr& node, const std::vector<NodePtr>& black_list, const bool node_check = false);
  void populateTreeFromNode(const NodePtr& node, const bool node_check = false);

  /**
   * @brief populateTreeFromNodeConsideringCost adds the successors of a given node to the tree. The specified node, although not added itself, must already belong to the tree (e.g., it serves as the root).
   * Successors are included only if the following conditions hold:
   *  1) They do not belong to the black_list.
   *  2) If node_check is true, they are collision-checked; if in collision, they are excluded.
   *  3) If the cost to reach the node n + metrics_->utopia(n->getConfiguration(), goal) < cost, where n is the considered successor.
   * @param node The successors of this node are added to the tree.
   * @param goal the goal to reach, considered in condition 3).
   * @param cost The cost of condition 3)
   * @param black_list A list of nodes to be excluded from the tree.
   * @param node_check If true, each successor undergoes a collision check and is added to the tree only if valid.
   */
  void populateTreeFromNodeConsideringCost(const NodePtr& node, const Eigen::VectorXd& goal, const double& cost, const std::vector<NodePtr> &black_list, const bool node_check = false);

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Tree(const NodePtr& root,
       const double& max_distance,
       const CollisionCheckerPtr& checker,
       const MetricsPtr& metrics,
       const bool& use_kdtree=true);

  /**
   * @brief isSubtree returs false because it is not a subtree (it is a tree)
   * @return always false
   */
  virtual bool isSubtree()
  {
    return false;
  }

  /**
   * @brief getRoot returns the root node
   * @return the root node
   */
  const NodePtr& getRoot()
  {
    return root_;
  }

  /**
   * @brief getNodes returns a vector containing the nodes of the tree
   * @return the vector of nodes of the tree
   */
  std::vector<NodePtr> getNodes()
  {
    return nodes_->getNodes();
  }

  /**
   * @brief getLeaves populates the input vector with the leaves of the tree, namely the nodes without successors.
   * @param leaves The vector to be filled with the tree's leaves.
   */
  void getLeaves(std::vector<NodePtr>& leaves);

  /**
   * @brief changeRoot sets the specified node as the root of the tree.
   * @param node The node to be set as the new root.
   * @return True if the node is successfully set as the root.
   */
  bool changeRoot(const NodePtr& node);

  /**
   * @brief addNode appends a node to the tree if it is not already included. This function adds the input node to the tree's list of nodes without verifying if the node is connected to another tree node. Ensure a proper connection to this node (unless it is the root) before invoking this function.
   * @param node The node to be added to the tree.
   * @param check_if_present If true, it checks for the node's existing presence in the tree. If already present, it is not added.
   */
  virtual void addNode(const NodePtr& node, const bool& check_if_present = true);

  /**
   * @brief removeNode deletes all connections associated with the node and removes it from the tree's list of nodes.
   * @param node The node to be removed.
   */
  virtual void removeNode(const NodePtr& node);

  bool tryExtend(const Eigen::VectorXd& configuration,
                 Eigen::VectorXd& next_configuration,
                 NodePtr& closest_node);

  bool tryExtendFromNode(const Eigen::VectorXd& configuration,
                         Eigen::VectorXd& next_configuration,
                         NodePtr& node);


  bool tryExtendWithPathCheck(const Eigen::VectorXd& configuration,
                              Eigen::VectorXd& next_configuration,
                              NodePtr& closest_node,
                              std::vector<ConnectionPtr> &checked_connections);

  bool tryExtendFromNodeWithPathCheck(const Eigen::VectorXd& configuration,
                                      Eigen::VectorXd& next_configuration,
                                      NodePtr& node,
                                      std::vector<ConnectionPtr> &checked_connections);

  /* selectNextConfiguration: compute next_configuration as
   * next_configuration = configurarion if configuration is close to the node (less than max_distance_)
   * next_configuration distance is limited to max_distance_ if configuration is far from node
   */
  double selectNextConfiguration(const Eigen::VectorXd& configuration,
                                 Eigen::VectorXd& next_configuration,
                                 const NodePtr &node);

  bool extend(const Eigen::VectorXd& configuration,
              NodePtr& new_node);

  bool extend(const Eigen::VectorXd& configuration,
              NodePtr& new_node,
              ConnectionPtr& connection);


  bool extendWithPathCheck(const Eigen::VectorXd& configuration,
                           NodePtr& new_node,
                           std::vector<ConnectionPtr> &checked_connections);

  bool extendWithPathCheck(const Eigen::VectorXd& configuration,
                           NodePtr& new_node,
                           ConnectionPtr& connection,
                           std::vector<ConnectionPtr> &checked_connections);

  bool extendOnly(NodePtr &closest_node,
                  NodePtr& new_node,
                  ConnectionPtr& connection);

  bool extendToNode(const NodePtr& node,
                    NodePtr& new_node);

  bool connect(const Eigen::VectorXd& configuration,
               NodePtr& new_node);

  bool informedExtend(const Eigen::VectorXd& configuration,   //Used in AnytimeRRT
                      NodePtr& new_node,
                      const Eigen::VectorXd &goal, const double &cost2beat, const double &bias);

  bool connectToNode(const NodePtr& node,
                     NodePtr& new_node,
                     const double &max_time = std::numeric_limits<double>::infinity());

  bool rewire(const Eigen::VectorXd& configuration,
              double r_rewire);

  bool rewireK(const Eigen::VectorXd& configuration);

  //Useful for replanning:  extend+rewireAndCheckPath -> new sample and rewire considering only nodes with path to them collision-free
  bool rewireWithPathCheck(const Eigen::VectorXd& configuration,
                           std::vector<ConnectionPtr> &checked_connections,
                           double r_rewire,
                           NodePtr& new_node);
  bool rewireWithPathCheck(const Eigen::VectorXd& configuration,
                           std::vector<ConnectionPtr> &checked_connections,
                           double r_rewire,
                           const std::vector<NodePtr> &white_list,
                           NodePtr& new_node);

  bool rewireWithPathCheck(const Eigen::VectorXd& configuration,
                           std::vector<ConnectionPtr> &checked_connections,
                           double r_rewire);

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
  bool rewireOnly(NodePtr& node, double r_rewire, const std::vector<NodePtr> &white_list, const int &what_rewire = 0);
  //Useful for replanning: rewireOnly considering those nodes that have a free path from the root to them
  bool rewireOnlyWithPathCheck(NodePtr& node, std::vector<ConnectionPtr> &checked_connections, double r_rewire, const int& what_rewire = 0);
  bool rewireOnlyWithPathCheck(NodePtr& node, std::vector<ConnectionPtr> &checked_connections, double r_rewire, const std::vector<NodePtr> &white_list, const int& what_rewire = 0);

  bool checkPathToNode(const NodePtr &node, std::vector<ConnectionPtr>& checked_connections);
  bool checkPathToNode(const NodePtr &node, std::vector<ConnectionPtr>& checked_connections, std::vector<ConnectionPtr>& path_connections);

  NodePtr findClosestNode(const Eigen::VectorXd& configuration);

  double costToNode(NodePtr node);

  std::vector<ConnectionPtr> getConnectionToNode(NodePtr node);  //la &?

  bool keepOnlyThisBranch(const std::vector<ConnectionPtr>& connections);

  bool addBranch(const std::vector<ConnectionPtr>& connections);
  bool addTree(TreePtr& additional_tree, const double &max_time = std::numeric_limits<double>::infinity());
  void cleanTree();
  std::multimap<double, NodePtr> near(const NodePtr& node, const double& r_rewire);
  std::multimap<double, NodePtr> nearK(const NodePtr& node);
  std::multimap<double, NodePtr> nearK(const Eigen::VectorXd& conf);

  bool isInTree(const NodePtr& node);
  unsigned int getNumberOfNodes()const
  {
    return nodes_->size();
  }

  unsigned int purgeNodesOutsideEllipsoid(const InformedSamplerPtr& sampler, const std::vector<NodePtr>& white_list);
  unsigned int purgeNodesOutsideEllipsoids(const std::vector<InformedSamplerPtr>& samplers, const std::vector<NodePtr>& white_list);
  unsigned int purgeNodes(const InformedSamplerPtr& sampler, const std::vector<NodePtr>& white_list, const bool check_bounds = true);
  virtual void purgeThisNode(NodePtr& node, unsigned int& removed_nodes);
  virtual bool purgeFromHere(NodePtr& node);
  virtual bool purgeFromHere(NodePtr& node, const std::vector<NodePtr>& white_list, unsigned int& removed_nodes);
  bool needCleaning(){return (getNumberOfNodes()>maximum_nodes_);}

  bool recheckCollision(); //return true if there are no collisions
  bool recheckCollisionFromNode(NodePtr &n); //return true if there are no collisions

  const double& getMaximumDistance() const {return max_distance_;}
  MetricsPtr& getMetrics() {return metrics_;}
  CollisionCheckerPtr& getChecker() {return checker_;}
  
  void setChecker(const CollisionCheckerPtr& checker)
  {
    checker_ = checker;
  }

  void setMetrics(const MetricsPtr& metrics)
  {
    metrics_ = metrics;
  }

  bool getUseKdTree(){return use_kdtree_;}

  XmlRpc::XmlRpcValue toXmlRpcValue() const;
  void toXmlFile(const std::string& file_name) const;
  friend std::ostream& operator<<(std::ostream& os, const Tree& tree);

  static TreePtr fromXmlRpcValue(const XmlRpc::XmlRpcValue& x,
                                 const double& max_distance,
                                 const CollisionCheckerPtr& checker,
                                 const MetricsPtr& metrics,
                                 const bool& lazy=false);
};

std::ostream& operator<<(std::ostream& os, const Tree& tree);


}
