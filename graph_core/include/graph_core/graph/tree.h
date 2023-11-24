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

  /**
   * @brief tryExtend attempts to extend the tree to the provided configuration.
   *
   * This function finds the closest existing node in the tree to the given configuration using findClosestNode function,
   * and then attempts to extend the tree from that closest node using tryExtendFromNode function.
   *
   * tryExtend does not create a new node.
   *
   * @param configuration The input configuration to which the function attempts to extend the tree.
   * @param next_configuration A reference to a VectorXd object where the newly generated configuration will be stored if the extension is successful.
   * @param closest_node A reference to a pointer to a Node object, pointing to the closest node in the tree.
   * @return Returns true if the extension is possible, and false otherwise.
   */
  bool tryExtend(const Eigen::VectorXd& configuration,
                 Eigen::VectorXd& next_configuration,
                 NodePtr& closest_node);

  /**
   * @brief tryExtendFromNode attempts to extend the tree from a given node to a new configuration.
   *
   * This function is responsible for trying to extend the tree from a specified node to the provided configuration by selecting a new configuration
   * and checking if the extension is valid based on a given tolerance and collision checking.
   * The new configuration if selected by selectNextConfiguration function. If the distance between the configuration and the closest node is less than max_distance_,
   * the function returns the input configuration. Otherwise it calculates a new configuration at a distance equal to max_distance_
   * from the node and in the direction of the input configuration.
   *
   * tryExtendFromNode does not create a new node.
   *
   * @param configuration The input configuration to which the function attempts to extend the tree.
   * @param next_configuration A reference to a VectorXd object where the newly generated configuration will be stored if the extension is successful.
   * @param node A reference to a pointer to a Node object representing the starting point for the extension.
   * @return Returns true if the extension is possible, and false otherwise.
   */
  bool tryExtendFromNode(const Eigen::VectorXd& configuration,
                         Eigen::VectorXd& next_configuration,
                         NodePtr& node);

  /**
   * @brief tryExtendWithPathCheck attempts to extend the tree to the provided configuration if the whole path to the new configuration is collision-free.
   *
   * This function finds the closest existing node in the tree to the given configuration using findClosestNode function,
   * and then attempts to extend the tree from that closest node using tryExtendFromNodeWithPathCheck function, which checks if the path to the
   * closest node found is collision-free.
   *
   * tryExtendWithPathCheck does not create a new node.
   *
   * @param configuration The input configuration to which the function attempts to extend the tree.
   * @param next_configuration A reference to a VectorXd object where the newly generated configuration will be stored if the extension is successful.
   * @param closest_node A reference to a pointer to a Node object representing the starting point for the extension.
   * @param checked_connections A reference to a vector of ConnectionPtr objects containing the connections checked during the path check.
   * @return Returns true if the extension is possible and the whole path from the root to the new configuration is collisione-free, and false otherwise.
   */
  bool tryExtendWithPathCheck(const Eigen::VectorXd& configuration,
                              Eigen::VectorXd& next_configuration,
                              NodePtr& closest_node,
                              std::vector<ConnectionPtr> &checked_connections);

  /**
   * @brief tryExtendFromNodeWithPathCheck attempts to extend the tree from a given node to the provided configuration if the whole path to the new configuration is collision-free.
   *
   * This function is responsible for trying to extend the tree from a specified node by first checking the path to that node using checkPathToNode function.
   * Note that checkPathToNode checks only connection for which isRecentlyChecked returns false to avoid multiple checks of the same connection.
   * If the path check is successful, it proceeds to select a new configuration using the selectNextConfiguration method
   * and checks if the extension is valid based on a given tolerance and collision checking.
   * If the distance between the configuration and the closest node is less than max_distance_, the function returns the input configuration.
   * Otherwise it calculates a new configuration at a distance equal to max_distance_ from the node and in the direction of the input configuration.
   *
   * tryExtendFromNodeWithPathCheck does not create a new node.
   *
   * @param configuration The input configuration to which the function attempts to extend the tree.
   * @param next_configuration A reference to a VectorXd object where the newly generated configuration will be stored if the extension is successful.
   * @param node A reference to a pointer to a Node object representing the starting point for the extension.
   * @param checked_connections A reference to a vector of ConnectionPtr objects containing the connections checked during the path check.
   * @return Returns true if the extension is possible and the whole path from the root to the new configuration is collisione-free, and false otherwise.
   */
  bool tryExtendFromNodeWithPathCheck(const Eigen::VectorXd& configuration,
                                      Eigen::VectorXd& next_configuration,
                                      NodePtr& node,
                                      std::vector<ConnectionPtr> &checked_connections);

  /**
   * @brief selectNextConfiguration selects the final configuration for tree expansion, which will start from the provided node in the direction of the provided configuration.
   *
   * The function calculates the distance between the input configuration and the current node's configuration using the norm() method.
   * If the calculated distance is below a specified tolerance (TOLERANCE), the next configuration is set to the input configuration.
   * If the distance is less than the maximum allowed distance (max_distance_), the next configuration is set to the input configuration.
   * Otherwise, the next configuration is determined based on linear interpolation between the current node's configuration and the input configuration.
   *
   * @param configuration The input configuration to determine the direction of expansion.
   * @param next_configuration A reference to a VectorXd object where the selected next configuration will be stored.
   * @param node A pointer to a Node object representing the starting node in the tree.
   * @return Returns the distance between the input configuration and the current node's configuration.
   */
  double selectNextConfiguration(const Eigen::VectorXd& configuration,
                                 Eigen::VectorXd& next_configuration,
                                 const NodePtr &node);


  /**
   * @brief extend extends the tree by attempting to add a new node based on the provided configuration.
   *
   * This function tries to extend the tree by finding the closest existing node to the given configuration using the tryExtend method.
   * If the extension is possible (t\ryExtend returns true), the tree is updated with a new node based on the configuration provided by tryExtend and
   * a connection is established between the closest existing node and the new node using the extendOnly function.
   * If the extension is not possible, the tree remains unchanged, and both new_node and connection are set to nullptr.
   *
   * @param configuration The input configuration to which the function attempts to extend the tree.
   * @param new_node A reference to a pointer to a Node object that will be set to the newly created node if the extension is successful.
   * @param connection A reference to a pointer to a Connection object that will be set to the connection between the existing and new nodes.
   * @return Returns true if the extension is successful, and false otherwise.
   */
  bool extend(const Eigen::VectorXd& configuration,
              NodePtr& new_node,
              ConnectionPtr& connection);
  bool extend(const Eigen::VectorXd& configuration,
              NodePtr& new_node);

  /**
   * @brief extendWithPathCheck extends the tree by attempting to add a new node based on the provided configuration, if that the whole path to the new selected configuration is collision-free.
   *
   * This function tries to extend the tree by finding the closest existing node to the given configuration and checking the whole path
   * to the configuration using the tryExtendWithPathCheck method.
   * If the extension is possible (tryExtend returns true), the tree is updated with a new node based on the configuration provided by tryExtendWithPathCheck and
   * a connection is established between the closest existing node and the new node using the extendOnly function. The flag recentlyChecked of the new connection is set true.
   * If the extension is not possible, the tree remains unchanged, and both new_node and connection are set to nullptr.
   *
   * A version wich does not return the connection pointer is also available.
   *
   * @param configuration The input configuration to which the function attempts to extend the tree.
   * @param new_node A reference to a pointer to a Node object that will be set to the newly created node if the extension is successful.
   * @param connection A reference to a pointer to a Connection object that will be set to the connection between the existing and new nodes.
   * @param checked_connections A reference to a vector of ConnectionPtr objects containing the connections checked during the path check.
   * @return Returns true if the extension is successful, and false otherwise.
   */
  bool extendWithPathCheck(const Eigen::VectorXd& configuration,
                           NodePtr& new_node,
                           ConnectionPtr& connection,
                           std::vector<ConnectionPtr> &checked_connections);
  bool extendWithPathCheck(const Eigen::VectorXd& configuration,
                           NodePtr& new_node,
                           std::vector<ConnectionPtr> &checked_connections);

  /**
   * @brief extendOnly extends the tree to a newly created node by establishing a connection with the closest existing node.
   *
   * This function extends the tree by creating a connection between the new node and the closest existing node, both provided by input.
   * The cost of the connection is calculated using the specified metrics, and the new node is added to the tree.
   *
   * @param closest_node A reference to a pointer to a Node object representing the closest existing node.
   * @param new_node A reference to a pointer to a Node object representing the new node.
   * @param connection A reference to a pointer to a Connection object that will be set to the connection between the two nodes.
   * @return Returns true if the extension is successful, and false otherwise.
   */
  bool extendOnly(NodePtr &closest_node,
                  NodePtr& new_node,
                  ConnectionPtr& connection);

  /**
   * @brief extendToNode extends the tree to include a specified node.
   *
   * This function extends the tree to include the given node. If the node is already in the tree, the function returns true.
   * Otherwise, it attempts to extend the tree to the node's configuration using the tryExtend method.
   * If the extension is successful, a new node is created, and a connection is established between the closest existing node and the new node.
   * The cost of the connection is calculated using the specified metrics.
   *
   * @param node A reference to a pointer to a Node object representing the node to be included in the tree.
   * @param new_node A reference to a pointer to a Node object representing the newly created node or the existing node if it is already in the tree.
   * @return Returns true if the tree is extended to include the node, and false otherwise.
   */
  bool extendToNode(const NodePtr& node,
                    NodePtr& new_node);

  /**
   * @brief Connect connects the tree to a specified configuration by repeatedly extending towards that configuration.
   *
   * This function connects the tree to a specified configuration by iteratively extending the tree towards that configuration.
   * The extension is performed using the extend method. If the extension is successful, the new node is updated, and the function
   * checks if the distance between the new node's configuration and the specified configuration is within tolerance. If so, the connection is considered successful.
   * The process continues until no further extension is possible or until the distance is within tolerance.
   *
   * @param configuration The target configuration to which the tree tries to extend.
   * @param new_node A reference to a pointer to a Node object representing the last node created by the extend function, namely the farthest from the closest node of the tree
   * and the closest one to node.
   * @return Returns true if the tree is successfully connected to the specified configuration, and false otherwise.
   */
  bool connect(const Eigen::VectorXd& configuration,
               NodePtr& new_node);

  /**
   * @brief informedExtend implements an informed extension method that prioritizes nodes with lower heuristic values.
   *
   * This function attempts to extend the tree from the closest nodes to a specified configuration, prioritizing nodes with lower heuristic values.
   * The heuristic is calculated as a weighted sum of the distance between the node and the configuration and the cost-to-come to reach the node from start.
   * Specifically, the heuristic is computed as:
   *          heuristic = bias*distance between node and configuration + (1-bias)*cost to the node from root
   * Bias is an input value between 0 and 1.
   * The function iterates through the closest nodes, calculates the heuristic, and adds nodes with a favorable heuristic to a priority queue.
   * It then attempts to extend the tree from the selected nodes and checks for successful extensions based on distance and path validity.
   * A close node is added to the queue if and only if the following condition is true:
   *
   *          cost to the node + distance between node and configuration + distance between configuration and goal < cost to beat
   *
   * @param configuration The target configuration to which the tree is extended.
   * @param new_node A reference to a pointer to a Node object representing the newly created node if the extension is successful.
   * @param goal The goal is the final configuration of the path planning problem. It is used in the cost-to-go calculation.
   * @param cost2beat The value used in the condition above.
   * @param bias The bias factor determining the weights in the heuristic calculation. It belongs to the interval [0,1]
   * @return Returns true if the tree is successfully extended to the specified configuration, and false otherwise.
   */
  bool informedExtend(const Eigen::VectorXd& configuration,   //Used in AnytimeRRT
                      NodePtr& new_node,
                      const Eigen::VectorXd &goal, const double &cost2beat, const double &bias);

  /**
   * @brief connectToNode connects the tree to a specified node.
   *
   * This function connects the tree to a specified node by repeatedly extending the tree towards that node.
   * The extension is performed using the extendToNode method. If the extension is successful, the new node is updated,
   * and the function checks if the distance between the new node's configuration and the specified node's configuration is within tolerance.
   * The process continues until no further extension is possible, the distance is within tolerance, or the maximum time limit is reached.
   *
   * @param node A reference to a pointer to a Node object representing the node to which the tree is connected.
   * @param new_node A reference to a pointer to a Node object representing the last node created by the extend function, namely the farthest from the closest node of the tree
   * and the closest one to node.
   * @param max_time The maximum time limit (in seconds) for attempting the connection.
   * @return Returns true if the tree is successfully connected to the specified node, and false otherwise.
   */
  bool connectToNode(const NodePtr& node,
                     NodePtr& new_node,
                     const double &max_time = std::numeric_limits<double>::infinity());

  /**
   * @brief rewireOnly rewires the tree by potentially updating parent and/or child connections of a specified node.
   *
   * This function attempts to rewire the tree by potentially updating the parent and/or child connections of a specified node.
   * The rewire operation is influenced by a given radius and a white list of nodes that should not be rewired. If a node belongs to the white list,
   * a new parent for that node is not searched. By setting the rewiring radius <=0 the nearest nodes considered are the K nearest neighbours.
   * The what_rewire parameter determines whether to rewire only parents (1), only children (2), or both (0).
   *
   * @param node A reference to a pointer to a Node object representing the node to be rewired.
   * @param r_rewire The radius within which nodes are considered for rewiring. If <= 0, the nearest nodes are the K nearest neighbours.
   * @param white_list A vector of NodePtr representing nodes that should not be considered for rewiring.
   * @param what_rewire An integer specifying the rewire operation: 0 for for both parents and children, 1 for parents only, and 2 for children only.
   * @return Returns true if the rewire operation results in improved connections, and false otherwise.
   */
  bool rewireOnly(NodePtr& node, double r_rewire, const std::vector<NodePtr> &white_list, const int &what_rewire = 0);
  bool rewireOnly(NodePtr& node, double r_rewire, const int &what_rewire = 0);

  /**
   * @brief rewireOnlyWithPathCheck rewires the tree by potentially updating parent and/or child connections of a specified node with path checking.
   *
   * This function attempts to rewire the tree by potentially updating the parent and/or child connections of a specified node.
   * The rewire operation is influenced by a given radius and a white list of nodes that should not be rewired. If a node belongs to the white list,
   * a new parent for that node is not searched. By setting the rewiring radius <=0 the nearest nodes considered are the K nearest neighbours.
   * The what_rewire parameter determines whether to rewire only parents (1), only children (2), or both (0).
   * Path checking is performed to ensure the validity of connections during the rewire operation, e.g. the validity of the path to the new potential parent of the node.
   *
   * @param node A reference to a pointer to a Node object representing the node to be rewired.
   * @param checked_connections A vector of ConnectionPtr representing the connections that have been checked for validity.
   * @param r_rewire The radius within which nodes are considered for rewiring. If <= 0, the nearest nodes are the K nearest neighbours.
   * @param white_list A vector of NodePtr representing nodes that should not be considered for rewiring.
   * @param what_rewire An integer specifying the rewire operation: 0 for parents and children only, 1 for parents only, and 2 for children only.
   * @return Returns true if the rewire operation results in improved connections, and false otherwise.
   */
  bool rewireOnlyWithPathCheck(NodePtr& node, std::vector<ConnectionPtr> &checked_connections, double r_rewire, const std::vector<NodePtr> &white_list, const int& what_rewire = 0);
  bool rewireOnlyWithPathCheck(NodePtr& node, std::vector<ConnectionPtr> &checked_connections, double r_rewire, const int& what_rewire = 0);

  /**
   * @brief rewire rewires the tree after extending it from a given configuration within a specified radius.
   *
   * This function extends the tree from a given configuration using the extend method.
   * If the extension is successful, it then attempts to rewire the tree around the new node using the rewireOnly method.
   *
   * @param configuration A reference to a constant Eigen::VectorXd representing the configuration from which the tree is extended.
   * @param r_rewire The radius within which nodes are considered for rewiring during the rewire operation. If <= 0, the nearest nodes are the K nearest neighbours.
   * @param new_node A reference to a pointer to a Node object representing the new node added to the tree during extension.
   * @return Returns true if the extension and rewire operations are successful, and false otherwise.
   */
  bool rewire(const Eigen::VectorXd& configuration,
              double r_rewire,
              NodePtr& new_node);
  bool rewire(const Eigen::VectorXd& configuration,
              double r_rewire);

  /**
   * @brief rewireWithPathCheck rewires the tree after extending it to a given configuration with path checking.
   *
   * This function extends the tree from a given configuration using the extendWithPathCheck method.
   * If the extension is successful, it then attempts to rewire the tree around the new node with path checking using the rewireOnlyWithPathCheck method.
   *
   * @param configuration A reference to a constant Eigen::VectorXd representing the configuration to which the tree is extended.
   * @param checked_connections A vector of ConnectionPtr representing the connections that have been checked for validity.
   * @param r_rewire The radius within which nodes are considered for rewiring during the rewire operation. If <= 0, the nearest nodes are the K nearest neighbours.
   * @param white_list A vector of NodePtr representing nodes that should not be considered for rewiring.
   * @param new_node A reference to a pointer to a Node object representing the new node added to the tree during extension.
   * @return Returns true if the extension and rewire operations are successful, and false otherwise.
   */
  bool rewireWithPathCheck(const Eigen::VectorXd& configuration,
                           std::vector<ConnectionPtr> &checked_connections,
                           double r_rewire,
                           const std::vector<NodePtr> &white_list,
                           NodePtr& new_node);
  bool rewireWithPathCheck(const Eigen::VectorXd& configuration,
                           std::vector<ConnectionPtr> &checked_connections,
                           double r_rewire,
                           NodePtr& new_node);
  bool rewireWithPathCheck(const Eigen::VectorXd& configuration,
                           std::vector<ConnectionPtr> &checked_connections,
                           double r_rewire);

  /**
   * @brief rewireToNode rewires the tree to a specific node after extending it.
   *
   * This function extends the tree to a specific node using the extendToNode method.
   * If the extension is successful, it then attempts to rewire the tree around the new node using the rewireOnly method.
   *
   * @param n A constant reference to a NodePtr representing the target node to which the tree is extended.
   * @param r_rewire The radius within which nodes are considered for rewiring during the rewire operation. If <= 0, the nearest nodes are the K nearest neighbours.
   * @param new_node A reference to a pointer to a Node object representing the new node added to the tree during extension.
   * @return Returns true if the extension and rewire operations are successful, and false otherwise.
   */
  bool rewireToNode(const NodePtr& n,
                    double r_rewire,
                    NodePtr& new_node);
  bool rewireToNode(const NodePtr& n,
                    double r_rewire);

  /**
   * @brief checkPathToNode checks the validity of the path to a specific node.
   *
   * This function checks the validity of the path to a specific node by examining the connections along the path.
   * It first retrieves the connections leading to the node using the getConnectionToNode method.
   * Then, it checks each connection for validity, updating costs and marking connections as checked.
   *
   * @param node A constant reference to a NodePtr representing the target node to which the path is checked.
   * @param checked_connections A vector of ConnectionPtr representing the connections that have been checked for validity.
   * @param path_connections A vector of ConnectionPtr representing the connections along the path to the target node.
   * @return Returns true if the path to the node is valid, and false otherwise.
   */
  bool checkPathToNode(const NodePtr &node, std::vector<ConnectionPtr>& checked_connections, std::vector<ConnectionPtr>& path_connections);
  bool checkPathToNode(const NodePtr &node, std::vector<ConnectionPtr>& checked_connections);

  /**
   * @brief findClosestNode finds the closest existing node in the tree to a given configuration.
   *
   * This function utilizes the nearestNeighbor method of the nodes_ member to find and return the closest existing node in the tree
   * to the specified configuration.
   *
   * @param configuration A constant reference to an Eigen::VectorXd representing the configuration for which the closest node is sought.
   * @return Returns a pointer to the closest existing node in the tree to the specified configuration.
   */
  NodePtr findClosestNode(const Eigen::VectorXd& configuration);

  /**
   * @brief costToNode calculates the cost to reach a specific node from the tree's root.
   *
   * This function calculates the cost to reach a specific node from the tree's root by traversing the tree along its parent connections.
   * The cost is the sum of the costs of all connections along the path to the root.
   *
   * @param node A pointer to a Node object representing the target node for which the cost is calculated.
   * @return Returns the cost to reach the specified node from the tree's root.
   */
  double costToNode(NodePtr node);

  /**
   * @brief getConnectionToNode retrieves the connections along the path to a specific node from the tree's root.
   *
   * This function retrieves the connections along the path to a specific node from the tree's root.
   * It traverses the tree along its parent connections and collects the connections in reverse order.
   *
   * @param node A constant reference to a NodePtr representing the target node for which the connections are retrieved.
   * @return Returns a vector of ConnectionPtr representing the connections along the path from the root to the specified node.
   */
  std::vector<ConnectionPtr> getConnectionToNode(const NodePtr& node);

  /**
   * @brief Keeps only the branch specified by a vector of connections in the tree.
   *
   * This function retains only the branch specified by a vector of connections in the tree.
   * It disconnects the nodes outside the specified branch and updates the tree's nodes_ member accordingly.
   *
   * @param connections A constant reference to a vector of ConnectionPtr representing the connections specifying the branch to be retained.
   * @return Returns true if the operation is successful, and false otherwise.
   */
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
