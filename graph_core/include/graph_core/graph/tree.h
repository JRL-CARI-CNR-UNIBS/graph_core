#pragma once
/*
Copyright (c) 2024, Manuel Beschi and Cesare Tonola, JRL-CARI CNR-STIIMA/UNIBS,
manuel.beschi@unibs.it, c.tonola001@unibs.it All rights reserved.

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

#include <fstream>
#include <graph_core/collision_checkers/collision_checker_base.h>
#include <graph_core/datastructure/kdtree.h>
#include <graph_core/datastructure/nearest_neighbors.h>
#include <graph_core/datastructure/vector.h>
#include <graph_core/metrics/metrics_base.h>
#include <graph_core/samplers/informed_sampler.h>
#include <graph_core/util.h>

namespace graph
{
namespace core
{
/**
 * @class Tree
 * @brief Class for defining a tree.
 */

class MetricsBase;
typedef std::shared_ptr<MetricsBase> MetricsPtr;

class Tree;
typedef std::shared_ptr<Tree> TreePtr;

class Tree : public std::enable_shared_from_this<Tree>
{
protected:
  /**
   * @brief Pointer to the root node of the tree.
   */
  NodePtr root_;

  /**
   * @brief K nearest neighbors.
   */
  double k_rrt_;

  /**
   * @brief Flag indicating whether to use a k-d tree for nearest neighbors
   * search.
   */
  bool use_kdtree_;

  /**
   * @brief Maximum distance for connection attempts in the tree.
   */
  double max_distance_ = 1;

  /**
   * @brief Maximum number of nodes allowed in the tree.
   */
  unsigned int maximum_nodes_ = std::numeric_limits<unsigned int>::max();

  /**
   * @brief Pointer to the metrics used for cost computation.
   */
  MetricsPtr metrics_;

  /**
   * @brief Pointer to the tree nodes data structure.
   */
  NearestNeighborsPtr nodes_;

  /**
   * @brief Pointer to a TraceLogger instance for logging.
   *
   * This member variable represents a pointer to a TraceLogger instance,
   * allowing to perform logging operations. TraceLogger is a part of the
   * cnr_logger library. Ensure that the logger is properly configured and
   * available for use.
   */
  cnr_logger::TraceLoggerPtr logger_;

  /**
   * @brief Pointer to the collision checker.
   */
  CollisionCheckerPtr checker_;

  /**
   * @brief Recursively purges nodes outside an ellipsoid region based on an
   * informed sampler.
   *
   * This function recursively purges nodes outside an ellipsoid region from the
   * tree, starting from the specified node. The decision to remove a node is
   * based on whether it lies within the admissible informed set defined by the
   * provided sampler. If a node is inside the admissible set, the function
   * continues to check its successors. If a node is outside the admissible set,
   * it is removed, along with its successors, unless they are in the white
   * list.
   *
   * @param node The node from which the recursive purge operation begins.
   * @param sampler An InformedSamplerPtr representing the sampler defining the
   * admissible set.
   * @param white_list A vector of NodePtr representing nodes that should not be
   * removed.
   * @param removed_nodes A reference to an unsigned int, counting the number of
   * nodes removed.
   */
  void purgeNodeOutsideEllipsoid(NodePtr& node, const SamplerPtr& sampler, const std::vector<NodePtr>& white_list,
                                 unsigned int& removed_nodes);
  /**
   * @brief Recursively purges nodes outside multiple ellipsoid regions based on
   * informed samplers.
   *
   * This function recursively purges nodes outside multiple ellipsoid regions
   * from the tree, starting from the specified node. The decision to remove a
   * node is based on whether it belongs to an admissible informed set defined
   * by any of the provided samplers or is present in the white list. If a node
   * is inside an admissible set or in the white list, the function continues to
   * check its successors. If a node is outside all admissible sets and not in
   * the white list, it is removed, along with its successors.
   *
   * @param node The node from which the recursive purge operation begins.
   * @param samplers A vector of InformedSamplerPtr representing the informed
   * samplers defining the admissible sets.
   * @param white_list A vector of NodePtr representing nodes that should not be
   * removed.
   * @param removed_nodes A reference to an unsigned int, counting the number of
   * nodes removed.
   */
  void purgeNodeOutsideEllipsoids(NodePtr& node, const std::vector<SamplerPtr>& samplers,
                                  const std::vector<NodePtr>& white_list, unsigned int& removed_nodes);

  /**
   * @brief Populates the tree with nodes based on specified criteria from a
   * given node.
   *
   * This functin adds the successors of a given node to the tree. The specified
   * node, although not added itself, must already belong to the tree (e.g., it
   * serves as the root). Successors are included only if the following
   * conditions hold: 1) They do not belong to the black_list. 2) If node_check
   * is true, they are collision-checked; if in collision, they are excluded. 3)
   * They satisfy the condition metrics_->utopia(n->getConfiguration(), focus1)
   * + metrics_->utopia(n->getConfiguration(), focus2) < cost, where n is the
   * considered successor.
   *
   * In the case of using the Euclidean distance as metrics, condition 3)
   * ensures that the node belongs to the ellipsoid defined by focus1, focus2
   * and cost.
   *
   * @param node The successors of this node are added to the tree.
   * @param focus1 The first focus of condition 3).
   * @param focus2 The second focus of condition 3).
   * @param cost The cost of condition 3).
   * @param black_list A list of nodes to be excluded from the tree.
   * @param node_check If true, each successor undergoes a collision check and
   * is added to the tree only if valid.
   */
  void populateTreeFromNode(const NodePtr& node, const Eigen::VectorXd& focus1, const Eigen::VectorXd& focus2,
                            const double& cost, const std::vector<NodePtr>& black_list, const bool node_check = false);
  void populateTreeFromNode(const NodePtr& node, const Eigen::VectorXd& focus1, const Eigen::VectorXd& focus2,
                            const double& cost, const bool node_check = false);
  void populateTreeFromNode(const NodePtr& node, const std::vector<NodePtr>& black_list, const bool node_check = false);
  void populateTreeFromNode(const NodePtr& node, const bool node_check = false);

  /**
   * @brief Populates the tree with nodes based on specified criteria from a
   * given node.
   *
   * This function adds the successors of a given node to the tree. The
   * specified node, although not added itself, must already belong to the tree
   * (e.g., it serves as the root). Successors are included only if the
   * following conditions hold: 1) They do not belong to the black_list. 2) If
   * node_check is true, they are collision-checked; if in collision, they are
   * excluded. 3) If the cost to reach the node n +
   * metrics_->utopia(n->getConfiguration(), goal) < cost, where n is the
   * considered successor.
   *
   * @param node The successors of this node are added to the tree.
   * @param goal the goal to reach, considered in condition 3).
   * @param cost The cost of condition 3)
   * @param black_list A list of nodes to be excluded from the tree.
   * @param node_check If true, each successor undergoes a collision check and
   * is added to the tree only if valid.
   */
  void populateTreeFromNodeConsideringCost(const NodePtr& node, const Eigen::VectorXd& goal, const double& cost,
                                           const std::vector<NodePtr>& black_list, const bool node_check = false);
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * @brief print_full_tree_ if false, the operator << will print number of
   * nodes and root. Otherwise, it will also print the tree connections.
   */
  bool print_full_tree_ = false;

  /**
   * @brief Constructor for the Tree class.
   *
   * Initializes a tree with the specified parameters, including the root node,
   * maximum distance, collision checker, metrics, logger, and an option to use
   * a k-d tree for nearest neighbor queries.
   *
   * @param root A constant reference to a NodePtr representing the root node of
   * the tree.
   * @param max_distance A constant reference to a double representing the
   * maximum distance for extending the tree.
   * @param checker A constant reference to a CollisionCheckerPtr representing
   * the collision checker used by the tree.
   * @param metrics A constant reference to a MetricsPtr representing the
   * metrics used by the tree.
   * @param logger A constant reference to a cnr_logger::TraceLoggerPtr
   * representing the logger used by the tree.
   * @param use_kdtree A constant reference to a bool indicating whether to use
   * a k-d tree for nearest neighbor queries (default is true).
   */
  Tree(const NodePtr& root, const double& max_distance, const CollisionCheckerPtr& checker, const MetricsPtr& metrics,
       const cnr_logger::TraceLoggerPtr& logger, const bool& use_kdtree = true);

  /**
   * @brief Checks if the tree is considered a subtree.
   *
   * @return Returns false, indicating that the tree is not considered a
   * subtree.
   */
  virtual bool isSubtree()
  {
    return false;
  }

  /**
   * @brief Retrieves a constant reference to the root node of the tree.
   *
   * @return Returns a constant reference to the root node of the tree.
   */
  const NodePtr& getRoot()
  {
    return root_;
  }

  /**
   * @brief Retrieves a vector of all nodes in the tree.
   *
   * @return Returns a vector of NodePtr containing all nodes in the tree.
   */
  std::vector<NodePtr> getNodes()
  {
    return nodes_->getNodes();
  }

  /**
   * @brief Retrieves the leaves of the tree.
   *
   * This function populates the provided vector 'leaves' with the leaf nodes of
   * the tree. A leaf node is defined as a node with exactly one parent
   * connection (excluding the root) and no child connections.
   *
   * @param leaves A vector of NodePtr to store the leaf nodes of the tree.
   */
  void getLeaves(std::vector<NodePtr>& leaves);

  /**
   * @brief Changes the root of the tree to the specified node.
   *
   * This function changes the root of the tree to the provided node. It first
   * checks if the node is already in the tree using the 'isInTree' method. If
   * the node is in the tree, the function retrieves the connections leading to
   * the node and flips them. Finally, the root of the tree is updated to the
   * provided node.
   *
   * @param node The node to be set as the new root of the tree.
   * @return Returns true if the root is successfully changed, and false
   * otherwise.
   */
  bool changeRoot(const NodePtr& node);

  /**
   * @brief Adds a node to the tree, optionally checking for its presence.
   *
   * This function adds the provided node to the tree. The addition is
   * conditional on whether the node is already present in the tree, determined
   * by the 'check_if_present' parameter. If 'check_if_present' is set to false
   * or the node is not in the tree, the node is inserted into the underlying
   * 'nodes_' structure.
   *
   * @param node The node to be added to the tree.
   * @param check_if_present A boolean flag indicating whether to check if the
   * node is already present in the tree.
   */
  virtual void addNode(const NodePtr& node, const bool& check_if_present = true);

  /**
   * @brief Removes a node from the tree.
   *
   * This function removes the provided node from the tree. The node is first
   * disconnected from its parent using the 'disconnect' method. Subsequently,
   * the node is deleted from the underlying 'nodes_' structure using the
   * 'deleteNode' method.
   *
   * @param node The node to be removed from the tree.
   */
  virtual void removeNode(const NodePtr& node);

  /**
   * @brief Attempts to extend the tree to the provided configuration.
   *
   * This function finds the closest existing node in the tree to the given
   * configuration using findClosestNode function, and then attempts to extend
   * the tree from that closest node using tryExtendFromNode function.
   *
   * tryExtend does not create a new node.
   *
   * @param configuration The input configuration to which the function attempts
   * to extend the tree.
   * @param next_configuration A reference to a VectorXd object where the newly
   * generated configuration will be stored if the extension is successful.
   * @param closest_node A reference to a pointer to a Node object, pointing to
   * the closest node in the tree.
   * @return Returns true if the extension is possible, and false otherwise.
   */
  bool tryExtend(const Eigen::VectorXd& configuration, Eigen::VectorXd& next_configuration, NodePtr& closest_node);

  /**
   * @brief Attempts to extend the tree from a given node to a new
   * configuration.
   *
   * This function is responsible for trying to extend the tree from a specified
   * node to the provided configuration by selecting a new configuration and
   * checking if the extension is valid based on a given tolerance and collision
   * checking. The new configuration if selected by selectNextConfiguration
   * function. If the distance between the configuration and the closest node is
   * less than max_distance_, the function returns the input configuration.
   * Otherwise it calculates a new configuration at a distance equal to
   * max_distance_ from the node and in the direction of the input
   * configuration.
   *
   * tryExtendFromNode does not create a new node.
   *
   * @param configuration The input configuration to which the function attempts
   * to extend the tree.
   * @param next_configuration A reference to a VectorXd object where the newly
   * generated configuration will be stored if the extension is successful.
   * @param node A reference to a pointer to a Node object representing the
   * starting point for the extension.
   * @return Returns true if the extension is possible, and false otherwise.
   */
  bool tryExtendFromNode(const Eigen::VectorXd& configuration, Eigen::VectorXd& next_configuration, NodePtr& node);

  /**
   * @brief Attempts to extend the tree to the provided configuration if the
   * whole path to the new configuration is collision-free.
   *
   * This function finds the closest existing node in the tree to the given
   * configuration using findClosestNode function, and then attempts to extend
   * the tree from that closest node using tryExtendFromNodeWithPathCheck
   * function, which checks if the path to the closest node found is
   * collision-free.
   *
   * tryExtendWithPathCheck does not create a new node.
   *
   * @param configuration The input configuration to which the function attempts
   * to extend the tree.
   * @param next_configuration A reference to a VectorXd object where the newly
   * generated configuration will be stored if the extension is successful.
   * @param closest_node A reference to a pointer to a Node object representing
   * the starting point for the extension.
   * @param checked_connections A reference to a vector of ConnectionPtr objects
   * containing the connections checked during the path check.
   * @return Returns true if the extension is possible and the whole path from
   * the root to the new configuration is collisione-free, and false otherwise.
   */
  bool tryExtendWithPathCheck(const Eigen::VectorXd& configuration, Eigen::VectorXd& next_configuration,
                              NodePtr& closest_node, std::vector<ConnectionPtr>& checked_connections);

  /**
   * @brief Attempts to extend the tree from a given node to the provided
   * configuration if the whole path to the new configuration is collision-free.
   *
   * This function is responsible for trying to extend the tree from a specified
   * node by first checking the path to that node using checkPathToNode
   * function. Note that checkPathToNode checks only connection for which
   * isRecentlyChecked returns false to avoid multiple checks of the same
   * connection. If the path check is successful, it proceeds to select a new
   * configuration using the selectNextConfiguration method and checks if the
   * extension is valid based on a given tolerance and collision checking. If
   * the distance between the configuration and the closest node is less than
   * max_distance_, the function returns the input configuration. Otherwise it
   * calculates a new configuration at a distance equal to max_distance_ from
   * the node and in the direction of the input configuration.
   *
   * tryExtendFromNodeWithPathCheck does not create a new node.
   *
   * @param configuration The input configuration to which the function attempts
   * to extend the tree.
   * @param next_configuration A reference to a VectorXd object where the newly
   * generated configuration will be stored if the extension is successful.
   * @param node A reference to a pointer to a Node object representing the
   * starting point for the extension.
   * @param checked_connections A reference to a vector of ConnectionPtr objects
   * containing the connections checked during the path check.
   * @return Returns true if the extension is possible and the whole path from
   * the root to the new configuration is collisione-free, and false otherwise.
   */
  bool tryExtendFromNodeWithPathCheck(const Eigen::VectorXd& configuration, Eigen::VectorXd& next_configuration,
                                      NodePtr& node, std::vector<ConnectionPtr>& checked_connections);

  /**
   * @brief Selects the final configuration for tree expansion, which will start
   * from the provided node in the direction of the provided configuration.
   *
   * The function calculates the distance between the input configuration and
   * the current node's configuration using the norm() method. If the calculated
   * distance is below a specified tolerance (TOLERANCE), the next configuration
   * is set to the input configuration. If the distance is less than the maximum
   * allowed distance (max_distance_), the next configuration is set to the
   * input configuration. Otherwise, the next configuration is determined based
   * on linear interpolation between the current node's configuration and the
   * input configuration.
   *
   * @param configuration The input configuration to determine the direction of
   * expansion.
   * @param next_configuration A reference to a VectorXd object where the
   * selected next configuration will be stored.
   * @param node A pointer to a Node object representing the starting node in
   * the tree.
   * @return Returns the distance between the input configuration and the
   * current node's configuration.
   */
  double selectNextConfiguration(const Eigen::VectorXd& configuration, Eigen::VectorXd& next_configuration,
                                 const NodePtr& node);

  /**
   * @brief Extends the tree by attempting to add a new node based on the
   * provided configuration.
   *
   * This function tries to extend the tree by finding the closest existing node
   * to the given configuration using the tryExtend method. If the extension is
   * possible (t\ryExtend returns true), the tree is updated with a new node
   * based on the configuration provided by tryExtend and a connection is
   * established between the closest existing node and the new node using the
   * extendOnly function. If the extension is not possible, the tree remains
   * unchanged, and both new_node and connection are set to nullptr.
   *
   * @param configuration The input configuration to which the function attempts
   * to extend the tree.
   * @param new_node A reference to a pointer to a Node object that will be set
   * to the newly created node if the extension is successful.
   * @param connection A reference to a pointer to a Connection object that will
   * be set to the connection between the existing and new nodes.
   * @return Returns true if the extension is successful, and false otherwise.
   */
  bool extend(const Eigen::VectorXd& configuration, NodePtr& new_node, ConnectionPtr& connection);
  bool extend(const Eigen::VectorXd& configuration, NodePtr& new_node);

  /**
   * @brief Extends the tree by attempting to add a new node based on the
   * provided configuration, if that the whole path to the new selected
   * configuration is collision-free.
   *
   * This function tries to extend the tree by finding the closest existing node
   * to the given configuration and checking the whole path to the configuration
   * using the tryExtendWithPathCheck method. If the extension is possible
   * (tryExtend returns true), the tree is updated with a new node based on the
   * configuration provided by tryExtendWithPathCheck and a connection is
   * established between the closest existing node and the new node using the
   * extendOnly function. The flag recentlyChecked of the new connection is set
   * true. If the extension is not possible, the tree remains unchanged, and
   * both new_node and connection are set to nullptr.
   *
   * A version wich does not return the connection pointer is also available.
   *
   * @param configuration The input configuration to which the function attempts
   * to extend the tree.
   * @param new_node A reference to a pointer to a Node object that will be set
   * to the newly created node if the extension is successful.
   * @param connection A reference to a pointer to a Connection object that will
   * be set to the connection between the existing and new nodes.
   * @param checked_connections A reference to a vector of ConnectionPtr objects
   * containing the connections checked during the path check.
   * @return Returns true if the extension is successful, and false otherwise.
   */
  bool extendWithPathCheck(const Eigen::VectorXd& configuration, NodePtr& new_node, ConnectionPtr& connection,
                           std::vector<ConnectionPtr>& checked_connections);
  bool extendWithPathCheck(const Eigen::VectorXd& configuration, NodePtr& new_node,
                           std::vector<ConnectionPtr>& checked_connections);

  /**
   * @brief Extends the tree to a newly created node by establishing a
   * connection with the closest existing node.
   *
   * This function extends the tree by creating a connection between the new
   * node and the closest existing node, both provided by input. The cost of the
   * connection is calculated using the specified metrics, and the new node is
   * added to the tree.
   *
   * @param closest_node A reference to a pointer to a Node object representing
   * the closest existing node.
   * @param new_node A reference to a pointer to a Node object representing the
   * new node.
   * @param connection A reference to a pointer to a Connection object that will
   * be set to the connection between the two nodes.
   * @return Returns true if the extension is successful, and false otherwise.
   */
  bool extendOnly(NodePtr& closest_node, NodePtr& new_node, ConnectionPtr& connection);

  /**
   * @brief Extends the tree to include a specified node.
   *
   * This function extends the tree to include the given node. If the node is
   * already in the tree, the function returns true. Otherwise, it attempts to
   * extend the tree to the node's configuration using the tryExtend method. If
   * the extension is successful, a new node is created, and a connection is
   * established between the closest existing node and the new node. The cost of
   * the connection is calculated using the specified metrics.
   *
   * @param node A reference to a pointer to a Node object representing the node
   * to be included in the tree.
   * @param new_node A reference to a pointer to a Node object representing the
   * newly created node or the existing node if it is already in the tree.
   * @return Returns true if the tree is extended to include the node, and false
   * otherwise.
   */
  bool extendToNode(const NodePtr& node, NodePtr& new_node);

  /**
   * @brief Connects the tree to a specified configuration by repeatedly
   * extending towards that configuration.
   *
   * This function connects the tree to a specified configuration by iteratively
   * extending the tree towards that configuration. The extension is performed
   * using the extend method. If the extension is successful, the new node is
   * updated, and the function checks if the distance between the new node's
   * configuration and the specified configuration is within tolerance. If so,
   * the connection is considered successful. The process continues until no
   * further extension is possible or until the distance is within tolerance.
   *
   * @param configuration The target configuration to which the tree tries to
   * extend.
   * @param new_node A reference to a pointer to a Node object representing the
   * last node created by the extend function, namely the farthest from the
   * closest node of the tree and the closest one to node.
   * @return Returns true if the tree is successfully connected to the specified
   * configuration, and false otherwise.
   */
  bool connect(const Eigen::VectorXd& configuration, NodePtr& new_node);

  /**
   * @brief Implements an informed extension method that prioritizes nodes with
   * lower heuristic values.
   *
   * This function attempts to extend the tree from the closest nodes to a
   * specified configuration, prioritizing nodes with lower heuristic values.
   * The heuristic is calculated as a weighted sum of the distance between the
   * node and the configuration and the cost-to-come to reach the node from
   * start. Specifically, the heuristic is computed as: heuristic =
   * bias*distance between node and configuration + (1-bias)*cost to the node
   * from root Bias is an input value between 0 and 1. The function iterates
   * through the closest nodes, calculates the heuristic, and adds nodes with a
   * favorable heuristic to a priority queue. It then attempts to extend the
   * tree from the selected nodes and checks for successful extensions based on
   * distance and path validity. A close node is added to the queue if and only
   * if the following condition is true:
   *
   *          cost to the node + distance between node and configuration +
   * distance between configuration and goal < cost to beat
   *
   * @param configuration The target configuration to which the tree is
   * extended.
   * @param new_node A reference to a pointer to a Node object representing the
   * newly created node if the extension is successful.
   * @param goal The goal is the final configuration of the path planning
   * problem. It is used in the cost-to-go calculation.
   * @param cost2beat The value used in the condition above.
   * @param bias The bias factor determining the weights in the heuristic
   * calculation. It belongs to the interval [0,1]
   * @return Returns true if the tree is successfully extended to the specified
   * configuration, and false otherwise.
   */
  bool informedExtend(const Eigen::VectorXd& configuration,  // Used in AnytimeRRT
                      NodePtr& new_node, const Eigen::VectorXd& goal, const double& cost2beat, const double& bias);

  /**
   * @brief Connects the tree to a specified node.
   *
   * This function connects the tree to a specified node by repeatedly extending
   * the tree towards that node. The extension is performed using the
   * extendToNode method. If the extension is successful, the new node is
   * updated, and the function checks if the distance between the new node's
   * configuration and the specified node's configuration is within tolerance.
   * The process continues until no further extension is possible, the distance
   * is within tolerance, or the maximum time limit is reached.
   *
   * @param node A reference to a pointer to a Node object representing the node
   * to which the tree is connected.
   * @param new_node A reference to a pointer to a Node object representing the
   * last node created by the extend function, namely the farthest from the
   * closest node of the tree and the closest one to node.
   * @param max_time The maximum time limit (in seconds) for attempting the
   * connection.
   * @return Returns true if the tree is successfully connected to the specified
   * node, and false otherwise.
   */
  bool connectToNode(const NodePtr& node, NodePtr& new_node,
                     const double& max_time = std::numeric_limits<double>::infinity());

  /**
   * @brief Rewires the tree by potentially updating parent and/or child
   * connections of a specified node.
   *
   * This function attempts to rewire the tree by potentially updating the
   * parent and/or child connections of a specified node. The rewire operation
   * is influenced by a given radius and a white list of nodes that should not
   * be rewired. If a node belongs to the white list, a new parent for that node
   * is not searched. By setting the rewiring radius <=0 the nearest nodes
   * considered are the K nearest neighbours. The what_rewire parameter
   * determines whether to rewire only parents (1), only children (2), or both
   * (0).
   *
   * @param node A reference to a pointer to a Node object representing the node
   * to be rewired.
   * @param r_rewire The radius within which nodes are considered for rewiring.
   * If <= 0, the nearest nodes are the K nearest neighbours.
   * @param white_list A vector of NodePtr representing nodes that should not be
   * considered for rewiring.
   * @param what_rewire An integer specifying the rewire operation: 0 for for
   * both parents and children, 1 for parents only, and 2 for children only.
   * @return Returns true if the rewire operation results in improved
   * connections, and false otherwise.
   */
  bool rewireOnly(NodePtr& node, double r_rewire, const std::vector<NodePtr>& white_list, const int& what_rewire = 0);
  bool rewireOnly(NodePtr& node, double r_rewire, const int& what_rewire = 0);

  /**
   * @brief Rewires the tree by potentially updating parent and/or child
   * connections of a specified node with path checking.
   *
   * This function attempts to rewire the tree by potentially updating the
   * parent and/or child connections of a specified node. The rewire operation
   * is influenced by a given radius and a white list of nodes that should not
   * be rewired. If a node belongs to the white list, a new parent for that node
   * is not searched. By setting the rewiring radius <=0 the nearest nodes
   * considered are the K nearest neighbours. The what_rewire parameter
   * determines whether to rewire only parents (1), only children (2), or both
   * (0). Path checking is performed to ensure the validity of connections
   * during the rewire operation, e.g. the validity of the path to the new
   * potential parent of the node.
   *
   * @param node A reference to a pointer to a Node object representing the node
   * to be rewired.
   * @param checked_connections A vector of ConnectionPtr representing the
   * connections that have been checked for validity.
   * @param r_rewire The radius within which nodes are considered for rewiring.
   * If <= 0, the nearest nodes are the K nearest neighbours.
   * @param white_list A vector of NodePtr representing nodes that should not be
   * considered for rewiring.
   * @param what_rewire An integer specifying the rewire operation: 0 for
   * parents and children only, 1 for parents only, and 2 for children only.
   * @return Returns true if the rewire operation results in improved
   * connections, and false otherwise.
   */
  bool rewireOnlyWithPathCheck(NodePtr& node, std::vector<ConnectionPtr>& checked_connections, double r_rewire,
                               const std::vector<NodePtr>& white_list, const int& what_rewire = 0);
  bool rewireOnlyWithPathCheck(NodePtr& node, std::vector<ConnectionPtr>& checked_connections, double r_rewire,
                               const int& what_rewire = 0);

  /**
   * @brief Rewires the tree after extending it from a given configuration
   * within a specified radius.
   *
   * This function extends the tree from a given configuration using the extend
   * method. If the extension is successful, it then attempts to rewire the tree
   * around the new node using the rewireOnly method.
   *
   * @param configuration A reference to a constant Eigen::VectorXd representing
   * the configuration from which the tree is extended.
   * @param r_rewire The radius within which nodes are considered for rewiring
   * during the rewire operation. If <= 0, the nearest nodes are the K nearest
   * neighbours.
   * @param new_node A reference to a pointer to a Node object representing the
   * new node added to the tree during extension.
   * @return Returns true if the extension and rewire operations are successful,
   * and false otherwise.
   */
  bool rewire(const Eigen::VectorXd& configuration, double r_rewire, NodePtr& new_node);
  bool rewire(const Eigen::VectorXd& configuration, double r_rewire);

  /**
   * @brief Rewires the tree after extending it to a given configuration with
   * path checking.
   *
   * This function extends the tree from a given configuration using the
   * extendWithPathCheck method. If the extension is successful, it then
   * attempts to rewire the tree around the new node with path checking using
   * the rewireOnlyWithPathCheck method.
   *
   * @param configuration A reference to a constant Eigen::VectorXd representing
   * the configuration to which the tree is extended.
   * @param checked_connections A vector of ConnectionPtr representing the
   * connections that have been checked for validity.
   * @param r_rewire The radius within which nodes are considered for rewiring
   * during the rewire operation. If <= 0, the nearest nodes are the K nearest
   * neighbours.
   * @param white_list A vector of NodePtr representing nodes that should not be
   * considered for rewiring.
   * @param new_node A reference to a pointer to a Node object representing the
   * new node added to the tree during extension.
   * @return Returns true if the extension and rewire operations are successful,
   * and false otherwise.
   */
  bool rewireWithPathCheck(const Eigen::VectorXd& configuration, std::vector<ConnectionPtr>& checked_connections,
                           double r_rewire, const std::vector<NodePtr>& white_list, NodePtr& new_node);
  bool rewireWithPathCheck(const Eigen::VectorXd& configuration, std::vector<ConnectionPtr>& checked_connections,
                           double r_rewire, NodePtr& new_node);
  bool rewireWithPathCheck(const Eigen::VectorXd& configuration, std::vector<ConnectionPtr>& checked_connections,
                           double r_rewire);

  /**
   * @brief Rewires the tree to a specific node after extending it.
   *
   * This function extends the tree to a specific node using the extendToNode
   * method. If the extension is successful, it then attempts to rewire the tree
   * around the new node using the rewireOnly method.
   *
   * @param n A constant reference to a NodePtr representing the target node to
   * which the tree is extended.
   * @param r_rewire The radius within which nodes are considered for rewiring
   * during the rewire operation. If <= 0, the nearest nodes are the K nearest
   * neighbours.
   * @param new_node A reference to a pointer to a Node object representing the
   * new node added to the tree during extension.
   * @return Returns true if the extension and rewire operations are successful,
   * and false otherwise.
   */
  bool rewireToNode(const NodePtr& n, double r_rewire, NodePtr& new_node);
  bool rewireToNode(const NodePtr& n, double r_rewire);

  /**
   * @brief Checks the validity of the path to a specific node.
   *
   * This function checks the validity of the path to a specific node by
   * examining the connections along the path. It first retrieves the
   * connections leading to the node using the getConnectionToNode method. Then,
   * it checks each connection for validity, updating costs and marking
   * connections as checked.
   *
   * @param node A constant reference to a NodePtr representing the target node
   * to which the path is checked.
   * @param checked_connections A vector of ConnectionPtr representing the
   * connections that have been checked for validity.
   * @param path_connections A vector of ConnectionPtr representing the
   * connections along the path to the target node.
   * @return Returns true if the path to the node is valid, and false otherwise.
   */
  bool checkPathToNode(const NodePtr& node, std::vector<ConnectionPtr>& checked_connections,
                       std::vector<ConnectionPtr>& path_connections);
  bool checkPathToNode(const NodePtr& node, std::vector<ConnectionPtr>& checked_connections);

  /**
   * @brief Finds the closest existing node in the tree to a given
   * configuration.
   *
   * This function utilizes the nearestNeighbor method of the nodes_ member to
   * find and return the closest existing node in the tree to the specified
   * configuration.
   *
   * @param configuration A constant reference to an Eigen::VectorXd
   * representing the configuration for which the closest node is sought.
   * @return Returns a pointer to the closest existing node in the tree to the
   * specified configuration.
   */
  NodePtr findClosestNode(const Eigen::VectorXd& configuration);

  /**
   * @brief Calculates the cost to reach a specific node from the tree's root.
   *
   * This function calculates the cost to reach a specific node from the tree's
   * root by traversing the tree along its parent connections. The cost is the
   * sum of the costs of all connections along the path to the root.
   *
   * @param node A pointer to a Node object representing the target node for
   * which the cost is calculated.
   * @return Returns the cost to reach the specified node from the tree's root.
   */
  double costToNode(NodePtr node);

  /**
   * @brief Retrieves the connections along the path to a specific node from the
   * tree's root.
   *
   * This function retrieves the connections along the path to a specific node
   * from the tree's root. It traverses the tree along its parent connections
   * and collects the connections in reverse order.
   *
   * @param node A constant reference to a NodePtr representing the target node
   * for which the connections are retrieved.
   * @return Returns a vector of ConnectionPtr representing the connections
   * along the path from the root to the specified node.
   */
  std::vector<ConnectionPtr> getConnectionToNode(const NodePtr& node);

  /**
   * @brief Keeps only the branch specified by a vector of connections in the
   * tree.
   *
   * This function retains only the branch specified by a vector of connections
   * in the tree. It disconnects the nodes outside the specified branch and
   * updates the tree's nodes_ member accordingly.
   *
   * @param connections A constant reference to a vector of ConnectionPtr
   * representing the connections specifying the branch to be retained.
   * @return Returns true if the operation is successful, and false otherwise.
   */
  bool keepOnlyThisBranch(const std::vector<ConnectionPtr>& connections);

  /**
   * @brief Adds a branch to the tree based on a sequence of connections.
   *
   * This function adds a branch to the tree by taking a sequence of connections
   * as input. The connections represent parent-child relationships between
   * nodes. The start node of the branch is extracted from the first connection
   * in the sequence, and the function ensures that the start node is already
   * part of the tree. If not, an error message is logged, and the function
   * returns false.
   *
   * The function then iterates through the connections, extracting child nodes
   * and adding them to the tree if they are not already present. The nodes are
   * added using the underlying tree structure managed by the 'nodes_' object.
   *
   * @param connections A vector of ConnectionPtr representing the connections
   * in the branch.
   * @return Returns true if the branch is successfully added to the tree, and
   * false otherwise.
   *
   * @note If the input vector 'connections' is empty, the function returns
   * false.
   */
  bool addBranch(const std::vector<ConnectionPtr>& connections);

  /**
   * @brief Adds the nodes of an additional tree to the current tree.
   *
   * This function adds the nodes of an additional tree to the current tree
   * while preserving proper connections. The additional tree is specified by
   * the 'additional_tree' parameter, and the maximum time for connecting nodes
   * is constrained by the 'max_time' parameter.
   *
   * If the root of the additional tree is not already part of the current tree,
   * an attempt is made to connect it to an existing node in the current tree.
   * If this connection fails, the function returns false.
   *
   * Otherwise, if the root of the additional tree belongs to the current tree,
   * the function iterates through the nodes of the additional tree (excluding
   * the root) and adds them to the current tree using the 'addNode' method. The
   * connections between nodes are maintained to ensure the tree structure
   * remains valid.
   *
   * @param additional_tree A TreePtr representing the additional tree to be
   * added.
   * @param max_time The maximum time allowed for connecting the two trees using
   * connectToNode function if the root of the additional tree does not belong
   * to the current tree.
   * @return Returns true if the nodes of the additional tree are successfully
   * added to the current tree, and false otherwise.
   */
  bool addTree(TreePtr& additional_tree, const double& max_time = std::numeric_limits<double>::infinity());

  /**
   * @brief Cleans the tree by removing nodes, except the root.
   *
   * This function cleans the tree by iterating through the children of the root
   * node and removing nodes and their descendants using the purgeFromHere
   * function.
   */
  void cleanTree();

  /**
   * @brief Finds nodes near a given node within a specified radius.
   *
   * This function queries the underlying 'nodes_' structure to find nodes near
   * the specified node within a given radius.
   *
   * @param node The target node for which nearby nodes are to be found.
   * @param radius The radius within which nodes are considered.
   * @return Returns a multimap of nodes ordered by their distances from the
   * target configuration. The multimap associates the distances with the
   * corresponding NodePtr instances.
   */
  std::multimap<double, NodePtr> near(const NodePtr& node, const double& radius);

  /**
   * @brief Finds the K nearest neighbors of a given node.
   *
   * This function queries the underlying 'nodes_' structure to find the K
   * nearest neighbors of the specified node.
   *
   * @param node The target node for which the K nearest neighbors are to be
   * found.
   * @return Returns a multimap of the K nearest neighbors ordered by their
   * distances from the target configuration. The multimap associates the
   * distances with the corresponding NodePtr instances.
   */
  std::multimap<double, NodePtr> nearK(const NodePtr& node);
  std::multimap<double, NodePtr> nearK(const Eigen::VectorXd& conf);

  /**
   * @brief Checks if a given node is present in the tree.
   *
   * This function checks whether the specified node is present in the tree by
   * invoking the 'findNode' method of the underlying 'nodes_' object.
   *
   * @param node The node to be checked for presence in the tree.
   * @return Returns true if the given node is found in the tree, and false
   * otherwise.
   */
  bool isInTree(const NodePtr& node);

  /**
   * @brief Retrieves the total number of nodes in the tree.
   * @return Returns the total number of nodes in the tree.
   */
  unsigned int getNumberOfNodes() const
  {
    return nodes_->size();
  }

  /**
   * @brief Purges nodes outside an ellipsoid region based on a given sampler
   *and white list.
   *
   * This function purges nodes outside an ellipsoid region from the tree if the
   *total number of nodes exceeds a specified maximum. The ellipsoid region is
   *defined by the provided sampler, and nodes listed in the white list are
   *exempt from removal.
   *9
   * If the current number of nodes in the tree is less than the specified
   *maximum, no nodes are purged, and the function returns 0.
   *
   * @param sampler An InformedSamplerPtr representing the sampler defining the
   *ellipsoid region.
   * @param white_list A vector of NodePtr representing nodes that should not be
   *removed.
   * @return Returns the number of nodes purged from the tree outside the
   *ellipsoid region.
   */
  unsigned int purgeNodesOutsideEllipsoid(const SamplerPtr& sampler, const std::vector<NodePtr>& white_list);
  unsigned int purgeNodesOutsideEllipsoids(const std::vector<SamplerPtr>& samplers,
                                           const std::vector<NodePtr>& white_list);

  /**
   * @brief Purges nodes from the tree based on specified conditions and
   * constraints.
   *
   * This function purges nodes from the tree if the total number of nodes
   * exceeds a specified maximum. Nodes are selectively removed based on the
   * following criteria:
   * - Nodes in the white list are exempt from removal.
   * - Nodes outside the bounds defined by the provided sampler are removed if
   * 'check_bounds' is set to true.
   * - Nodes with no child connections are removed.
   *
   * The function iterates through the nodes in the tree and applies the removal
   * criteria, stopping when the number of nodes in the tree is reduced to the
   * specified maximum.
   *
   * @param sampler An InformedSamplerPtr representing the sampler used for
   * bounds checking.
   * @param white_list A vector of NodePtr representing nodes that should not be
   * removed.
   * @param check_bounds A boolean flag indicating whether bounds checking
   * should be performed.
   * @return Returns the number of nodes purged from the tree.
   */
  unsigned int purgeNodes(const SamplerPtr& sampler, const std::vector<NodePtr>& white_list,
                          const bool check_bounds = true);

  /**
   * @brief Purges the specified node from the tree and updates the count of
   * removed nodes.
   *
   * This function disconnects the specified node from its parent and children,
   * and then attempts to remove it from the underlying 'nodes_' structure. If
   * the removal is successful, the 'removed_nodes' count is incremented.
   *
   * @param node The node to be purged from the tree.
   * @param removed_nodes A reference to an unsigned int, counting the number of
   * nodes removed.
   */
  virtual void purgeThisNode(NodePtr& node, unsigned int& removed_nodes);

  /**
   * @brief Recursively purges nodes starting from the specified node and
   * updates the count of removed nodes.
   *
   * This function recursively purges nodes starting from the specified node and
   * updates the count of removed nodes. Nodes listed in the white list are
   * exempt from removal. The function traverses the subtree rooted at the given
   * node and disconnects nodes that are not in the white list, proceeding to
   * remove them from the underlying 'nodes_' structure.
   *
   * @param node The starting node for the recursive purge operation.
   * @param white_list A vector of NodePtr representing nodes that should not be
   * removed.
   * @param removed_nodes A reference to an unsigned int, counting the number of
   * nodes removed.
   * @return Returns true if the specified node and its descendants are
   * successfully purged, and false otherwise.
   */
  virtual bool purgeFromHere(NodePtr& node, const std::vector<NodePtr>& white_list, unsigned int& removed_nodes);
  virtual bool purgeFromHere(NodePtr& node);

  /**
   * @brief Checks if the tree needs cleaning based on the current number of
   * nodes.
   *
   * This function checks whether the tree needs cleaning by comparing the
   * current number of nodes in the tree to the specified maximum number of
   * nodes. Cleaning is deemed necessary if the current number of nodes exceeds
   * the maximum.
   *
   * @return Returns true if the tree needs cleaning (i.e., the current number
   * of nodes exceeds the maximum), and false otherwise.
   */
  bool needCleaning()
  {
    return (getNumberOfNodes() > maximum_nodes_);
  }

  /**
   * @brief Rechecks collision status for the subtree rooted at the specified
   * node.
   *
   * This function recursively rechecks collision status for the subtree rooted
   * at the specified node. It iterates through the child connections of the
   * node, checking the collision status for each connection using the
   * associated collision checker. If a collision is detected, the subtree
   * rooted at the child node is purged, and the function returns false. If no
   * collision is detected, the function continues to recheck collision status
   * for the descendants in the subtree.
   *
   * @param n The root node of the subtree for which collision status is to be
   * rechecked.
   * @return Returns true if the subtree rooted at the specified node is
   * collision-free, and false otherwise.
   */
  bool recheckCollisionFromNode(NodePtr& n);

  /**
   * @brief Rechecks collision status for the entire tree.
   *
   * This function initiates the process of rechecking collision status for the
   * entire tree by invoking the 'recheckCollisionFromNode' method with the root
   * node as the starting point. It recursively traverses the tree, rechecking
   * collision status for each subtree rooted at individual nodes.
   *
   * @return Returns true if the entire tree is collision-free, and false
   * otherwise.
   */
  bool recheckCollision();

  /**
   * @brief Retrieves the maximum distance parameter used in the tree.
   *
   * This function returns a constant reference to the maximum distance
   * parameter used in the tree.
   *
   * @return Returns a constant reference to the maximum distance parameter.
   */
  const double& getMaximumDistance() const
  {
    return max_distance_;
  }

  /**
   * @brief Retrieves a reference to the MetricsPtr associated with the tree.
   *
   * This function returns a reference to the MetricsPtr associated with the
   * tree.
   *
   * @return Returns a reference to the MetricsPtr associated with the tree.
   */
  MetricsPtr& getMetrics()
  {
    return metrics_;
  }

  /**
   * @brief Retrieves a reference to the CollisionCheckerPtr associated with the
   * tree.
   *
   * This function returns a reference to the CollisionCheckerPtr associated
   * with the tree.
   *
   * @return Returns a reference to the CollisionCheckerPtr associated with the
   * tree.
   */
  CollisionCheckerPtr& getChecker()
  {
    return checker_;
  }

  /**
   * @brief Sets the CollisionCheckerPtr for the tree.
   *
   * @param checker The CollisionCheckerPtr to be set for the tree.
   */

  void setChecker(const CollisionCheckerPtr& checker)
  {
    checker_ = checker;
  }

  /**
   * @brief Sets the MetricsPtr for the tree.
   *
   * @param metrics The MetricsPtr to be set for the tree.
   */

  void setMetrics(const MetricsPtr& metrics)
  {
    metrics_ = metrics;
  }

  /**
   * @brief Retrieves a pointer to the TraceLogger associated with the path.
   *
   * This member function provides read-only access to the TraceLogger instance
   * associated with the path, allowing external components to access and
   * utilize the logging capabilities.
   *
   * @return A constant reference to the TraceLogger pointer.
   */
  const cnr_logger::TraceLoggerPtr& getLogger() const
  {
    return logger_;
  }

  /**
   * @brief Sets the TraceLogger associated with the path.
   */
  void setLogger(const cnr_logger::TraceLoggerPtr& logger)
  {
    logger_ = logger;
  }

  /**
   * @brief Retrieves the flag indicating the use of a Kd-tree in the tree
   * structure.
   *
   * @return Returns true if a Kd-tree is used in the tree structure, and false
   * otherwise.
   */
  bool getUseKdTree()
  {
    return use_kdtree_;
  }

  /**
   * @brief Convert the Tree to a YAML::Node.
   *
   * This function converts the Tree to a YAML::Node. It represents the Tree's
   * nodes and connections in YAML format. Nodes configuration is written in the
   * yaml under the field 'nodes'. Connections are written under the field
   * 'connections' as a sequence containing the indices in the nodes in the
   * field 'nodes'.
   *
   * @return A YAML::Node representing the Tree.
   */
  YAML::Node toYAML() const;

  /**
   * @brief Write the Tree to a YAML file.
   *
   * This function writes the YAML representation of the Tree to a file with the
   * specified name.
   *
   * @param file_name The name of the file to write the YAML representation to.
   */
  void toYAML(const std::string& file_name) const;

  /**
   * @brief Output stream operator for a Tree.
   *
   * This operator allows streaming a textual representation of the Tree to an
   * output stream. It prints the number of nodes in the Tree and the
   * information about the root node.
   *
   * @param os The output stream where the Tree information will be printed.
   * @param tree The Tree to be printed.
   * @return A reference to the output stream for chaining.
   */
  friend std::ostream& operator<<(std::ostream& os, const Tree& tree);

  /**
   * @brief Create a Tree from a YAML::Node.
   *
   * This function constructs a Tree from a YAML::Node, assuming the YAML node
   * contains the required fields: 'nodes' representing a sequence of Nodes and
   * 'connections' defining connections by specifying the indices of nodes in
   * the 'nodes' sequence to connect.
   *
   * @note Among the sequence of nodes, it searches for the one without parents
   * to locate the root of the tree.
   *
   * @param yaml The YAML::Node containing 'nodes' and 'connections' fields.
   * @param metrics The MetricsPtr for computing connection costs.
   * @param checker The CollisionCheckerPtr for collision checking.
   * @param logger The TraceLoggerPtr for logging error messages.
   * @return A TreePtr constructed from the YAML::Node. If an error occurs
   * during construction, returns nullptr.
   */
  static TreePtr fromYAML(const YAML::Node& yaml, const MetricsPtr& metrics, const CollisionCheckerPtr& checker,
                          const cnr_logger::TraceLoggerPtr& logger);
};

/**
 * @brief Static inline function to get a parameter from the parameter server
 * and set it to a given TreePtr.
 *
 * This function attempts to retrieve a parameter named `param_name` from the
 * parameter namespace `param_ns`, and assigns its value to the TreePtr `param`.
 * If the parameter is found, it is set to the TreePtr and the function returns
 * true. If the parameter is not found, a warning is logged and the function
 * returns false. If an error occurs while retrieving the parameter, an error is
 * logged and an invalid_argument exception is thrown. Additionally, it sets the
 * logger, metrics, and collision checker for the retrieved TreePtr.
 *
 * @param logger A pointer to a TraceLogger object used for logging messages.
 * @param param_ns The namespace of the parameter on the parameter server.
 * @param param_name The name of the parameter to retrieve.
 * @param param Reference to the PathPtr where the parameter value will be
 * assigned.
 * @param metrics A pointer to the Metrics object to be set for the retrieved
 * TreePtr.
 * @param checker A pointer to the CollisionChecker object to be set for the
 * retrieved TreePtr.
 * @return True if the parameter is successfully retrieved and assigned, false
 * otherwise.
 * @throws std::invalid_argument if an error occurs while retrieving the
 * parameter.
 */
bool get_param(const cnr_logger::TraceLoggerPtr& logger, const std::string& param_ns, const std::string& param_name,
               TreePtr& param, const MetricsPtr& metrics, const CollisionCheckerPtr& checker);
}  // end namespace core
}  // end namespace graph
