#pragma once
/*
Copyright (c) 2019, Manuel Beschi CNR-STIIMA manuel.beschi@stiima.cnr.it
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions aprotectednd the following disclaimer.
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
#include <math.h>

namespace graph
{
namespace core
{

//class Path; //Defined in util.h
typedef std::shared_ptr<Path> PathPtr;

/**
 * @class Path
 * @brief Class for defining a path as a vector of connections.
 */
class Path: public std::enable_shared_from_this<Path>
{
  friend class PathOptimizerBase;

protected:

  /**
   * @brief The starting node of the path.
   */
  NodePtr start_node_;

  /**
   * @brief The goal node of the path.
   */
  NodePtr goal_node_;

  /**
   * @brief The list of connections that define the path.
   */
  std::vector<ConnectionPtr> connections_;

  /**
   * @brief The metrics used for evaluating the cost of the path.
   */
  MetricsPtr metrics_;

  /**
   * @brief The collision checker used for verifying the validity of the path.
   */
  CollisionCheckerPtr checker_;

  /**
   * @brief The cost associated with the path.
   */
  double cost_;

  /**
   * @brief The tree associated with the path.
   */
  TreePtr tree_;

  /**
   * @brief Pointer to a TraceLogger instance for logging.
   *
   * This member variable represents a pointer to a TraceLogger instance, allowing
   * to perform logging operations. TraceLogger is a part of the cnr_logger library.
   * Ensure that the logger is properly configured and available for use.
   */
  cnr_logger::TraceLoggerPtr logger_;

  /**
   * @brief Compute the total cost of the path.
   *
   * This method calculates the total cost of the path by summing the costs of individual connections.
   */
  void computeCost();

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * @brief Constructor for the Path class.
   *
   * This constructor initializes a Path instance with the specified connections, metrics, and collision checker.
   *
   * @param connections A vector of ConnectionPtr representing the connections in the path.
   * @param metrics The MetricsPtr used for computing the cost of the path.
   * @param checker The CollisionCheckerPtr used for checking collision along the path.
   * @param logger The cnr_logger::TraceLoggerPtr logger for logging operations.
   */
  Path(std::vector<ConnectionPtr> connections,
       const MetricsPtr& metrics,
       const CollisionCheckerPtr& checker,
       const cnr_logger::TraceLoggerPtr& logger);

  /**
   * @brief Constructor for the Path class from a vector of nodes.
   *
   * This constructor initializes a Path instance with the specified vector of nodes, metrics, and collision checker.
   * It crates connections between consecutive nodes.
   *
   * @param nodes A vector of NodePtr representing the nodes in the path.
   * @param metrics The MetricsPtr used for computing the cost of the path.
   * @param checker The CollisionCheckerPtr used for checking collision along the path.
   * @param logger The cnr_logger::TraceLoggerPtr logger for logging operations.
   */
  Path(std::vector<NodePtr> nodes,
       const MetricsPtr& metrics,
       const CollisionCheckerPtr& checker,
       const cnr_logger::TraceLoggerPtr& logger);

  /**
   * @brief Get the cost of the path.
   *
   * This method computes and returns the cost of the path by invoking the computeCost() function.
   *
   * @return A reference to the computed cost of the path.
   */
  const double& cost()
  {
    computeCost();
    return cost_;
  }

  /**
   * @brief Get a shared pointer to the current Path instance.
   *
   * This method returns a shared pointer to the current Path instance using shared_from_this().
   *
   * @return A shared pointer to the current Path instance.
   */
  PathPtr pointer()
  {
    return shared_from_this();
  }

  /**
   * @brief Add a node at the current configuration to the path.
   *
   * This method adds a node at the specified configuration to the path. If a connection is provided (non-null),
   * it checks if the configuration corresponds to that of the connection parent or child. If it does, the
   * corresponding parent or child node is returned. Otherwise, a new node is created at the specified configuration
   * and the 'is_a_new_node' is set True. If rewire is set to true, it attempts to rewire the path and tree by creating
   * new connections.
   *
   * @param configuration The configuration of the new node.
   * @param conn The connection to check for the configuration.
   * @param rewire Flag indicating whether to rewire the path.
   * @param is_a_new_node Output parameter indicating whether a new node was created.
   * @return A pointer to the path node corresponding to the configuration.
   */
  NodePtr addNodeAtCurrentConfig(const Eigen::VectorXd& configuration, ConnectionPtr &conn, const bool &rewire, bool& is_a_new_node);
  NodePtr addNodeAtCurrentConfig(const Eigen::VectorXd& configuration, ConnectionPtr &conn, const bool &rewire);
  NodePtr addNodeAtCurrentConfig(const Eigen::VectorXd& configuration, const bool& rewire);

  /**
   * @brief Resample the path to reduce the distance between nodes.
   *
   * This method resamples the path to reduce the distance between nodes, ensuring that the distance between consecutive
   * nodes does not exceed the specified maximum distance. It adds new nodes to achieve the resampling, and returns the
   * number of nodes added.
   *
   * @param max_distance The maximum distance between nodes in the resampled path.
   * @return The number of nodes added to the path.
   */
  int resample(const double &max_distance);

  /**
   * @brief Remove a node from the path and update connections accordingly.
   *
   * This method removes a specified node from the path, and updates the connections to maintain the path integrity.
   * A node can be removed if:
   *  - it is not the start or goal node;
   *  - it has only one parent and one child;
   *  - parent and child connections are parallel.
   *
   * The node is disconnected and removed from the tree.
   * It returns true if the removal is successful, and false otherwise.
   *
   * @param node The node to be removed from the path.
   * @param idx_conn The index of the connection before the specified node in the connections vector.
   * @param white_list A list of nodes that should not be removed.
   * @param new_conn A reference to a ConnectionPtr, which will be updated to point to the new connection after the removal.
   * @param toll The tolerance for considering two connections as parallel.
   * @return True if the node removal is successful, false otherwise.
   */
  bool removeNode(const NodePtr &node, const size_t &idx_conn, const std::vector<NodePtr> &white_list, ConnectionPtr &new_conn, const double &toll = 1e-06);
  bool removeNode(const NodePtr &node, const size_t& idx_conn, const std::vector<NodePtr> &white_list);
  bool removeNode(const NodePtr& node, const std::vector<NodePtr> &white_list, ConnectionPtr &new_conn);
  bool removeNode(const NodePtr& node, const std::vector<NodePtr> &white_list);

  /**
   * @brief Remove nodes from the path and update connections accordingly.
   *
   * This method removes nodes from the path based on the provided white list, and updates the connections to maintain path integrity.
   * Nodes are removed using the removeNode method.
   * It returns true if at least one node is removed, and false otherwise.
   *
   * @param white_list A list of nodes that should not be removed.
   * @param deleted_nodes A vector to store the nodes that are successfully removed.
   * @param toll A tolerance value to check for parallelism between connections.
   * @return True if at least one node is removed, false otherwise.
   */
  bool removeNodes(const std::vector<NodePtr>& white_list, std::vector<NodePtr>& deleted_nodes, const double &toll = 1e-06);
  bool removeNodes(const std::vector<NodePtr> &white_list, const double& toll = 1e-06);
  bool removeNodes(const double& toll = 1e-06);

  /**
   * @brief Find a connection in the path based on the given configuration.
   *
   * This method searches for a connection in the path that corresponds to the specified configuration.
   * If found, it returns the connection and its index in the path. If not found, it returns nullptr.
   *
   * @param configuration The configuration to search for in the path.
   * @param idx The index of the found connection in the path.
   * @param verbose If true, print additional information for debugging.
   * @return The found connection or nullptr if not found.
   */
  ConnectionPtr findConnection(const Eigen::VectorXd& configuration, size_t &idx, bool verbose = false);
  ConnectionPtr findConnection(const Eigen::VectorXd& configuration);

  /**
   * @brief Find the closest node in the path to the given configuration.
   *
   * This method searches for the node in the path that is closest to the specified configuration.
   * It returns the closest node and the distance between the configuration and the closest node.
   *
   * @param configuration The configuration for which to find the closest node.
   * @param dist Output parameter for the distance between the configuration and the closest node.
   * @return The closest node in the path to the given configuration.
   */
  NodePtr findCloserNode(const Eigen::VectorXd& configuration, double &dist);
  NodePtr findCloserNode(const Eigen::VectorXd& configuration);
  NodePtr findCloserNode(const NodePtr& node);
  NodePtr findCloserNode(const NodePtr& node, double &dist);

  /**
   * @brief Get a subpath from the current path starting from a specified configuration.
   *
   * If `clone` is true, the method returns a clone of the subpath without modifying the original path.
   * Nodes and connections of the original path are cloned and no tree is linked to the path.
   * If `clone` is false, the method adds a new node at the specified configuration to the original path,
   * rewires the path and the tree, and returns the actual subpath.
   *
   * @param conf The configuration from which to start the subpath.
   * @param clone If true, return a copy of the subpath without modifying the original path.
   *                 If false, add a new node at the specified configuration and return the actual subpath.
   * @return The subpath starting from the specified configuration.
   */
  PathPtr getSubpathFromConf(const Eigen::VectorXd& conf, const bool clone);

  /**
   * @brief Get a subpath from the current path ending at a specified configuration.
   *
   * If `clone` is true, the method returns a copy of the subpath without modifying the original path.
   * Nodes and connections of the original path are cloned and no tree is linked to the path.
   * If `clone` is false, the method adds a new node at the specified configuration to the original path,
   * rewires the path and the tree, and returns the actual subpath.
   *
   * @param conf The configuration at which to end the subpath.
   * @param clone If true, return a copy of the subpath without modifying the original path.
   *              If false, add a new node at the specified configuration and return the actual subpath.
   * @return The subpath ending at the specified configuration.
   */
  PathPtr getSubpathToConf(const Eigen::VectorXd& conf, const bool clone);

  /**
   * @brief Get a subpath from the current path starting at the specified configuration.
   *
   * If the specified configuration is equal to the last node's configuration, an error is thrown as no subpath is available.
   * If the specified configuration is equal to the start node's configuration, the method returns a pointer to the current path.
   * Otherwise, the method finds the connection corresponding to the specified configuration and constructs a subpath
   * starting from that connection's child node.
   *
   * @param conf The configuration at which to start the subpath.
   * @return The subpath starting at the specified configuration.
   */
  PathPtr getSubpathFromNode(const Eigen::VectorXd& conf);
  PathPtr getSubpathFromNode(const NodePtr& node);

  /**
   * @brief Get a subpath from the current path ending at the specified configuration.
   *
   * If the specified configuration is equal to the first node's configuration, an error is thrown as no subpath is available.
   * If the specified configuration is equal to the goal node's configuration, the method returns a pointer to the current path.
   * Otherwise, the method finds the connection corresponding to the specified configuration and constructs a subpath
   * ending at that connection's child node.
   *
   * @param conf The configuration at which to end the subpath.
   * @return The subpath ending at the specified configuration.
   */
  PathPtr getSubpathToNode(const Eigen::VectorXd& conf);
  PathPtr getSubpathToNode(const NodePtr& node);

  /**
   * @brief Compute the sum of Euclidean norms of all connections in the path.
   *
   * @return The sum of Euclidean norms of all connections in the path.
   */
  double computeEuclideanNorm();

  /**
   * @brief Compute a point on the path given a curvilinear abscissa.
   *
   * @param abscissa The curvilinear abscissa.
   * @param connection Outputs the connection on which the point lies.
   * @return The configuration of the point on the path.
   */
  Eigen::VectorXd pointOnCurvilinearAbscissa(const double& abscissa, ConnectionPtr &connection);
  Eigen::VectorXd pointOnCurvilinearAbscissa(const double& abscissa);

  /**
   * @brief Compute the curvilinear abscissa of a point on the path.
   *
   * @param conf The configuration of the point.W
   * @param idx Outputs the index of the connection on which the point lies.
   * @return The curvilinear abscissa of the point.
   */
  double curvilinearAbscissaOfPoint(const Eigen::VectorXd& conf, size_t &idx);
  double curvilinearAbscissaOfPoint(const Eigen::VectorXd& conf);

  /**
   * @brief Compute the curvilinear abscissa of a point on a specific connection of the path.
   *
   * @param conf The configuration of the point.
   * @param conn_idx The index of the connection on which the point lies.
   * @return The curvilinear abscissa of the point on the specified connection.
   */
  double curvilinearAbscissaOfPointGivenConnection(const Eigen::VectorXd& conf,const size_t& conn_idx);

  /**
   * @brief Get the cost of the path from a specific configuration.
   *
   * @param conf The configuration of interest.
   * @return The cost from of the path the specified configuration.
   */
  double getCostFromConf(const Eigen::VectorXd& conf);

  /**
   * @brief Get the curvilinear norm from the specified configuration to the end of the path.
   *
   * @param conf The configuration of interest.
   * @return The curvilinear norm from the specified configuration to the end of the path.
   */
  double getNormFromConf(const Eigen::VectorXd& conf);

  /**
   * @brief Get the nodes associated with the path.
   *
   * This method retrieves a vector of nodes that form the path. The nodes include both
   * the parent and child nodes of each connection in the path.
   *
   * @return A vector of NodePtr representing the nodes in the path.
   */
  std::vector<NodePtr> getNodes() const;

  /**
   * @brief Get the waypoints along the path.
   *
   * This method retrieves the configurations of all waypoints along the path.
   *
   * @return A vector of Eigen::VectorXd containing the configurations of waypoints.
   *         An empty vector is returned if the path has no connections.
   */
  std::vector<Eigen::VectorXd> getWaypoints() const;

  /**
   * @brief Get the start node of the path.
   *
   * This method retrieves the start node of the path.
   *
   * @return A shared pointer to the start node of the path.
   */
  NodePtr getStartNode() const
  {
    return start_node_;
  }

  /**
   * @brief Get the goal node of the path.
   *
   * This method retrieves the goal node of the path.
   *
   * @return A shared pointer to the goal node of the path.
   */
  NodePtr getGoalNode() const
  {
    return goal_node_;
  }

  /**
   * @brief Get the associated tree.
   *
   * This method retrieves the tree associated with the path.
   *
   * @return A shared pointer to the associated tree.
   */
  TreePtr getTree()
  {
    return tree_;
  }

  /**
   * @brief Get the collision checker.
   *
   * This method retrieves the collision checker of the path.
   *
   * @return A shared pointer to the associated tree.
   */
  CollisionCheckerPtr getChecker()
  {
    return checker_;
  }

  /**
   * @brief Get the metrics used for evaluating the cost of the path.
   *
   * This method retrieves the metrics used to evaluate the cost of the path.
   *
   * @return A shared pointer to the metrics of the path.
   */
  MetricsPtr getMetrics()
  {
    return metrics_;
  }

  /**
   * @brief Set the metrics used for evaluating the cost of the path.
   *
   * This method sets the metrics that will be used to evaluate the cost of the path.
   *
   * @param metrics A shared pointer to the metrics used for evaluating path cost.
   */
  void setMetrics(const MetricsPtr& metrics)
  {
    metrics_ = metrics;
  }

  /**
   * @brief Get the number of connections in the path.
   *
   * This method returns the number of connections present in the path.
   *
   * @return The number of connections in the path.
   */
  unsigned int getConnectionsSize()
  {
    return connections_.size();
  }

  /**
   * @brief Retrieves a pointer to the TraceLogger associated with the path.
   *
   * This member function provides read-only access to the TraceLogger instance associated
   * with the path, allowing external components to access and utilize the logging capabilities.
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
  void  setLogger(const cnr_logger::TraceLoggerPtr& logger)
  {
    logger_ = logger;
  }

  /**
   * @brief Set the collision checker for the path and its associated tree.
   *
   * This method sets the collision checker for the path. If a tree is associated with the path,
   * it also updates the collision checker for the tree.
   *
   * @param checker The collision checker to be set.
   */
  void setChecker(const CollisionCheckerPtr &checker)
  {
    checker_ = checker;
    if(tree_)
      tree_->setChecker(checker);
  }

  /**
   * @brief Set the tree associated with the path.
   *
   * This method sets the tree associated with the path.
   *
   * @param tree A shared pointer to the Tree object.
   */
  void setTree(const TreePtr& tree)
  {
    tree_ = tree;
  }

  /**
   * @brief Get the connections that compose the path.
   *
   * This method returns a vector of shared pointers to Connection objects representing the connections that compose the path.
   *
   * @return A vector of shared pointers to Connection objects.
   */
  std::vector<ConnectionPtr> getConnections()const
  {
    return connections_;
  }

  /**
   * @brief Get the i-th connection that composes the path.
   *
   * @param i The index of the desired connection.
   *
   * @return The shared pointers to i-th Connection object, nullptr if 'i' exceeds the vector bounds.
   */
  ConnectionPtr getConnection(const size_t &i)const;

  /**
   * @brief Get a constant reference to the connections that compose the path.
   *
   * This method returns a constant reference to a vector of shared pointers to Connection objects representing the connections that compose the path.
   *
   * @return A constant reference to a vector of shared pointers to Connection objects.
   */
  const std::vector<ConnectionPtr>& getConnectionsConst() const
  {
    return connections_;
  }

  /**
   * @brief Create a copy of the path.
   *
   * This method creates and returns a new PathPtr that is a copy of the current path.
   * The new path contains cloned nodes and connections, and the original and cloned paths are independent.
   * The cloned path does not belong to the original tree, because and the nodes and connections are cloned.
   *
   * @return A PathPtr representing the copy of the path.
   */
  PathPtr clone();

  /**
   * @brief Check if all connections in the path lie on the same straight line.
   *
   * This method checks if all connections in the path are parallel, indicating that they lie on the same straight line.
   *
   * @param toll Tolerance for considering connections as parallel.
   * @return True if all connections are parallel, false otherwise.
   */
  bool onLine(double toll = 1e-06);

  /**
   * @brief Set the connections of the path.
   *
   * This method sets the connections of the path using the provided vector of connections. It updates the cost,
   * start_node_, and goal_node_ attributes based on the input connections.
   *
   * @param conn A vector of ConnectionPtr representing the connections to set for the path.
   */
  void setConnections(const std::vector<ConnectionPtr>& conn);

  /**
   * @brief Split a connection and update the path.
   *
   * This method splits a connection by replacing the connection at position specified by the iterator with conn1
   * and inserting conn2 at the next position. It updates the connections of the path and, if applicable, the tree
   * by adding the child node of conn1.
   *
   * The splitting connections conn1 and conn2 must be provided as input. Conn1 must have the same parent of the original
   * connection, conn2 the same child and conn1 and conn2 must be connected by the same node.
   *
   * @param conn1 The first connection to be used in the split.
   * @param conn2 The second connection to be inserted after conn1 in the split.
   * @param it An iterator pointing to the connection to split.
   * @return True if the split is successful, false otherwise.
   */
  bool splitConnection(const ConnectionPtr& conn1, const ConnectionPtr& conn2, const std::vector<ConnectionPtr>::iterator &it);
  bool splitConnection(const ConnectionPtr& conn1, const ConnectionPtr& conn2, const ConnectionPtr& conn);

  /**
   * @brief Restore a connection in the path and cuts off a node with the provided connection.
   *
   * This method restores a connection in the path and cuts off a node with the provided connection.
   * It updates the connections of the path and, if applicable, the tree by removing the specified node.
   *
   * @param conn The connection to be used in the restoration.
   * @param node2remove The node to be removed and replaced by the connection.
   * @return True if the restoration is successful, false otherwise.
   */
  bool restoreConnection(const ConnectionPtr& conn, const NodePtr& node2remove);

  /**
   * @brief Checks if the path is valid, taking into account collision checking.
   *
   * This function checks the validity of the path using collision checking. If a custom
   * collision checker (`this_checker`) is provided, it is used; otherwise, the internal
   * collision checker associated with the path is used.
   * If not valid, cost is set equal to infinity.
   *
   * @param this_checker Optional custom collision checker.
   * @return True if the path is collision-free, false otherwise.
   */
  bool isValid(const CollisionCheckerPtr &this_checker = nullptr);

  /**
   * @brief Checks the validity of the path from a specific configuration.
   *
   * This function checks the validity of the path from a given configuration (`conf`) starting
   * from a specific connection index (`conn_idx`). The result is determined using collision checking.
   * If a custom collision checker (`this_checker`) is provided, it is used; otherwise, the internal
   * collision checker associated with the path is used.
   * If not valid, cost is set equal to infinity.
   *
   * @param conf The configuration from which to check the validity.
   * @param conn_idx The index of the connection to start checking from.
   * @param pos_closest_obs_from_goal Output parameter indicating the position of the closest obstacle
   *                                  from the goal, if the path is invalid.
   * @param this_checker Optional custom collision checker.
   * @return True if the path is collision-free from the given configuration, false otherwise.
   */
  bool isValidFromConf(const Eigen::VectorXd &conf, const size_t &conn_idx, int &pos_closest_obs_from_goal, const CollisionCheckerPtr &this_checker = nullptr);
  bool isValidFromConf(const Eigen::VectorXd &conf, int &pos_closest_obs_from_goal, const CollisionCheckerPtr &this_checker = nullptr);
  bool isValidFromConf(const Eigen::VectorXd &conf, const size_t& conn_idx, const CollisionCheckerPtr &this_checker = nullptr);
  bool isValidFromConf(const Eigen::VectorXd &conf, const CollisionCheckerPtr &this_checker = nullptr);

  /**
   * @brief Checks the validity of the path from a specific connection.
   *
   * This function checks the validity of the path starting from a specific connection (`this_conn`).
   * The result is determined using collision checking. If a custom collision checker (`this_checker`)
   * is provided, it is used; otherwise, the internal collision checker associated with the path is used.
   * If not valid, cost is set equal to infinity.
   *
   * @param this_conn The connection from which to start checking the validity.
   * @param this_checker Optional custom collision checker.
   * @return True if the path is collision-free from the given connection, false otherwise.
   */
  bool isValidFromConn(const ConnectionPtr &this_conn, const CollisionCheckerPtr &this_checker = nullptr);

  /**
   * @brief Projects a point onto the closest connection of the path.
   *
   * This function projects a given point onto the closest connection of the path. It iterates through all connections
   * in the path, finds the one with the minimum distance to the point, and returns the projection.
   * The distance is computed between the point to project and the projection on each connection.
   *
   * @param point The point to be projected onto the path.
   * @param verbose If true, additional information will be printed during the projection process.
   * @return The projected point on the closest connection of the path.
   */
  Eigen::VectorXd projectOnClosestConnection(const Eigen::VectorXd& point, const bool verbose = false);

  /**
   * @brief Projects a point onto the path and returns the connection and precise projection.
   *
   * This function projects a point onto the path and returns the connection on which the projection lies,
   * along with the precise projection on the connection. A past projection can be provided to compute a projection
   * which will lie on the path at a curvilinear abscissa >= of that of the past projection.
   * The provided projection is the one that minimize the distance from the input point.
   *
   * @param point The point to be projected onto the path.
   * @param past_projection The past projection point used for curvilinear abscissa calculation.
   * @param conn The connection on which the precise projection lies.
   * @param verbose Whether to print verbose information for debugging.
   * @return The precise projection on the path.
   */
  Eigen::VectorXd projectOnPath(const Eigen::VectorXd& point, const Eigen::VectorXd &past_projection, ConnectionPtr &conn, const bool& verbose = false);
  Eigen::VectorXd projectOnPath(const Eigen::VectorXd& point, const Eigen::VectorXd &past_projection, const bool& verbose = false);
  Eigen::VectorXd projectOnPath(const Eigen::VectorXd& point, const bool& verbose = false);

  /**
   * @brief Flips the path by reversing the order of connections.
   *
   * This function reverses the order of connections in the path and updates the start and goal nodes accordingly.
   */
  void flip();

  /**
   * @brief Convert the Path to a YAML::Node.
   *
   * This function converts the Path to a YAML::Node.
   * It creates a YAML sequence with each element representing a connection in the path.
   *
   * @param file_name The name of the file to write the YAML representation to.
   * @param reverse If true, the path connections will be listed in reverse order.
   * @return A YAML::Node representing the Path.
   */
  YAML::Node toYAML(const bool reverse=false) const;

  /**
   * @brief Write the Path to a YAML file.
   *
   * This function writes the YAML representation of the Path to a file with the specified name.
   *
   * @param file_name The name of the file to write the YAML representation to.
   * @param reverse If true, the path connections will be listed in reverse order.
   */
  void toYAML(const std::string& file_name, const bool reverse=false) const;

  /**
   * @brief Create a Path from a YAML::Node.
   *
   * This function creates a Path from a YAML::Node.
   * It expects the YAML node to be a sequence, where each element represents a Node in the path.
   *
   * @param yaml The YAML::Node containing the sequence of Nodes.
   * @param logger The TraceLoggerPtr for logging error messages.
   * @return A Path constructed from the YAML::Node. If an error occurs during construction, returns an empty Path.
   */
  static PathPtr fromYAML(const YAML::Node& yaml, const MetricsPtr& metrics,
                          const CollisionCheckerPtr& checker, const cnr_logger::TraceLoggerPtr& logger);

  friend std::ostream& operator<<(std::ostream& os, const Path& path);
};

/**
 * @brief Overloaded stream insertion operator for Path objects.
 *
 * This operator allows printing the information of a Path object to an output stream.
 *
 * @param os Output stream.
 * @param path Path object to be printed.
 * @return Reference to the output stream.
 */
std::ostream& operator<<(std::ostream& os, const Path& path);

/**
 * @brief Static inline function to get a parameter from the parameter server and set it to a given PathPtr.
 *
 * This function attempts to retrieve a parameter named `param_name` from the parameter namespace `param_ns`,
 * and assigns its value to the PathPtr `param`. If the parameter is found, it is set to the PathPtr and
 * the function returns true. If the parameter is not found, a warning is logged and the function returns false.
 * If an error occurs while retrieving the parameter, an error is logged and an invalid_argument exception is thrown.
 * Additionally, it sets the logger, metrics, and collision checker for the retrieved PathPtr.
 *
 * @param logger A pointer to a TraceLogger object used for logging messages.
 * @param param_ns The namespace of the parameter on the parameter server.
 * @param param_name The name of the parameter to retrieve.
 * @param param Reference to the PathPtr where the parameter value will be assigned.
 * @param metrics A pointer to the Metrics object to be set for the retrieved PathPtr.
 * @param checker A pointer to the CollisionChecker object to be set for the retrieved PathPtr.
 * @return True if the parameter is successfully retrieved and assigned, false otherwise.
 * @throws std::invalid_argument if an error occurs while retrieving the parameter.
 */
inline bool get_param(const cnr_logger::TraceLoggerPtr& logger, const std::string param_ns, const std::string param_name, PathPtr& param,
                             const MetricsPtr& metrics, const CollisionCheckerPtr& checker);

} //end namespace core
} // end namespace graph
