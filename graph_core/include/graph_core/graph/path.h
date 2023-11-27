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

#include <graph_core/util.h>
#include <graph_core/metrics.h>
#include <graph_core/collision_checker.h>
#include <graph_core/graph/tree.h>
#include <math.h>

namespace pathplan
{

class Path;
typedef std::shared_ptr<Path> PathPtr;
class Path: public std::enable_shared_from_this<Path>
{
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
   * @brief The minimum length of a connection in the path.
   *
   * This is a threshold for bisection function.
   */
  double min_length_ = 0.04;

  /**
   * @brief The tree associated with the path.
   */
  TreePtr tree_;

  /**
   * @brief Vector indicating whether warping changes are needed for each connection in the path.
   *
   * The change_warp_ vector is used to mark connections where warping changes are needed.
   * Each element in the vector corresponds to a connection in the path, and a value of true
   * indicates that warping changes are required for the corresponding connection.
   */
  std::vector<bool> change_warp_;

  /**
   * @brief Compute the total cost of the path.
   *
   * This method calculates the total cost of the path by summing the costs of individual connections.
   */
  void computeCost();

  /**
   * @brief Set the changed flag for the specified connection index.
   *
   * This method sets the "change_warp" flag to true for the specified connection index in the path.
   *
   * @param connection_idx The index of the connection to mark as changed.
   */
  void setChanged(const unsigned int& connection_idx);

  /**
   * @brief Perform bisection to improve the path between two connections.
   *
   * This method applies bisection to refine the path between two connections in the path.
   * It adjusts the configuration of an intermediate node to improve the overall cost of the path.
   *
   * @param connection_idx The index of the connection in the path where bisection is applied.
   * @param center The center point for bisection.
   * @param direction The direction vector for bisection.
   * @param max_distance The maximum distance for bisection.
   * @param min_distance The minimum distance for bisection.
   * @return Returns true if the bisection process results in an improvement, false otherwise.
   */
  bool bisection(const unsigned int& connection_idx,
                 const Eigen::VectorXd& center,
                 const Eigen::VectorXd& direction,
                 double max_distance,
                 double min_distance);

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
   */
  Path(std::vector<ConnectionPtr> connections, const MetricsPtr& metrics, const CollisionCheckerPtr& checker);

  /**
   * @brief Constructor for the Path class from a vector of nodes.
   *
   * This constructor initializes a Path instance with the specified vector of nodes, metrics, and collision checker.
   * It crates connections between consecutive nodes.
   *
   * @param nodes A vector of NodePtr representing the nodes in the path.
   * @param metrics The MetricsPtr used for computing the cost of the path.
   * @param checker The CollisionCheckerPtr used for checking collision along the path.
   */
  Path(std::vector<NodePtr> nodes, const MetricsPtr& metrics, const CollisionCheckerPtr& checker);

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
  bool removeNode(const NodePtr &node, const int& idx_conn, const std::vector<NodePtr> &white_list, ConnectionPtr &new_conn, const double &toll = 1e-06);
  bool removeNode(const NodePtr &node, const int& idx_conn, const std::vector<NodePtr> &white_list);
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
  ConnectionPtr findConnection(const Eigen::VectorXd& configuration, int& idx, bool verbose = false);
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
   * @param conf The configuration of the point.
   * @param idx Outputs the index of the connection on which the point lies.
   * @return The curvilinear abscissa of the point.
   */
  double curvilinearAbscissaOfPoint(const Eigen::VectorXd& conf, int& idx);
  double curvilinearAbscissaOfPoint(const Eigen::VectorXd& conf);

  /**
   * @brief Compute the curvilinear abscissa of a point on a specific connection of the path.
   *
   * @param conf The configuration of the point.
   * @param conn_idx The index of the connection on which the point lies.
   * @return The curvilinear abscissa of the point on the specified connection.
   */
  double curvilinearAbscissaOfPointGivenConnection(const Eigen::VectorXd& conf,const int& conn_idx);

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
   * @brief Get the change warp vector.
   *
   * This method retrieves the vector indicating changes in the warp status along the path.
   * Each element in the vector corresponds to a connection in the path, and a value of `true`
   * indicates that the warp has changed at that connection.
   *
   * @return A vector of boolean values indicating changes in warp status.
   */
  std::vector<bool> getChangeWarp()
  {
    return change_warp_;
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
   * @brief Set the change warp vector for the path.
   *
   * This method sets the change warp vector, which indicates whether each connection in the path has undergone a change.
   *
   * @param change_warp A vector of boolean values representing the change warp for each connection.
   */
  void setChangeWarp(const std::vector<bool>& change_warp)
  {
    change_warp_ = change_warp;
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

  PathPtr clone();
  bool onLine(double toll = 1e-06);
  void setConnections(const std::vector<ConnectionPtr>& conn);
  bool splitConnection(const ConnectionPtr& conn1, const ConnectionPtr& conn2, const std::vector<ConnectionPtr>::iterator &it);
  bool splitConnection(const ConnectionPtr& conn1, const ConnectionPtr& conn2, const ConnectionPtr& conn);
  bool restoreConnection(const ConnectionPtr& conn, const NodePtr& node2remove);
  bool simplify(const double& distance = 0.02);
  bool isValid(const CollisionCheckerPtr &this_checker = nullptr);
  bool isValidFromConf(const Eigen::VectorXd &conf, const CollisionCheckerPtr &this_checker = nullptr);
  bool isValidFromConf(const Eigen::VectorXd &conf, const int& conn_idx, const CollisionCheckerPtr &this_checker = nullptr);
  bool isValidFromConf(const Eigen::VectorXd &conf, int &pos_closest_obs_from_goal, const CollisionCheckerPtr &this_checker = nullptr);
  bool isValidFromConf(const Eigen::VectorXd &conf, const int &conn_idx, int &pos_closest_obs_from_goal, const CollisionCheckerPtr &this_checker = nullptr);
  bool isValidFromConn(const ConnectionPtr &this_conn, const CollisionCheckerPtr &this_checker = nullptr);
  Eigen::VectorXd projectOnClosestConnection(const Eigen::VectorXd& point, const bool verbose = false);
  Eigen::VectorXd projectKeepingAbscissa(const Eigen::VectorXd& point, const Eigen::VectorXd &past_prj, const double &weight = 0.5, const bool& verbose = false);
  Eigen::VectorXd projectOnPath(const Eigen::VectorXd& point, const bool& verbose = false);
  Eigen::VectorXd projectOnPath(const Eigen::VectorXd& point, const Eigen::VectorXd &past_projection, const bool& verbose = false);
  Eigen::VectorXd projectOnPath(const Eigen::VectorXd& point, const Eigen::VectorXd &past_projection, ConnectionPtr &conn, const bool& verbose = false);


  /**
   * @brief Attempt to warp the path by adjusting configurations along connections.
   *
   * This function attempts to warp the path by adjusting configurations along connections
   * where the norm of the connection is greater than the specified minimum distance. The
   * warping process is controlled by the change_warp_ vector, and the maximum time limit
   * for warping is given by max_time.
   *
   * @param min_dist The minimum distance threshold for considering a connection for warping.
   * @param max_time The maximum time limit for the warping process (set to 0 for no time limit).
   * @return Returns true if any warping changes were made, false otherwise.
   */
  bool warp(const double& min_dist = 0.1, const double& max_time = std::numeric_limits<double>::infinity());

  void flip();

  XmlRpc::XmlRpcValue toXmlRpcValue(bool reverse=false) const;
  friend std::ostream& operator<<(std::ostream& os, const Path& path);
};

std::ostream& operator<<(std::ostream& os, const Path& path);
}
