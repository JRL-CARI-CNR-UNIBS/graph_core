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

#include <Eigen/Core>
#include <graph_core/graph/connection.h>
#include <graph_core/util.h>

namespace graph {
namespace core {
/**
 * @class Node
 * @brief Class for defining node of a graph.
 */
class Node : public std::enable_shared_from_this<Node> {
  friend class Connection;

protected:
  /**
   * @brief Configuration vector of the node.
   *
   * This member variable represents the configuration vector associated with
   * the node.
   */
  Eigen::VectorXd configuration_;

  /**
   * @brief Number of degrees of freedom (ndof) of the node.
   *
   * This member variable represents the number of degrees of freedom associated
   * with the node.
   */
  unsigned int ndof_;

  /**
   * @brief Vector of weak pointers to parent connections.
   *
   * This member variable represents a vector of weak pointers to parent
   * connections of the node.
   */
  std::vector<ConnectionWeakPtr>
      parent_connections_; // Weak ptr to avoid pointers cycles

  /**
   * @brief Vector of weak pointers to net parent connections.
   *
   * This member variable represents a vector of weak pointers to net parent
   * connections of the node.
   */
  std::vector<ConnectionWeakPtr>
      net_parent_connections_; // Weak ptr to avoid pointers cycles

  /**
   * @brief Vector of pointers to child connections.
   *
   * This member variable represents a vector of pointers to child connections
   * of the node.
   */
  std::vector<ConnectionPtr> child_connections_;

  /**
   * @brief Vector of pointers to net child connections.
   *
   * This member variable represents a vector of pointers to net child
   * connections of the node.
   */
  std::vector<ConnectionPtr> net_child_connections_;

  /**
   * @brief Vector of boolean flags.
   *
   * This member variable represents a vector of boolean flags associated with
   * the node. You can add new flags specific to your algorithm using function
   * setFlag and passing the vector-index to store the flag.
   * getReservedFlagsNumber allows you to know how many positions are reserved
   * for the defaults. setFlag doesn't allow you to overwrite these positions.
   * To overwrite them, use the flag-specific functions.
   */
  std::vector<bool> flags_;

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
   * @brief Adds a parent connection to the node.
   *
   * This function adds the provided connection to the list of parent
   * connections for the node. The connection is added only if its child matches
   * the current node.
   *
   * @param connection The ConnectionPtr representing the parent connection to
   * be added.
   */
  void addParentConnection(const ConnectionPtr &connection);

  /**
   * @brief Adds a child connection to the node.
   *
   * This function adds the provided connection to the list of child connections
   * for the node. The connection is added only if its parent matches the
   * current node.
   *
   * @param connection The ConnectionPtr representing the child connection to be
   * added.
   */
  void addChildConnection(const ConnectionPtr &connection);

  /**
   * @brief Adds a net parent connection to the node.
   *
   * This function adds the provided connection to the list of net parent
   * connections for the node. The connection is added only if its child matches
   * the current node.
   *
   * @param connection The ConnectionPtr representing the net parent connection
   * to be added.
   */
  void addNetParentConnection(const ConnectionPtr &connection);

  /**
   * @brief Adds a net child connection to the node.
   *
   * This function adds the provided connection to the list of net child
   * connections for the node. The connection is added only if its parent
   * matches the current node.
   *
   * @param connection The ConnectionPtr representing the net child connection
   * to be added.
   */
  void addNetChildConnection(const ConnectionPtr &connection);

  /**
   * @brief Removes the specified parent connection from the node.
   *
   * This function removes the specified parent connection from the node. It
   * searches for the connection in the parent connections vector and removes
   * it. Additionally, it marks the connection's child flag as not valid.
   *
   * @param connection The ConnectionPtr representing the parent connection to
   * be removed.
   */
  void removeParentConnection(const ConnectionPtr &connection);
  void removeParentConnection(
      const std::vector<ConnectionWeakPtr>::iterator &it_conn);
  /**
   * @brief Removes the specified child connection from the node.
   *
   * This function removes the specified child connection from the node. It
   * searches for the connection in the child connections vector and removes it.
   * Additionally, it marks the connection's parent flag as not valid.
   *
   * @param connection The ConnectionPtr representing the child connection to be
   * removed.
   */
  void removeChildConnection(const ConnectionPtr &connection);
  void
  removeChildConnection(const std::vector<ConnectionPtr>::iterator &it_conn);

  /**
   * @brief Removes the specified net parent connection from the node.
   *
   * This function removes the specified net parent connection from the node. It
   * searches for the connection in the net parent connections vector and
   * removes it. Additionally, it marks the connection's child flag as not
   * valid.
   *
   * @param connection The ConnectionPtr representing the net parent connection
   * to be removed.
   */
  void removeNetParentConnection(const ConnectionPtr &connection);
  void removeNetParentConnection(
      const std::vector<ConnectionWeakPtr>::iterator &it_conn);

  /**
   * @brief Removes the specified net child connection from the node.
   *
   * This function removes the specified net child connection from the node. It
   * searches for the connection in the net child connections vector and removes
   * it. Additionally, it marks the connection's parent flag as not valid.
   *
   * @param connection The ConnectionPtr representing the net child connection
   * to be removed.
   */
  void removeNetChildConnection(const ConnectionPtr &connection);
  void
  removeNetChildConnection(const std::vector<ConnectionPtr>::iterator &it_conn);

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * @brief Map of properties associated with the node.
   *
   * This member variable represents a map of propeFties associated with the
   * node. The map uses strings as keys and std::any as values to store
   * heterogeneous data types. Store in properties_ any object you need to
   * customize the node.
   */
  std::unordered_map<std::string, std::any> properties_;

  /**
   * @brief Constructor for the Node class.
   *
   * This constructor initializes a Node object with the provided configuration
   * vector. It sets the configuration and calculates the number of degrees of
   * freedom (ndof). Default flags are inserted into the 'flags_' vector at this
   * point.
   *
   * @param configuration The Eigen::VectorXd representing the configuration of
   * the node.
   */

  Node(const Eigen::VectorXd &configuration);
  Node(const Eigen::VectorXd &configuration,
       const cnr_logger::TraceLoggerPtr &logger);

  /**
   * @brief Retrieves a shared pointer to the Node.
   *
   * This function returns a shared pointer to the current Node instance using
   * the shared_from_this() method.
   *
   * @return Returns a shared pointer to the current Node.
   */
  NodePtr pointer() { return shared_from_this(); }

  /**
   * Add here your reserved flags.
   * Example:
   * const static unsigned int idx_your_custom_flag1_ = 0;
   * const static unsigned int idx_your_custom_flag2_ = 1;
   *
   * Increment number_reserved_flags_ accordingly!
   * Initialize flags_ in the constructor accordingly!
   * If you need to modify or read these flags externally, implement getter and
   * setter functions!
   */

  /**
   * @brief Number of reserved flags for the Node class.
   *
   * This static constexpr member variable represents the number of reserved
   * flags for the Node class.
   */
  static constexpr unsigned int number_reserved_flags_ = 0;
  // NO DEFAULT FLAGS SO FAR

  /**
   * @brief Sets the value of a flag at the specified index.
   *
   * This function sets the value of a flag at the specified index. If the index
   * is equal to the current number of flags, a new flag is added to the end of
   * the flags list. If the index is less than the current number of flags, the
   * function checks whether it is attempting to overwrite a default flag (with
   * an index less than 'number_reserved_flags_'). If so, an error message is
   * logged, and the function returns false. Otherwise, the value of the
   * existing flag is updated.
   *
   * @param idx The index at which to set the flag.
   * @param flag The value to set for the flag.
   * @return Returns true if the flag is successfully set, and false otherwise.
   */
  bool setFlag(const size_t &idx, const bool flag);

  /**
   * @brief Sets a new flag with the provided value and returns its index.
   *
   * This function sets a new flag with the provided value and returns its
   * index. The new flag is added to the end of the flags list, and its index is
   * equal to the current number of flags.
   *
   * @param flag The value to set for the new flag.
   * @return Returns the index of the newly added flag.
   */
  unsigned int setFlag(const bool flag);

  /**
   * @brief Retrieves the value of the flag at the specified index.
   *
   * This function retrieves the value of the flag at the specified index. If
   * the index is within the range of existing flags, the corresponding flag
   * value is returned. If the index is beyond the range of existing flags, the
   * provided default value is returned.
   *
   * @param idx The index of the flag to retrieve.
   * @param default_value The default value to return if the flag at the
   * specified index does not exist.
   * @return Returns the value of the flag at the specified index or the default
   * value if the index is out of range.
   */
  bool getFlag(const size_t &idx, const bool default_value);

  /**
   * @brief Retrieves a pointer to the TraceLogger associated with the node.
   *
   * This member function provides read-only access to the TraceLogger instance
   * associated with the node, allowing external components to access and
   * utilize the logging capabilities.
   *
   * @return A constant reference to the TraceLogger pointer.
   */
  const cnr_logger::TraceLoggerPtr &getLogger() const { return logger_; }

  /**
   * @brief Sets the TraceLogger associated with the node.
   */
  void setLogger(const cnr_logger::TraceLoggerPtr &logger) { logger_ = logger; }

  /**
   * @brief Retrieves the number of parent connections for the node.
   *
   * @return Returns the number of parent connections associated with the node.
   */
  const size_t getParentConnectionsSize() const;

  /**
   * @brief Retrieves the number of net parent connections for the node.
   *
   * @return Returns the number of net parent connections associated with the
   * node.
   */
  const size_t getNetParentConnectionsSize() const;

  /**
   * @brief Retrieves the number of child connections for the node.
   *
   * @return Returns the number of child connections associated with the node.
   */
  const size_t getChildConnectionsSize() const;

  /**
   * @brief Retrieves the number of net child connections for the node.
   *
   * @return Returns the number of net child connections associated with the
   * node.
   */
  const size_t getNetChildConnectionsSize() const;

  /**
   * @brief Retrieves the i-th parent connection associated with the node.
   *
   * This function retrieves the i-th parent connection associated with the
   * node.
   *
   * @param i The index of the parent connection to retrieve.
   * @return Returns a shared pointer to the i-th parent connection.
   */
  ConnectionPtr parentConnection(const int &i) const;

  /**
   * @brief Retrieves the i-th net parent connection associated with the node.
   *
   * This function retrieves the i-th net parent connection associated with the
   * node.
   *
   * @param i The index of the net parent connection to retrieve.
   * @return Returns a shared pointer to the i-th net parent connection.
   */
  ConnectionPtr netParentConnection(const int &i) const;

  /**
   * @brief Retrieves the i-th child connection associated with the node.
   *
   * This function retrieves the i-th child connection associated with the node.
   *
   * @param i The index of the child connection to retrieve.
   * @return Returns a shared pointer to the i-th child connection.
   */
  ConnectionPtr childConnection(const int &i) const;

  /**
   * @brief Retrieves the i-th net child connection associated with the node.
   *
   * This function retrieves the i-th net child connection associated with the
   * node.
   *
   * @param i The index of the net child connection to retrieve.
   * @return Returns a shared pointer to the i-th net child connection.
   */
  ConnectionPtr netChildConnection(const int &i) const;

  /**
   * @brief Retrieves the child nodes associated with the node.
   *
   * This function retrieves the child nodes associated with the node through
   * its child connections. If the node has no child connections, an empty
   * vector is returned. Otherwise, a vector of shared pointers to the child
   * nodes is returned.
   *
   * @return Returns a vector of shared pointers to the child nodes associated
   * with the node.
   */
  std::vector<NodePtr> getChildren() const;

  /**
   * @brief Retrieves the parent nodes associated with the node.
   *
   * This function retrieves the parent nodes associated with the node through
   * its parent connections. If the node has no parent connections, an empty
   * vector is returned. Otherwise, a vector of shared pointers to the parent
   * nodes is returned.
   *
   * @return Returns a vector of shared pointers to the parent nodes associated
   * with the node.
   */
  std::vector<NodePtr> getParents() const;

  /**
   * @brief Retrieves the net parent nodes associated with the node.
   *
   * This function retrieves the net parent nodes associated with the node
   * through its net parent connections. If the node has no net parent
   * connections, an empty vector is returned. Otherwise, a vector of shared
   * pointers to the net parent nodes is returned.
   *
   * @return Returns a vector of shared pointers to the net parent nodes
   * associated with the node.
   */
  std::vector<NodePtr> getNetParents() const;

  /**
   * @brief Retrieves the net child nodes associated with the node.
   *
   * This function retrieves the net child nodes associated with the node
   * through its net child connections. If the node has no net child
   * connections, an empty vector is returned. Otherwise, a vector of shared
   * pointers to the net child nodes is returned.
   *
   * @return Returns a vector of shared pointers to the net child nodes
   * associated with the node.
   */
  std::vector<NodePtr> getNetChildren() const;

  /**
   * @brief Retrieves the parent connections associated with the node.
   *
   * This function retrieves the parent connections associated with the node and
   * returns them as a vector of shared pointers. If the node has no parent
   * connections, an empty vector is returned.
   *
   * @return Returns a vector of shared pointers to the parent connections
   * associated with the node.
   */
  std::vector<ConnectionPtr> getParentConnections() const;

  /**
   * @brief Retrieves the net parent connections associated with the node.
   *
   * This function retrieves the net parent connections associated with the node
   * and returns them as a vector of shared pointers. If the node has no net
   * parent connections, an empty vector is returned.
   *
   * @return Returns a vector of shared pointers to the net parent connections
   * associated with the node.
   */
  std::vector<ConnectionPtr> getNetParentConnections() const;

  /**
   * @brief Retrieves the child connections associated with the node.
   *
   * This function retrieves the child connections associated with the node and
   * returns them as a vector of shared pointers. If the node has no child
   * connections, an empty vector is returned.
   *
   * @return Returns a vector of shared pointers to the child connections
   * associated with the node.
   */
  std::vector<ConnectionPtr> getChildConnections() const;

  /**
   * @brief Retrieves the net child connections associated with the node.
   *
   * This function retrieves the net child connections associated with the node
   * and returns them as a vector of shared pointers. If the node has no net
   * child connections, an empty vector is returned.
   *
   * @return Returns a vector of shared pointers to the net child connections
   * associated with the node.
   */
  std::vector<ConnectionPtr> getNetChildConnections() const;

  /**
   * @brief Disconnects all child connections of the node.
   *
   * This function disconnects all child connections of the node. For each child
   * connection, it checks if the connection and its child are valid. If valid,
   * it removes the parent connection from the child node.
   */
  void disconnectChildConnections();

  /**
   * @brief Disconnects all parent connections of the node.
   *
   * This function disconnects all parent connections of the node. For each
   * parent connection, it checks if the connection and its parent are valid. If
   * valid, it removes the child connection from the parent node.
   */
  void disconnectParentConnections();

  /**
   * @brief Disconnects all net parent connections of the node.
   *
   * This function disconnects all net parent connections of the node. For each
   * net parent connection, it checks if the connection and its parent are
   * valid. If valid, it removes the net child connection from the parent node.
   */
  void disconnectNetParentConnections();

  /**
   * @brief Disconnects all net child connections of the node.
   *
   * This function disconnects all net child connections of the node. For each
   * net child connection, it checks if the connection and its child are valid.
   * If valid, it removes the net parent connection from the child node.
   */
  void disconnectNetChildConnections();

  /**
   * @brief Disconnects all connections of the node.
   *
   * This function disconnects all parent, net parent, child, and net child
   * connections of the node by calling their respective disconnect methods.
   */
  void disconnect();

  /**
   * @brief Switches a parent net connection with the existing parent connection
   * of the node.
   *
   * This function switches a parent net connection with the existing parent
   * connection of the node. It checks if the provided net connection is a
   * parent net connection of the node. If not, it returns false. Otherwise, it
   * performs the switch by converting the existing parent connection to a
   * parent net connection and the provided net connection to a regular parent
   * connection.
   *
   * @param net_connection The ConnectionPtr representing the net parent
   * connection to be switched.
   * @return Returns true if the switch is successful, false otherwise.
   */
  bool switchParentConnection(const ConnectionPtr &net_connection);

  /**
   * @brief Retrieves the configuration vector of the node.
   *
   * This function retrieves and returns the configuration vector associated
   * with the node.
   *
   * @return Returns a constant reference to the Eigen::VectorXd representing
   * the configuration of the node.
   */
  const Eigen::VectorXd &getConfiguration() { return configuration_; }

  /**
   * @brief Retrieves the number of reserved flags for the node.
   *
   * This function retrieves and returns the number of reserved flags for the
   * node.
   *
   * @return Returns an unsigned integer representing the number of reserved
   * flags for the node.
   */
  static unsigned int getReservedFlagsNumber();

  /**
   * @brief Destructor for the Node class.
   *
   * This destructor is responsible for cleaning up resources associated with
   * the Node object, including disconnecting parent, child, net parent, and net
   * child connections.
   */
  ~Node();

  /**
   *@brief Convert the Node's configuration to a YAML::Node.
   *
   * This function converts the configuration of the Node to a YAML::Node.
   * It creates a YAML sequence with each element representing a configuration
   *value.
   *
   * @return A YAML::Node representing the Node's configuration.
   */
  YAML::Node toYAML() const;

  /**
   * @brief Create a NodePtr from a YAML::Node.
   *
   * This static function creates a NodePtr (shared pointer to a Node) from a
   * YAML::Node. It expects the YAML node to be a sequence, where each element
   * represents a configuration value. The function extracts the configuration
   * values and constructs a Node with the specified configuration.
   *
   * @param yaml The YAML::Node containing the sequence of configuration values.
   * @param logger The TraceLoggerPtr for logging error messages.
   * @return A shared pointer to the constructed Node.
   *  If the YAML::Node is not a sequence or if an error occurs during
   * construction, returns nullptr.
   */
  static NodePtr fromYAML(const YAML::Node &yaml,
                          const cnr_logger::TraceLoggerPtr &logger);
  static NodePtr fromYAML(const YAML::Node &yaml, std::string &what);

  friend std::ostream &operator<<(std::ostream &os, const Node &node);
};

/**
 * @brief Overloaded stream insertion operator for Node.
 *
 * This operator outputs information about the Node to the provided output
 * stream.
 *
 * @param os The output stream to which the Node information is written.
 * @param node The Node object to be output.
 * @return Returns the modified output stream.
 */
std::ostream &operator<<(std::ostream &os, const Node &node);

/**
 * @brief Template function to get a graph::core::Node from the parameter
 * server.
 *
 * This function attempts to retrieve a parameter named `param_name` from the
 * parameter namespace `param_ns`, and assigns its value to the pointer `param`.
 * If the parameter is found, it is set to the pointer and the function returns
 * true. If the parameter is not found, a warning is logged and the function
 * returns false. If an error occurs while retrieving the parameter, an error is
 * logged and an invalid_argument exception is thrown.
 *
 * @tparam NodePtr The pointer type to which the parameter value will be
 * assigned.
 * @param logger A pointer to a TraceLogger object used for logging messages.
 * @param param_ns The namespace of the parameter on the parameter server.
 * @param param_name The name of the parameter to retrieve.
 * @param param Reference to the pointer where the parameter value will be
 * assigned.
 * @return True if the parameter is successfully retrieved and assigned, false
 * otherwise.
 * @throws std::invalid_argument if an error occurs while retrieving the
 * parameter.
 */
template <>
inline bool get_param<NodePtr>(const cnr_logger::TraceLoggerPtr &logger,
                               const std::string &param_ns,
                               const std::string &param_name, NodePtr &param);

} // end namespace core
} // end namespace graph
