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

#include <graph_core/graph/node.h>

namespace graph
{
namespace core
{

/**
 * @class Connection
 * @brief Class for defining connection between nodes of a graph.
 * Connection can be standard or net connections:
 *  - standard: connections of a tree -> each node has at max 1 standard parent connection.
 *  - net: do not belong to the tree  -> a node can have multiple net parent connections.
 *         The tree can not see these connections.
 */
class Connection: public std::enable_shared_from_this<Connection>
{
  friend class Node;

protected:

  /**
   * @brief Weak pointer to the parent node.
   */
  NodeWeakPtr parent_;

  /**
   * @brief Shared pointer to the child node.
   */
  NodePtr child_;

  /**
   * @brief Cost associated with the connection.
   */
  double cost_;

  /**
   * @brief Euclidean norm of the connection.
   */
  double euclidean_norm_;

  /**
   * @brief Time of the last cost update for the connection.
   */
  std::chrono::time_point<std::chrono::system_clock> time_cost_update_;

  /**
   * @brief Likelihood associated with the connection.
   */
  double likelihood_;

  /**
   * @brief Pointer to a TraceLogger instance for logging.
   *
   * This member variable represents a pointer to a TraceLogger instance, allowing
   * to perform logging operations. TraceLogger is a part of the cnr_logger library.
   * Ensure that the logger is properly configured and available for use.
   */
  cnr_logger::TraceLoggerPtr logger_;

  /**
   * @brief Vector of boolean flags.
   *
   * This member variable represents a vector of boolean flags associated with the connection.
   * By default, the first three positions are reserved for valid flag, net flag and recently checked flag.
   * You can add new flags specific to your algorithm using function setFlag and passing the vector-index to store the flag.
   * getReservedFlagsNumber allows you to know how many positions are reserved for the defaults.
   * setFlag doesn't allow you to overwrite these positions. To overwrite them, use the flag-specific functions.
   */
  std::vector<bool> flags_;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * Add here your reserved flags.
   * Increment number_reserved_flags_ accordingly!
   * Initialize flags_ in the constructor accordingly!
   * If you need to modify or read these flags externally, implement getter and setter functions!
   * If you want, print your flag when << operator is called on a connection.
   */

  /**
   * @brief Index of the parent node's validity flag in the flags_ array.
   */
  static constexpr unsigned int idx_parent_valid_ = 0;

  /**
   * @brief Index of the child node's validity flag in the flags_ array.
   */
  static constexpr unsigned int idx_child_valid_ = 1;

  /**
   * @brief Index of the net connection flag in the flags_ array.
   */
  static constexpr unsigned int idx_net_ = 2;

  /**
   * @brief Index of the recently checked flag in the flags_ array.
   */
  static constexpr unsigned int idx_recently_checked_ = 3;

  /**
   * @brief Number of reserved flags in the flags_ array.
   */
  static constexpr unsigned int number_reserved_flags_ = 4;

  /**
   * @brief Map of properties associated with the connection.
   *
   * This member variable represents a map of properties associated with the connection.
   * The map uses strings as keys and std::any as values to store heterogeneous data types.
   * Store in properties_ any object you need to customize the connection.
   */
  std::unordered_map<std::string, std::any> properties_;

  /**
   * @brief Constructor for the Connection class.
   *
   * This constructor initializes a Connection object with the provided parent and child Nodes.
   * It calculates the Euclidean norm between the parent and child configurations and sets deafult values for other members.
   * The flags are set to {false, is_net, false} for valid, is_net, and recently_checked.
   *
   * @param parent The NodePtr to the parent Node.
   * @param child The NodePtr to the child Node.
   * @param logger The cnr_logger::TraceLoggerPtr logger for logging operations.
   * @param is_net Boolean indicating if the connection is a net connection.
   */
  Connection(const NodePtr& parent, const NodePtr& child, const cnr_logger::TraceLoggerPtr& logger, const bool is_net = false);

  /**
   * @brief Returns a shared pointer to the Connection.
   *
   * This function returns a shared pointer to the Connection using shared_from_this().
   *
   * @return Returns a shared pointer to the Connection.
   */
  ConnectionPtr pointer()
  {
    return shared_from_this();
  }

  /**
   * @brief Checks if the Connection is a net connection.
   *
   * @return Returns true if the Connection is a net connection, false otherwise.
   */
  bool isNet() const
  {
    return flags_[idx_net_];
  }

  /**
   * @brief Checks if the Connection has been recently checked.
   *
   * @return Returns true if the Connection has been recently checked, false otherwise.
   */
  bool isRecentlyChecked() const
  {
    return flags_[idx_recently_checked_];
  }

  /**
   * @brief Sets the recently checked status of the Connection.
   *
   * @param checked Boolean indicating the recently checked status to be set.
   */
  void setRecentlyChecked(bool checked)
  {
    flags_[idx_recently_checked_] = checked;
  }

  /**
   * @brief Checks if the Connection is valid, i.e. whether both the parent node and the child node are aware of this connection.
   *
   * @return Returns true if the Connection is valid, false otherwise.
   */
  bool isValid() const
  {
    return (flags_[idx_parent_valid_] && flags_[idx_child_valid_]);
  }

  /**
   * @brief Sets the cost of the Connection and updates the time of the cost update.
   *
   * @param cost The cost value to be set for the Connection.
   */
  void setCost(const double& cost)
  {
    cost_ = cost;
    time_cost_update_ = std::chrono::system_clock::now();
  }

  /**
   * @brief Gets the cost of the Connection.
   *
   * @return Returns a reference to the cost value of the Connection.
   */
  const double& getCost() const
  {
    return cost_;
  }

  /**
   * @brief Gets the time of the last cost update for the Connection.
   *
   * @return Returns a reference to the time of the last cost update for the Connection.
   */
  const std::chrono::time_point<std::chrono::system_clock>& getTimeCostUpdate() const
  {
    return time_cost_update_;
  }

  /**
   * @brief setTimeCostUpdate sets the time when the connection cost is updated.
   *  E.g., if you clone a path, you may want to set the same time_cost_update_ of the connections of the original path
   * @param time
   */
  /**
   * @brief Sets the time of the last cost update for the Connection.
   *
   * E.g., if you clone a path, you may want to set the same time_cost_update_ of the connections of the original path
   *
   * @param time The time value to be set for the last cost update.
   */
  void setTimeCostUpdate(const std::chrono::time_point<std::chrono::system_clock>& time)
  {
    time_cost_update_ = time;
  }

  /**
   * @brief Gets the Euclidean norm of the Connection.
   *
   * @return Returns the Euclidean norm of the Connection.
   */
  double norm() const
  {
    return euclidean_norm_;
  }

  /**
   * @brief Retrieves a pointer to the TraceLogger associated with the node.
   *
   * This member function provides read-only access to the TraceLogger instance associated
   * with the node, allowing external components to access and utilize the logging capabilities.
   *
   * @return A constant reference to the TraceLogger pointer.
   */
  const cnr_logger::TraceLoggerPtr& getLogger() const
  {
    return logger_;
  }

  /**
   * @brief Gets the parent Node of the Connection.
   *
   * @return Returns a shared pointer to the parent Node of the Connection.
   */
  NodePtr getParent() const
  {
    //    assert(not parent_.expired());
    return parent_.lock();
  }

  /**
   * @brief Gets the child Node of the Connection.
   *
   * @return Returns a shared pointer to the child Node of the Connection.
   */
  NodePtr getChild() const
  {
    return child_;
  }

  /**
   * @brief Sets the value of a flag at the specified index.
   *
   * This function sets the value of a flag at the specified index. If the index is equal to the current
   * number of flags, a new flag is added to the end of the flags list. If the index is less than the current
   * number of flags, the function checks whether it is attempting to overwrite a default flag (with an index
   * less than 'number_reserved_flags_'). If so, an error message is logged, and the function returns false.
   * Otherwise, the value of the existing flag is updated.
   *
   * @param idx The index at which to set the flag.
   * @param flag The value to set for the flag.
   * @return Returns true if the flag is successfully set, and false otherwise.
   */
  bool setFlag(const size_t &idx, const bool flag);

  /**
   * @brief Sets a new flag with the provided value and returns its index.
   *
   * This function sets a new flag with the provided value and returns its index. The new flag is added to
   * the end of the flags list, and its index is equal to the current number of flags.
   *
   * @param flag The value to set for the new flag.
   * @return Returns the index of the newly added flag.
   */
  unsigned int setFlag(const bool flag);

  /**
   * @brief Sets the likelihood value for the Connection.
   *
   * @param likelihood The likelihood value to be set.
   */
  void setLikelihood(const double& likelihood){likelihood_=likelihood;}

  /**
   * @brief Retrieves the value of the flag at the specified index.
   *
   * This function retrieves the value of the flag at the specified index. If the index is within the
   * range of existing flags, the corresponding flag value is returned. If the index is beyond the range
   * of existing flags, the provided default value is returned.
   *
   * @param idx The index of the flag to retrieve.
   * @param default_value The default value to return if the flag at the specified index does not exist.
   * @return Returns the value of the flag at the specified index or the default value if the index is out of range.
   */
  bool getFlag(const size_t& idx, const bool default_value);

  /**
   * @brief Adds the Connection to the corresponding nodes' connection vectors.
   *
   * This function sets the valid flag and adds the Connection to the connection vectors
   * of the parent and child nodes, depending on whether it is a net connection or not.
   */
  void add();

  /**
   * @brief Adds the Connection to the corresponding nodes' connection vectors.
   *
   * This function sets the valid flag, sets the net flag, and adds the Connection to the connection vectors
   * of the parent and child nodes, depending on whether it is a net connection or not.
   *
   * @param is_net A boolean indicating whether the Connection is a net connection.
   */
  void add(const bool is_net);

  /**
   * @brief Removes the Connection from the corresponding nodes' connection vectors.
   *
   * This function resets the valid flag and removes the Connection from the connection vectors
   * of the parent and child nodes.
   */
  void remove();

  /**
   * @brief Flips the direction of the Connection by swapping parent and child nodes.
   *
   * This function removes the Connection from the parent and child nodes, swaps the parent and child
   * pointers, and adds the new Connection between the swapped parent and child nodes.
   */
  void flip();

  /**
   * @brief Converts the Connection to a regular (non-net) connection.
   *
   * This function converts the Connection to a regular connection.
   *
   * @return Returns true if the Connection is successfully converted to a regular connection, false otherwise.
   */
  bool convertToConnection();

  /**
   * @brief Converts the Connection to a net connection.
   *
   * This function converts the Connection to a net connection.
   *
   * @return Returns true if the Connection is successfully converted to a net connection, false otherwise.
   */
  bool convertToNetConnection();

  /**
   * @brief Changes the type of the Connection.
   *
   * This function changes the type of the Connection. If the current type is a net connection,
   * it converts it to a regular connection, and vice versa.
   */
  void changeConnectionType();

  /**
   * @brief Checks if two connections are parallel within a specified tolerance.
   *
   * This function checks if two connections are parallel by comparing their dot product with the product
   * of their length. The connections are considered parallel if the the difference between these two values is less than toll.
   *
   * @param conn The ConnectionPtr to compare with.
   * @param toll Tolerance for the comparison.
   * @return Returns true if the connections are parallel, false otherwise.
   */
  bool isParallel(const ConnectionPtr& conn, const double& toll = 1e-06);

  /**
   * @brief Projects a point onto the connection and calculates the distance from the point to the connection.
   *
   * This function projects a given point onto the connection defined by its parent and child nodes.
   * It also calculates the distance from the point to the connection.
   *
   * @param point The point to be projected onto the connection.
   * @param distance Output parameter for the distance from the point to the connection.
   * @param in_conn Output parameter indicating if the projection is within the connection (true) or not (false).
   * @param verbose Flag to enable verbose logging (default is false).
   * @return Returns the projected point on the connection.
   */
  Eigen::VectorXd projectOnConnection(const Eigen::VectorXd& point, double& distance, bool& in_conn, const bool& verbose = false);

  /**
   * @brief Gets the number of reserved flags for the Connection class.
   *
   * This static function returns the number of reserved flags for the Connection class.
   * Use this function to know where you can save new flags.
   *
   * @return Returns the number of reserved flags.
   */
  static unsigned int getReservedFlagsNumber()
  {
    return number_reserved_flags_;
  }

  /**
   * @brief Destructor for the Connection class.
   */
  ~Connection();

  friend std::ostream& operator<<(std::ostream& os, const Connection& connection);
};

/**
 * @brief Overloaded stream insertion operator for the Connection class.
 *
 * This operator prints information about the Connection object to the output stream, including parent and child configurations,
 * cost, length, net status, and flags.
 *
 * @param os The output stream.
 * @param connection The Connection object to be printed.
 * @return The modified output stream.
 */
std::ostream& operator<<(std::ostream& os, const Connection& connection);

} //end namespace core
} //end namespace graph
