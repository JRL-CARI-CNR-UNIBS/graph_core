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

#include <map>
#include <any>
#include <ros/ros.h>
#include <eigen3/Eigen/Core>
#include <graph_core/util.h>
#include <graph_core/graph/connection.h>

namespace pathplan
{
class Node: public std::enable_shared_from_this<Node>
{
  friend class Connection;

protected:
  Eigen::VectorXd configuration_;
  unsigned int ndof_;

  std::vector<ConnectionWeakPtr> parent_connections_;     //Weak ptr to avoid pointers cycles
  std::vector<ConnectionWeakPtr> net_parent_connections_; //Weak ptr to avoid pointers cycles

  std::vector<ConnectionPtr> child_connections_;
  std::vector<ConnectionPtr> net_child_connections_;

  /**
   * @brief flags_ is a vector of flags.You can add new flags specific to your algorithm using function setFlag and passing the vector-index to store the flag.
   * getReservedFlagsNumber allows you to know how many positions are reserved for the defaults. setFlag doesn't allow you to overwrite these positions.
   * To overwrite them, you should use the flag-specific functions.
   */
  std::vector<bool> flags_;

  /**
     * @brief addParentConnection adds a parent connection to the node
     * @param connection is the parent connection to add (the connection's child should be this node)
     */
  void addParentConnection(const ConnectionPtr& connection);

  /**
     * @brief addChildConnection adds a child connection to the node
     * @param connection is the child connection to add (the connection's parent should be this node)
     */
  void addChildConnection(const ConnectionPtr& connection);

  /**
     * @brief addNetParentConnection adds a net parent connection to the node
     * @param connection is the net parent connection to add (the connection's child should be this node)
     */
  void addNetParentConnection(const ConnectionPtr& connection);

  /**
     * @brief addNetChildConnection adds a net child connection to the node
     * @param connection is the net child connection to add (the connection's parent should be this node)
     */
  void addNetChildConnection(const ConnectionPtr& connection);

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * @brief properties_ is a map of generic type objects. Store in properties_ any object you need to customize the node
   */
  std::map<std::string, std::vector<std::any>> properties_;

  /**
   * @brief Node is the constructor
   * @param configuration is the node's configuration, as an Eigen::VectorXd object
   */
  Node(const Eigen::VectorXd& configuration);

  /**
   * @brief pointer returns a std::shared_pointer to this node
   * @return
   */
  NodePtr pointer()
  {
    return shared_from_this();
  }

  /**
   * Add here your reserved flags.
   * Example:
   * const static unsigned int idx_your_custom_flag1_ = 0;
   * const static unsigned int idx_your_custom_flag2_ = 1;
   *
   * Increment number_reserved_flags_ accordingly!
   * Initialize flags_ in the constructor accordingly!
   * If you need to modify or read these flags externally, implement getter and setter functions!
   */

  //NO DEFAULT FLAGS SO FAR

  static constexpr unsigned int number_reserved_flags_ = 0;

  /**
     * @brief setFlag sets your custome flag. Use this function only if the flag was not created before because it creates a new one flag in flags_ vector
     * @param flag the NEW flag to set
     * @return the index of the flag in flags_ vector to use when you want to change the value
     */
  unsigned int setFlag(const bool flag);

  /**
     * @brief setFlag sets the custom flag at index idx. The flag should be already present in the flags_ vector.
     * If not, it add the new flag if and only if the index idx is equal to flags_ size
     * @param idx the index of the flag
     * @param flag the value of the flag
     * @return true if the flag is set correctly, flase otherwise
     */
  bool setFlag(const int& idx, const bool flag);

  /**
     * @brief getFlag returns the value of the flag at position idx. It returns the value if the flag exists, otherwise return the default value.
     * @param idx the index of the flag you are asking for.
     * @param default_value the default value returned if the flag doesn't exist.
     * @return the flag if it exists, the default value otherwise.
     */
  bool getFlag(const int& idx, const bool default_value);

  /**
     * @brief getParentConnectionsSize returns the number of parent connections
     * @return the number of parent connections
     */
  const int getParentConnectionsSize() const;

  /**
     * @brief getNetParentConnectionsSize returns the number of net parent connections
     * @return the number of net parent connections
     */
  const int getNetParentConnectionsSize() const;

  /**
     * @brief getChildConnectionsSize returns the number of child connections
     * @return the number of net child connections
     */
  const int getChildConnectionsSize() const;

  /**
     * @brief getNetChildConnectionsSize returns the number of net child connections
     * @return the number of net child connections
     */
  const int getNetChildConnectionsSize() const;

  /**
     * @brief parentConnection returns the i-th parent connection
     * @param i is the index of the parent connection
     * @return the i-th parent connection
     */
  ConnectionPtr parentConnection(const int& i) const;

  /**
     * @brief netParentConnection returns the i-th net parent connection
     * @param i is the index of the net parent connection
     * @return the i-th net parent connection
     */
  ConnectionPtr netParentConnection(const int& i) const;

  /**
     * @brief childConnection returns the i-th child connection
     * @param i is the index of the child connection
     * @return the i-th child connection
     */
  ConnectionPtr childConnection(const int& i) const;

  /**
     * @brief netChildConnection returns the i-th net child connection
     * @param i is the index of the net child connection
     * @return the i-th child connection
     */
  ConnectionPtr netChildConnection(const int& i) const;

  /**
     * @brief getChildren returs the vector of child nodes
     * @return the vector of child nodes
     */
  std::vector<NodePtr> getChildren() const;

  /**
     * @brief getParents returns the vector of parent nodes
     * @return the vector of parent nodes
     */
  std::vector<NodePtr> getParents() const;

  /**
     * @brief getNetParents returns the vector of parent nodes connected through a net connection
     * @return the vector of parent nodes connected through a net connection
     */
  std::vector<NodePtr> getNetParents() const;

  /**
     * @brief getNetChildren returns the vector of child nodes connected through a net connection
     * @return the vector of child nodes connected through a net connection
     */
  std::vector<NodePtr> getNetChildren() const;

  //  const std::vector<NodePtr> getChildrenConst() const;
  //  const std::vector<NodePtr> getParentsConst() const;
  //  const std::vector<NodePtr> getNetParentsConst() const;
  //  const std::vector<NodePtr> getNetChildrenConst() const;

  /**
     * @brief getParentConnections returns the vector of parent connections of the node
     * @return the vector of parent connections of the node
     */
  std::vector<ConnectionPtr> getParentConnections() const;

  /**
     * @brief getNetParentConnections returns the vector of net parent connections of the node
     * @return the vector of net parent connections of the node
     */
  std::vector<ConnectionPtr> getNetParentConnections() const;

  /**
     * @brief getChildConnections returns the vector of child connections of the node
     * @return the vector of parent connections of the node
     */
  std::vector<ConnectionPtr> getChildConnections() const;

  /**
     * @brief getNetChildConnections returns the vector of net child connections of the node
     * @return the vector of net child connections of the node
     */
  std::vector<ConnectionPtr> getNetChildConnections() const;

  //  /**
  //   * @brief getParentConnectionsConst returns the vector of parent connections of the node
  //   * @return a const vector of parent connections of the node
  //   */
  //  const std::vector<ConnectionPtr>& getParentConnectionsConst() const;

  //  /**
  //   * @brief getNetParentConnectionsConst returns the vector of net parent connections of the node
  //   * @return a const vector of net parent connections of the node
  //   */
  //  const std::vector<ConnectionPtr>& getNetParentConnectionsConst() const;

  //  /**
  //   * @brief getChildConnectionsConst returns the vector of child connections of the node
  //   * @return a const vector of child connections of the node
  //   */
  //  const std::vector<ConnectionPtr>& getChildConnectionsConst() const;

  //  /**
  //   * @brief getNetChildConnectionsConst returns the vector of net child connections of the node
  //   * @return a const vector of net child connections of the node
  //   */
  //  const std::vector<ConnectionPtr>& getNetChildConnectionsConst() const;

  /**
     * @brief disconnectChildConnections deletes the connections to the children of the node
     */
  void disconnectChildConnections();

  /**
     * @brief disconnectParentConnections deletes the connections to the parents of the node
     */
  void disconnectParentConnections();

  /**
     * @brief disconnectNetParentConnections deletes the net connections to the parents of the node
     */
  void disconnectNetParentConnections();

  /**
     * @brief disconnectNetChildConnections deletes the net connections to the children of the node
     */
  void disconnectNetChildConnections();

  /**
     * @brief disconnect deletes all the connections of the node
     */
  void disconnect();


  /**
     * @brief removeParentConnection deletes the parent connection given as input, if actually a parent connection of the node
     * @param connection is the parent connection to delete
     */
  void removeParentConnection(const ConnectionPtr& connection);

  /**
     * @brief removeChildConnection deletes the child connection given as input, if actually a child connection of the node
     * @param connection is the child connection to delete
     */
  void removeChildConnection(const ConnectionPtr& connection);

  /**
     * @brief removeNetParentConnection deletes the net parent connection given as input, if actually a net parent connection of the node
     * @param connection is the net parent connection to delete
     */
  void removeNetParentConnection(const ConnectionPtr& connection);

  /**
     * @brief removeNetChildConnection deletes the net child connection given as input, if actually a net child connection of the node
     * @param connection is the net child connection to delete
     */
  void removeNetChildConnection(const ConnectionPtr& connection);

  /**
     * @brief switchParentConnection turns a parent net connection of the node into a parent connection
     * @param net_connection is the net parent connection to turn into a parent connection
     * @return true if successful, false if the net_connection given as input is not a net connection or not a connection of the node
     */
  bool switchParentConnection(const ConnectionPtr& net_connection);

  /**
   * @brief getConfiguration return the configuration of the node
   * @return the configuration of the node as an Eigen::VectorXd
   */
  const Eigen::VectorXd& getConfiguration()
  {
    return configuration_;
  }

  ~Node();

  XmlRpc::XmlRpcValue toXmlRpcValue() const;

  static NodePtr fromXmlRpcValue(const XmlRpc::XmlRpcValue& x);

  /**
     * @brief getReservedFlagsNumber tells you how many positions are occupied by the defaults. Use this to know where you can save your new flags.
     * @return the first free position in flags_ vector, so the idx next to the defaults.
     */
  static unsigned int getReservedFlagsNumber();

  friend std::ostream& operator<<(std::ostream& os, const Node& path);
};

std::ostream& operator<<(std::ostream& os, const Node& node);

}
