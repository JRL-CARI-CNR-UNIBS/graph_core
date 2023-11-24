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

namespace pathplan
{
class Connection: public std::enable_shared_from_this<Connection>
{
  friend class Node;

protected:
  NodeWeakPtr parent_;
  NodePtr child_;
  double cost_;
  double euclidean_norm_;
  double time_cost_update_;
  double likelihood_;

  /**
   * @brief flags_ is a vector of flags. By default, the first three positions are reserved for valid flag, net flag and recently checked flag.
   * You can add new flags specific to your algorithm using function setFlag and passing the vector-index to store the flag.
   * getReservedFlagsNumber allows you to know how many positions are reserved for the defaults. setFlag doesn't allow you to overwrite these positions.
   * To overwrite them, you should use the flag-specific function (like setRecentlyChecked).
   */
  std::vector<bool> flags_;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * Add here your reserved flags.
   * Increment number_reserved_flags_ accordingly!
   * Initialize flags_ in the constructor accordingly!
   * If you need to modify or read these flags externally, implement getter and setter functions!
   * If you want, print your flag when << operator is called on a connection
   */
  static constexpr unsigned int idx_valid_ = 0;
  static constexpr unsigned int idx_net_ = 1;
  static constexpr unsigned int idx_recently_checked_ = 2;

  static constexpr unsigned int number_reserved_flags_ = 3;

  /**
   * @brief Connection is the constructor of the Connection object
   * @param parent is the parent node of the connection
   * @param child is the child node of the connection
   * @param is_net is a bool to define the type of connection: net or standard.
   */
  Connection(const NodePtr& parent, const NodePtr& child, const bool is_net = false);

  /**
   * @brief pointer returns a std::shared_pointer to this connection
   * @return
   */
  ConnectionPtr pointer()
  {
    return shared_from_this();
  }

  /**
   * @brief isNet tells if the connection is of type net or not. It basically check the value of the reserved flag.
   * @return true if net, false otherwise
   */
  bool isNet() const
  {
    return flags_[idx_net_];
  }

  /**
   * @brief isRecentlyChecked tells if the connection has been recently checked.  It basically check the value of the reserved flag.
   * @return true if recently checked, false otherwise
   */
  bool isRecentlyChecked() const
  {
    return flags_[idx_recently_checked_];
  }

  /**
   * @brief setRecentlyChecked sets the "recently checked" flag
   * @param checked true if the connection has been recently checked, false otherwise
   */
  void setRecentlyChecked(bool checked)
  {
    flags_[idx_recently_checked_] = checked;
  }

  /**
   * @brief isValid tells if the connection is valid, that means it is connected to a parent and to a child
   * @return true if valid, false otherwise
   */
  bool isValid() const
  {
    return flags_[idx_valid_];
  }

  /**
   * @brief setCost sets the cost of the connection and updates time_cost_update_ accordingly
   * @param cost is the cost
   */
  void setCost(const double& cost)
  {
    cost_ = cost;
    time_cost_update_ = ros::WallTime::now().toSec();
  }

  /**
   * @brief getCost returns the cost of the connection
   * @return the cost of the connection
   */
  const double& getCost() const
  {
    return cost_;
  }

  /**
   * @brief getTimeCostUpdate returns the time when the connection cost was last updated
   */
  const double& getTimeCostUpdate() const
  {
    return time_cost_update_;
  }

  /**
   * @brief setTimeCostUpdate sets the time when the connection cost is updated.
   *  E.g., if you clone a path, you may want to set the same time_cost_update_ of the connections of the original path
   * @param time
   */
  void setTimeCostUpdate(const double& time)
  {
    time_cost_update_ = time;
  }

  /**
   * @brief norm returns the length of the connection, computed using the Euclidean norm.
   * @return
   */
  double norm() const
  {
    return euclidean_norm_;
  }

  /**
   * @brief getParent return the parent node
   * @return the parent node
   */
  NodePtr getParent() const
  {
    assert(not parent_.expired());
    return parent_.lock();
  }

  /**
   * @brief getChild return the child node
   * @return the child node
   */
  NodePtr getChild() const
  {
    return child_;
  }

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
   * @brief setLikelihood sets the likelihood member
   * @param likelihood the likelihood to set
   */
  void setLikelihood(const double& likelihood){likelihood_=likelihood;}

  /**
   * @brief getFlag returns the value of the flag at position idx. It returns the value if the flag exists, otherwise return the default value.
   * @param idx the index of the flag you are asking for.
   * @param default_value the default value returned if the flag doesn't exist.
   * @return the flag if it exists, the default value otherwise.
   */
  bool getFlag(const int& idx, const bool default_value);

  /**
   * @brief add notifies parent a child nodes that a new conenction exists. It should be called immediatly after the connection object is created
   */
  void add();

  /**
   * @brief add notifies parent a child nodes that a new conenction exists. It should be called immediatly after the connection object is created
   * @param is_net allows to define if the connection is of type net or not. By default, it is not net
   */
  void add(const bool is_net);

  /**
   * @brief remove delete the connection and notifies its parent and child
   */
  void remove();

  /**
   * @brief flip reverses the direction of the connection, the child becomes the parent and viceversa
   */
  void flip();

  /**
   * @brief convertToConnection turns a net connection into a standard connection, if it is net connection
   * @return true if successful
   */
  bool convertToConnection();

  /**
   * @brief convertToNetConnection turns a connection into a net connection, if it is a standard connection
   * @return true if successful
   */
  bool convertToNetConnection();

  /**
   * @brief changeConnectionType calls convertToNetConnection or convertToConnection based on the connection type
   */
  void changeConnectionType();

  /**
   * @brief isParallel checks if the two connections are parallel
   * @param conn is the input connection. The function check if this connection is parallel to conn
   * @param toll is the error tolerance. Two connections are considered parallel when the disparity between the scalar product of the two connections is less than a tolerance value from the product of their lengths.
   * @return true if parallel
   */
  bool isParallel(const ConnectionPtr& conn, const double& toll = 1e-06);

  /**
   * @brief projectOnConnection projects a point on this connection
   * @param point is the point to be projected
   * @param distance is the distance between the projected point and point
   * @param in_conn is true if the projection is between connection's parent and child, false if the projected point is on extending of the connection
   * @param verbose set the verbosity
   * @return the projected point as an Eigen::VectorXd
   */
  Eigen::VectorXd projectOnConnection(const Eigen::VectorXd& point, double& distance, bool& in_conn, const bool& verbose = false);

  /**
   * @brief getReservedFlagsNumber tells you how many positions are occupied by the defaults. Use this to know where you can sve your new flags.
   * @return the first free position in flags_ vector, so the idx next to the defaults.
   */
  static unsigned int getReservedFlagsNumber()
  {
    return number_reserved_flags_;
  }

  ~Connection();

  friend std::ostream& operator<<(std::ostream& os, const Connection& connection);
};

std::ostream& operator<<(std::ostream& os, const Connection& connection);

}
