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

#include <graph_core/graph/path.h>
namespace pathplan
{

Path::Path(std::vector<ConnectionPtr> connections,
           const MetricsPtr& metrics,
           const CollisionCheckerPtr& checker):
  connections_(connections),
  metrics_(metrics),
  checker_(checker)
{
  assert(connections_.size() > 0);

  cost_ = 0;

  for (const ConnectionPtr& conn : connections_)
  {
    cost_ += conn->getCost();
    change_warp_.push_back(true);
    change_slip_child_.push_back(true);
    change_slip_parent_.push_back(true);
#ifdef NO_SPIRAL
    change_spiral_.push_back(true);
#endif
  }
  change_warp_.at(0) = false;
  change_slip_child_.at(0) = false;
  change_slip_parent_.at(0) = false;
#ifdef NO_SPIRAL
  change_spiral_.at(0) = false;
#endif

}


double Path::computeEuclideanNorm()
{
  double euclidean_norm = 0;

  for (const ConnectionPtr& conn : connections_)
    euclidean_norm += conn->norm();
  return euclidean_norm;
}


Eigen::VectorXd Path::pointOnCurvilinearAbscissa(const double& abscissa)
{
  assert(connections_.size() > 0);
  if (abscissa <= 0)
    return connections_.at(0)->getParent()->getConfiguration();
  double euclidean_norm = 0;
  for (const ConnectionPtr& conn : connections_)
  {
    if ((euclidean_norm + conn->norm()) > abscissa)
    {
      double ratio = (abscissa - euclidean_norm) / conn->norm();
      return conn->getParent()->getConfiguration() + ratio * (conn->getChild()->getConfiguration() - conn->getParent()->getConfiguration());
    }
    euclidean_norm += conn->norm();

  }
  return connections_.back()->getChild()->getConfiguration();
}

double Path::curvilinearAbscissaOfPoint(const Eigen::VectorXd& conf, int& idx)
{
  ConnectionPtr connection = findConnection(conf,idx);
  if(connection == NULL)
  {
    ROS_ERROR("The configuration doesn't belong to the path");
    assert(0);
  }
  else
  {
    double euclidean_norm = 0;
    double sub_euclidean_norm = -1;
    for (const ConnectionPtr& conn : connections_)
    {
      if(connection == conn)
      {
        sub_euclidean_norm = euclidean_norm; //norma fino alla connessione precedente a quella dove si trova la configurazione
      }
      euclidean_norm += conn->norm();
    }

    if(sub_euclidean_norm == -1) assert(0);

    double dist = (conf - connection->getParent()->getConfiguration()).norm();
    double abscissa = (sub_euclidean_norm+dist)/euclidean_norm;

    return abscissa;
  }
}

double Path::curvilinearAbscissaOfPoint(const Eigen::VectorXd& conf)
{
  int idx;
  return curvilinearAbscissaOfPoint(conf,idx);
}

void Path::computeCost()
{
  cost_ = 0;

  for (const ConnectionPtr& conn : connections_)
    cost_ += conn->getCost();
}

void Path::setChanged(const unsigned int &connection_idx)
{
  change_slip_child_.at(connection_idx) = 1;
  change_slip_parent_.at(connection_idx) = 1;
  change_warp_.at(connection_idx) = 1;
#ifdef NO_SPIRAL
  change_spiral_.at(connection_idx) = 1;
#endif
}

bool Path::bisection(const unsigned int &connection_idx,
                     const Eigen::VectorXd &center,
                     const Eigen::VectorXd &direction,
                     double max_distance,
                     double min_distance)
{
  assert(connection_idx < connections_.size());
  assert(connection_idx > 0);
  ConnectionPtr& conn12 = connections_.at(connection_idx - 1);
  ConnectionPtr& conn23 = connections_.at(connection_idx);

  NodePtr parent = conn12->getParent();
  NodePtr child = conn23->getChild();

  bool improved = false;
  double cost = conn12->getCost() + conn23->getCost();

  unsigned int iter = 0;
  double distance;

  while (iter++ < 5 & (max_distance - min_distance) > min_length_)
  {
    if (iter > 0)
      distance = 0.5 * (max_distance + min_distance);
    else
      distance = min_distance;

    Eigen::VectorXd p = center + direction * distance;
    assert(p.size() == center.size());
    double cost_pn = metrics_->cost(parent->getConfiguration(), p);
    double cost_nc = metrics_->cost(p, child->getConfiguration());
    double cost_n = cost_pn + cost_nc;

    if (cost_n >= cost)
    {
      min_distance = distance;
      continue;
    }
    bool is_valid = checker_->checkPath(parent->getConfiguration(), p) && checker_->checkPath(p, child->getConfiguration());
    if (!is_valid)
    {
      min_distance = distance;
      continue;
    }
    improved = true;
    max_distance = distance;
    cost = cost_n;
    //    conn12->remove(); // keep this connection, it could be useful in case of tree
    conn23->remove();

    NodePtr n = std::make_shared<Node>(p);
    conn12 = std::make_shared<Connection>(parent, n);
    conn23 = std::make_shared<Connection>(n, child);
    conn12->setCost(cost_pn);
    conn23->setCost(cost_nc);
    conn12->add();
    conn23->add();

    if (tree_)
      tree_->addNode(n, false);



  }
  if (improved)
    computeCost();
  return improved;
}

bool Path::warp()
{
  for (unsigned int idx = 1; idx < connections_.size(); idx++)
  {
    if (change_warp_.at(idx - 1) || change_warp_.at(idx))
    {
      Eigen::VectorXd center = 0.5 * (connections_.at(idx - 1)->getParent()->getConfiguration() +
                                      connections_.at(idx)->getChild()->getConfiguration());
      Eigen::VectorXd direction = connections_.at(idx - 1)->getChild()->getConfiguration() - center;
      double max_distance = direction.norm();
      double min_distance = 0;

      direction.normalize();

      if (!bisection(idx, center, direction, max_distance, min_distance))
      {
        change_warp_.at(idx) = 0;
      }
      else
        setChanged(idx);
    }
  }
  return std::any_of(change_warp_.cbegin(), change_warp_.cend(), [](bool i)
  {
    return i;
  });

}

bool Path::slipChild()
{
  for (unsigned int idx = 1; idx < connections_.size(); idx++)
  {
    if (change_slip_child_.at(idx - 1) || change_slip_child_.at(idx))
    {
      Eigen::VectorXd center = connections_.at(idx)->getChild()->getConfiguration();
      Eigen::VectorXd direction = connections_.at(idx - 1)->getChild()->getConfiguration() - center;
      double max_distance = direction.norm();
      double min_distance = 0;

      direction.normalize();

      if (!bisection(idx, center, direction, max_distance, min_distance))
      {
        change_slip_child_.at(idx) = 0;
      }
      else
        setChanged(idx);
    }
  }
  return std::any_of(change_slip_child_.cbegin(), change_slip_child_.cend(), [](bool i)
  {
    return i;
  });
}

bool Path::slipParent()
{
  for (unsigned int idx = 1; idx < connections_.size(); idx++)
  {
    if (change_slip_parent_.at(idx - 1) || change_slip_parent_.at(idx))
    {
      Eigen::VectorXd center = connections_.at(idx - 1)->getParent()->getConfiguration();
      Eigen::VectorXd direction = connections_.at(idx - 1)->getChild()->getConfiguration() - center;
      double max_distance = direction.norm();
      double min_distance = 0;

      direction.normalize();

      if (!bisection(idx, center, direction, max_distance, min_distance))
      {
        change_slip_parent_.at(idx) = 0;
      }
      else
        setChanged(idx);
    }
  }
  return std::any_of(change_slip_parent_.cbegin(), change_slip_parent_.cend(), [](bool i)
  {
    return i;
  });

}

#ifdef NO_SPIRAL
bool Path::spiral()
{
  for (unsigned int idx = 1; idx < connections_.size(); idx++)
  {
    if (change_spiral_.at(idx - 1) || change_spiral_.at(idx))
    {
      Eigen::VectorXd center = 0.5 * (connections_.at(idx - 1)->getParent()->getConfiguration() +
                                      connections_.at(idx)->getChild()->getConfiguration());
      Eigen::VectorXd direction1 = connections_.at(idx - 1)->getChild()->getConfiguration() - center;
      double max_distance = direction1.norm();
      double min_distance = 0;

      direction1.normalize();

      Eigen::VectorXd direction2 = connections_.at(idx)->getChild()->getConfiguration() -
          connections_.at(idx - 1)->getParent()->getConfiguration();
      direction2.normalize();
      Eigen::VectorXd direction3(direction1.size());
      direction3.setRandom();
      direction3 = direction3 - direction3.dot(direction1) * direction1;
      direction3 = direction3 - direction3.dot(direction2) * direction2;
      direction3.normalize();

      Eigen::VectorXd direction = direction1 * 0.5 + direction3 * 0.5;

      if (!bisection(idx, center, direction1, max_distance, min_distance))
      {
        change_spiral_.at(idx) = 0;
      }
      else
        setChanged(idx);
    }
  }
  return std::any_of(change_spiral_.cbegin(), change_spiral_.cend(), [](bool i)
  {
    return !i;
  });

}
#endif
bool Path::resample(const double &distance)
{
  bool resampled = false;
  unsigned int ic = 0;
  while (ic < connections_.size())
  {
    if (connections_.at(ic)->norm() <= distance)
      continue;

    ROS_ERROR("still unimplemented");
    resampled = true;
  }

  return resampled;

}

std::vector<Eigen::VectorXd> Path::getWaypoints()
{
  std::vector<Eigen::VectorXd> wp;
  if (connections_.size() == 0)
    return wp;

  wp.push_back(connections_.at(0)->getParent()->getConfiguration());
  for (const ConnectionPtr& conn : connections_)
    wp.push_back(conn->getChild()->getConfiguration());

  return wp;
}


ConnectionPtr Path::findConnection(const Eigen::VectorXd& configuration, int& idx)
{
  ConnectionPtr conn = NULL;
  idx = -1;

  Eigen::VectorXd parent;
  Eigen::VectorXd child;

  double dist, dist1, dist2;

  for(unsigned int i=0; i<connections_.size(); i++)
  {
    parent = connections_.at(i)->getParent()->getConfiguration();
    child =  connections_.at(i)->getChild()->getConfiguration();

    dist = (parent-child).norm();
    dist1 = (parent - configuration).norm();
    dist2 = (configuration-child).norm();

    if(abs(dist-dist1-dist2)<1e-06)
    {
      conn = connections_.at(i);
      idx = i;
      return conn;
    }
  }

  ROS_ERROR("Connection not found");
}

Eigen::VectorXd Path::projectOnConnection(const Eigen::VectorXd& point, const ConnectionPtr &conn, double& distance, bool& in_conn)
{
  //To find the projection of point on the connection, the area of the triangle composed by the parent,
  //the child and the point is calculated using the Erone's formula. Then it is checked if the projection is
  // between the parent and the child

  Eigen::VectorXd parent = conn->getParent()->getConfiguration();
  Eigen::VectorXd child = conn->getChild()->getConfiguration();

  double a = (parent-child).norm();
  double b = (parent-point).norm();
  double c = (point-child).norm();

  double p = (a+b+c)/2;
  double A = sqrt(p*(p-a)*(p-b)*(p-c)); //Erone's formula

  distance = 2*A/a;  //A=a*h/2 -> h=2A/a

  double c_pitagora = sqrt(b*b+a*a);
  double s; //distance parent-projection

  if(c<=c_pitagora) s = sqrt(b*b-distance*distance);
  else s = - sqrt(b*b-distance*distance);

  Eigen::VectorXd projection = parent+(child-parent)*s/a;

  if(s/a>=0 && s/a<=1) in_conn = 1;
  else in_conn = 0;

  if(conn == connections_.front() && s/a<0)  //if the point is before the start it is projected on the start
  {
    projection = parent; //the start
    in_conn = 1;
  }

  if(conn == connections_.back() && s/a>1)  //if the point is before the start it is projected on the start
  {
    projection = child;  //the goal
    in_conn = 1;
  }

  return projection;
}

const Eigen::VectorXd Path::projectOnClosestConnection(const Eigen::VectorXd& point)
{
  //The point is projected on the connection on which its projection is between parent and child and to which the distance is the smallest

  Eigen::VectorXd pr;
  Eigen::VectorXd projection;
  double min_distance = std::numeric_limits<double>::infinity();

  for(const ConnectionPtr conn:connections_)
  {
    double distance;
    bool in_conn;
    pr = projectOnConnection(point,conn,distance,in_conn);

    if(in_conn)
    {
      if(distance<min_distance)
      {
        min_distance = distance;
        projection = pr;
      }
    }
  }

  if(min_distance == std::numeric_limits<double>::infinity())
  {
    projection = findCloserNode(point)->getConfiguration();
    ROS_ERROR("projection on path not found");
    //assert(0);
  }
  return projection;
}

const Eigen::VectorXd Path::projectOnClosestConnection(const Eigen::VectorXd& point, int& n_conn)
{
  //The point is projected on the connection on which its projection is between parent and child and to which the distance is the smallest and the history of the projection is taken into consideration

  int idx;
  Eigen::VectorXd pr;
  Eigen::VectorXd projection;
  double min_distance = std::numeric_limits<double>::infinity();

  ConnectionPtr conn;
  for(unsigned int i=0;i<connections_.size();i++)
  {
    conn = connections_.at(i);

    double distance;
    bool in_conn;
    pr = projectOnConnection(point,conn,distance,in_conn);

    if(in_conn)
    {
      if(distance<min_distance)
      {
        if(i==n_conn || i==n_conn+1)
        {
          min_distance = distance;
          projection = pr;
          idx = i;
        }
      }
    }
  }


  if(min_distance == std::numeric_limits<double>::infinity())
  {
    projection = connections_.at(n_conn)->getChild()->getConfiguration();

    //projection = findCloserNode(point)->getConfiguration();
    //findConnection(projection,n_conn);
    ROS_ERROR("projection on path not found");
  }
  else  n_conn = idx;

  return projection;
}

NodePtr Path::addNodeAtCurrentConfig(const Eigen::VectorXd& configuration, ConnectionPtr& conn, bool& rewire)
{
  if(conn != NULL)
  {
    NodePtr parent = conn->getParent();
    NodePtr child = conn->getChild();

    NodePtr actual_node;
    if((parent->getConfiguration()-configuration).norm()<1e-06) //if the current conf is too close to the parent or to the child, it is approximated with the parent/child
    {
      actual_node = parent;
    }
    else if((child->getConfiguration()-configuration).norm()<1e-06)
    {
      actual_node = child;
    }
    else
    {
      actual_node = std::make_shared<Node>(configuration);

      if(rewire)
      {
        double cost_parent = metrics_->cost(parent->getConfiguration(), actual_node->getConfiguration());
        ConnectionPtr conn_parent = std::make_shared<Connection>(parent, actual_node);
        conn_parent->setCost(cost_parent);
        conn_parent->add();

        double cost_child = metrics_->cost(actual_node->getConfiguration(),child->getConfiguration());
        ConnectionPtr conn_child = std::make_shared<Connection>(actual_node,child);
        conn_child->setCost(cost_child);
        conn_child->add();

        conn->remove();

        std::vector<ConnectionPtr> conn_vector;

        if(parent->getConfiguration() == this->getConnections().front()->getParent()->getConfiguration())  //the parent is the start node
        {
          PathPtr subpath_child = this->getSubpathFromNode(child);

          conn_vector.push_back(conn_parent);
          conn_vector.push_back(conn_child);
          std::vector<ConnectionPtr> conn_sup = subpath_child->getConnections();
          conn_vector.insert(conn_vector.end(),conn_sup.begin(),conn_sup.end());
        }
        else if(child->getConfiguration() == this->getConnections().back()->getChild()->getConfiguration())  //the child is the goal node
        {
          PathPtr subpath_parent = this->getSubpathToNode(parent);

          std::vector<ConnectionPtr> conn_sup = subpath_parent->getConnections();
          conn_vector.insert(conn_vector.begin(),conn_sup.begin(),conn_sup.end());
          conn_vector.push_back(conn_parent);
          conn_vector.push_back(conn_child);
        }
        else
        {
          PathPtr subpath_parent = this->getSubpathToNode(parent);
          PathPtr subpath_child = this->getSubpathFromNode(child);

          std::vector<ConnectionPtr> conn_sup = subpath_parent->getConnections();
          conn_vector.insert(conn_vector.begin(),conn_sup.begin(),conn_sup.end());
          conn_vector.push_back(conn_parent);
          conn_vector.push_back(conn_child);
          conn_sup = subpath_child->getConnections();
          conn_vector.insert(conn_vector.end(),conn_sup.begin(),conn_sup.end());
        }
        this->setConnections(conn_vector);
      }
    }
    return actual_node;
  }

  ROS_ERROR("Connection not found, the node can't be created");
}

NodePtr Path::findCloserNode(const Eigen::VectorXd& configuration)
{
  if(connections_.size()<1)
  {
    ROS_ERROR("No connections");
    throw std::invalid_argument("No connections");
  }

  NodePtr closest_node=connections_.at(0)->getParent();
  double min_dist = (closest_node->getConfiguration()-configuration).norm();
  double dist;
  for (const ConnectionPtr& conn: connections_)
  {
    dist = (conn->getChild()->getConfiguration()-configuration).norm();
    if(dist<min_dist)
    {
      closest_node=conn->getChild();
      min_dist=dist;
    }
  }
  return closest_node;
}

NodePtr Path::findCloserNode(const NodePtr& node)
{
  Eigen::VectorXd configuration = node->getConfiguration();

  NodePtr closest_node = findCloserNode(configuration);

  return closest_node;
}

PathPtr Path::getSubpathToNode(const Eigen::VectorXd& conf)
{
  NodePtr node = std::make_shared<Node>(conf);
  return getSubpathToNode(node);
}

PathPtr Path::getSubpathToNode(const NodePtr& node)
{
  if((node->getConfiguration()-connections_.front()->getParent()->getConfiguration()).norm()<1e-06)
  {
    ROS_ERROR("No subpath available, the node is equal to the first node of the path");
    ROS_INFO_STREAM("configuration: "<<node->getConfiguration().transpose());
    throw std::invalid_argument("No subpath available, the node is equal to the first node of the path");
  }

  if((node->getConfiguration()-connections_.back()->getChild()->getConfiguration()).norm()<1e-06)
  {
    return this->pointer();
  }

  PathPtr subpath;
  for(unsigned int idx=0; idx<connections_.size(); idx++)
  {
    if((node->getConfiguration()-connections_.at(idx)->getChild()->getConfiguration()).norm()<1e-06)
    {
      std::vector<ConnectionPtr> conn;
      conn.assign(connections_.begin(), connections_.begin()+idx+1); // to save the idx connections

      subpath = std::make_shared<Path>(conn, metrics_,checker_);
      return subpath;
    }
  }

  ROS_ERROR("The node doesn to belong to this path");
  ROS_INFO_STREAM("configuration: "<<node->getConfiguration().transpose());
  throw std::invalid_argument("The node doesn to belong to this path");
}

PathPtr Path::getSubpathFromNode(const Eigen::VectorXd& conf)
{
  NodePtr node = std::make_shared<Node>(conf);
  return getSubpathFromNode(node);
}

PathPtr Path::getSubpathFromNode(const NodePtr& node)
{
  if((node->getConfiguration()-connections_.back()->getChild()->getConfiguration()).norm()<1e-06)
  {
    ROS_ERROR("No subpath available, the node is equal to the last node of the path");
    ROS_INFO_STREAM("configuration: "<<node->getConfiguration().transpose());
    throw std::invalid_argument("No subpath available, the node is equal to the last node of the path");
  }

  if((node->getConfiguration()-connections_.front()->getParent()->getConfiguration()).norm()<1e-06)
  {
    return this->pointer();
  }

  PathPtr subpath;
  for(unsigned int idx=0; idx<connections_.size(); idx++)
  {
    if((node->getConfiguration()-connections_.at(idx)->getChild()->getConfiguration()).norm()<1e-06)
    {
      std::vector<ConnectionPtr> conn;
      conn.assign(connections_.begin()+idx+1, connections_.end());

      subpath = std::make_shared<Path>(conn, metrics_,checker_);
      return subpath;
    }
  }

  ROS_ERROR("The node doesn t belong to this path");
  ROS_INFO_STREAM("configuration: "<<node->getConfiguration().transpose());
  throw std::invalid_argument("The node doesn to belong to this path");
}

bool Path::simplify(const double& distance)
{
  bool simplified;
  unsigned int ic = 1;
  while (ic < connections_.size())
  {
    double dist = (connections_.at(ic)->getParent()->getConfiguration() - connections_.at(ic)->getChild()->getConfiguration()).norm();
    if (dist > distance)
    {
      ic++;
      continue;
    }
    if (checker_->checkPath(connections_.at(ic - 1)->getParent()->getConfiguration(),
                            connections_.at(ic)->getChild()->getConfiguration()))
    {
      simplified = true;
      double cost = metrics_->cost(connections_.at(ic - 1)->getParent(),
                                   connections_.at(ic)->getChild());
      ConnectionPtr conn = std::make_shared<Connection>(connections_.at(ic - 1)->getParent(),
                                                        connections_.at(ic)->getChild());
      conn->setCost(cost);
      conn->add();
      connections_.at(ic)->remove();
      connections_.erase(connections_.begin() + (ic - 1), connections_.begin() + ic + 1);
      connections_.insert(connections_.begin() + (ic - 1), conn);

      change_warp_.erase(change_warp_.begin() + ic);
      change_warp_.at(ic - 1) = 1;
      change_slip_parent_.erase(change_slip_parent_.begin() + ic);
      change_slip_parent_.at(ic - 1) = 1;
      change_slip_child_.erase(change_slip_child_.begin() + ic);
      change_slip_child_.at(ic - 1) = 1;

#ifndef NO_SPIRAL
      change_spiral_.erase(change_spiral_.begin() + ic);
      change_spiral_.begin() + (ic - 1) = 1;
#endif
    }
    else
      ic++;
  }

  return simplified;
}


bool Path::isValid()
{
  bool valid = true;
  Eigen::VectorXd parent;
  Eigen::VectorXd child;
  double cost;

  if(cost_ == std::numeric_limits<double>::infinity())
  {
    return false;
  }

  for(const ConnectionPtr& conn : connections_)
  {
    assert(conn);
    if(!checker_->checkPath(conn->getParent()->getConfiguration(), conn->getChild()->getConfiguration()))
    {
      conn->setCost(std::numeric_limits<double>::infinity());
      valid = false;
    }
    else
    {
      parent = conn->getParent()->getConfiguration();
      child = conn->getChild()->getConfiguration();
      cost = metrics_->cost(parent,child);
      conn->setCost(cost);
    }
  }

  if(!valid) cost_ = std::numeric_limits<double>::infinity();
  else computeCost();

  return valid;
}

std::ostream& operator<<(std::ostream& os, const Path& path)
{
  os << "cost = " << path.cost_ << std::endl;
  if (path.connections_.size() == 0)
    os << "no waypoints";

  os << "waypoints= " << std::endl << "[";

  for (const ConnectionPtr& conn : path.connections_)
  {
    os << conn->getParent()->getConfiguration().transpose() << ";" << std::endl;
  }
  os << path.connections_.back()->getChild()->getConfiguration().transpose() << "];" << std::endl;

  return os;
}

}
