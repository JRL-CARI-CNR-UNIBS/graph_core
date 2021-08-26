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

Path::Path(std::vector<NodePtr> nodes,
           const MetricsPtr& metrics,
           const CollisionCheckerPtr& checker):
  metrics_(metrics),
  checker_(checker)
{
  assert(nodes_.size() > 0);

  connections_.clear();
  cost_ = 0;
  for(unsigned int i=0;i<nodes.size()-1;i++)
  {
    NodePtr parent = nodes.at(i);
    NodePtr child  = nodes.at(i+1);

    ConnectionPtr conn = std::make_shared<Connection>(parent,child);
    double cost = metrics->cost(parent,child);
    conn->setCost(cost);
    conn->add();

    connections_.push_back(conn);

    cost_ += cost;
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

PathPtr Path::clone()
{
  std::vector<ConnectionPtr> new_conn_vector;

  for(const ConnectionPtr &conn: connections_)
  {
    new_conn_vector.push_back(conn->clone());
  }

  PathPtr new_path = std::make_shared<Path>(new_conn_vector,metrics_,checker_);

  new_path->setChangeSpiral(change_spiral_);
  new_path->setChangeWarp(change_warp_);
  new_path->setgChangeSlipChild(change_slip_child_);
  new_path->setgChangeSlipParent(change_slip_parent_);
  new_path->setTree(tree_);

  return new_path;
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
  double abscissa = std::numeric_limits<double>::infinity();

  ConnectionPtr connection = findConnection(conf,idx);
  if(connection == NULL)
  {
    ROS_ERROR("The configuration does not belong to the path -> the curvilinear abscissa can not be computed");
    ROS_INFO_STREAM("conf: "<<conf.transpose());
    assert(0);
    return abscissa;
  }
  else
  {
    return curvilinearAbscissaOfPointGivenConnection(conf,idx);
  }
}

double Path::curvilinearAbscissaOfPoint(const Eigen::VectorXd& conf)
{
  int idx;
  return curvilinearAbscissaOfPoint(conf,idx);
}

double Path::curvilinearAbscissaOfPointGivenConnection(const Eigen::VectorXd& conf, const int& conn_idx)
{
  double abscissa = std::numeric_limits<double>::infinity();

  if(conn_idx < 0 || conn_idx >= connections_.size())
  {
    ROS_ERROR("The connection does not belong to the path -> the curvilinear abscissa can not be computed");
    ROS_INFO_STREAM("conn_idx: "<<conn_idx);
    assert(0);
    return abscissa;
  }

  ConnectionPtr connection = connections_.at(conn_idx);

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
  abscissa = (sub_euclidean_norm+dist)/euclidean_norm;

  return abscissa;
}

double Path::getCostFromConf(const Eigen::VectorXd &conf)
{
  computeCost();
  double cost = 0;
  int idx;
  ConnectionPtr this_conn = findConnection(conf,idx);

  if(this_conn == NULL)
  {
    ROS_ERROR("cost can't be computed");
    return 0;
  }
  else
  {
    if(conf == connections_.at(0)->getParent()->getConfiguration()) return cost_;

    if(idx < connections_.size()-1)
    {
      for(unsigned int i=idx+1;i<connections_.size();i++)
      {
        cost += connections_.at(i)->getCost();
        if(cost == std::numeric_limits<double>::infinity()) return std::numeric_limits<double>::infinity();
      }
    }

    if(conf == this_conn->getParent()->getConfiguration()) cost += this_conn->getCost();
    else if (conf == this_conn->getChild()->getConfiguration()) cost += 0;
    else
    {
      if(this_conn->getCost() == std::numeric_limits<double>::infinity())
      {
        ConnectionPtr conn = std::make_shared<Connection>(std::make_shared<Node>(conf),this_conn->getChild());
        if(checker_->checkConnection(conn)) cost += metrics_->cost(conf,this_conn->getChild()->getConfiguration());
        else cost = std::numeric_limits<double>::infinity();
      }
      else cost += metrics_->cost(conf,this_conn->getChild()->getConfiguration());
    }
  }
  return cost;
}

double Path::getNormFromConf(const Eigen::VectorXd &conf)
{
  double norm = 0;
  int idx;
  ConnectionPtr this_conn = findConnection(conf,idx);

  if(this_conn == NULL)
  {
    ROS_ERROR("norm can't be computed");
    assert(0);
    return 0;
  }
  else
  {
    norm += (conf-this_conn->getChild()->getConfiguration()).norm();
    if(idx < connections_.size()-1)
    {
      for(unsigned int i=idx+1;i<connections_.size();i++) norm += connections_.at(i)->norm();
    }
  }

  return norm;
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

bool Path::warp(const double &min_dist, const double &max_time)
{
  if(max_time > 0)
  {
    ros::WallTime tic = ros::WallTime::now();
    for (unsigned int idx = 1; idx < connections_.size(); idx++)
    {
      if(connections_.at(idx-1)->norm()>min_dist && connections_.at(idx)->norm()>min_dist)
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

      if((ros::WallTime::now()-tic).toSec()>=0.98*max_time) break;
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


ConnectionPtr Path::findConnection(const Eigen::VectorXd& configuration)
{
  int idx;
  return findConnection(configuration,idx);
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

    if(std::abs(dist-dist1-dist2)<1.0e-04)
    {
      conn = connections_.at(i);
      idx = i;
      return conn;
    }
  }

  ROS_ERROR("Connection not found");
  ROS_INFO_STREAM("conf: "<<configuration.transpose());
  ROS_INFO_STREAM("parent0: "<<connections_.at(0)->getParent()->getConfiguration().transpose());
  ROS_INFO_STREAM("child0: "<<connections_.at(0)->getChild()->getConfiguration().transpose());

  return NULL;
}

Eigen::VectorXd Path::projectOnConnection(const Eigen::VectorXd& point, const ConnectionPtr &conn, double& distance, bool& in_conn)
{
  Eigen::VectorXd parent = conn->getParent()->getConfiguration();
  Eigen::VectorXd child = conn->getChild()->getConfiguration();

  Eigen::VectorXd conn_vector = child-parent;
  Eigen::VectorXd point_vector = point-parent;

  double conn_length = (conn_vector).norm();
  assert(conn_length>0);

  double point_length = (point_vector).norm();
  double s = point_vector.dot(conn_vector/conn_length);

  Eigen::VectorXd projection = parent + conn_vector*s/conn_length;
  distance = std::sqrt(point_length*point_length-s*s);

  if(s>=0 && s<=conn_length)
  {
    in_conn = 1;
  }
  else
    in_conn = 0;

  if(conn == connections_.front() && s<0)  //if the point is before the start it is projected on the start
  {
    projection = parent; //the start
    in_conn = 1;
  }

  if(conn == connections_.back() && s>conn_length)  //if the point is before the start it is projected on the start
  {
    projection = child;  //the goal
    in_conn = 1;
  }

  if (in_conn)
  {
    int index=0;
    if (findConnection(projection,index)==NULL)
    {
      ROS_ERROR("distance = %f, s=%f, conn_length=%f",distance,s,conn_length);
    }
  }

  return projection;

  // //////////////////////////////////////////////////
  /*  double a = (parent-child).norm();
  double b = (parent-point).norm();
  double c = (point-child).norm();

  double p = (a+b+c)/2.0;
  double A = std::sqrt(p*(p-a)*(p-b)*(p-c)); //Erone's formula

  distance = 2.0*A/a;  //A=a*h/2 -> h=2A/a

  double c_pitagora = std::sqrt(b*b+a*a);
  double s; //distance parent-projection

  if(c<=c_pitagora) s = std::sqrt(b*b-distance*distance);
  else s = - std::sqrt(b*b-distance*distance);

  Eigen::VectorXd projection = parent+(child-parent)*s/a;

  if(s/a>=0 && s/a<=1)
  {
    in_conn = 1;
  }
  else if(conn == connections_.front() && s/a<0)  //if the point is before the start it is projected on the start
  {
    projection = parent; //the start
    in_conn = 1;
  }
  else if(conn == connections_.back() && s/a>1)  //if the point is before the start it is projected on the start
  {
    projection = child;  //the goal
    in_conn = 1;
  }
  else
  {
    in_conn = 0;
  }

  return projection;*/
}

const Eigen::VectorXd Path::projectOnClosestConnection(const Eigen::VectorXd& point)
{
  //The point is projected on the connection on which its projection is between the parent and the child and from which the distance is the smallest one.

  Eigen::VectorXd pr;
  Eigen::VectorXd projection;
  double min_distance = std::numeric_limits<double>::infinity();

  for(const ConnectionPtr conn:connections_)
  {
    double distance;
    bool in_connection;
    pr = projectOnConnection(point,conn,distance,in_connection);

    if(in_connection)
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
  }
  return projection;
}

const Eigen::VectorXd Path::projectOnClosestConnectionKeepingPastPrj(const Eigen::VectorXd& point, const Eigen::VectorXd &past_prj, int& n_conn)
{
  //The point is projected on the connection on which its projection is between the parent and the child and from which the distance is the smallest one.
  //The point can stay on the same connection or can move the next one. If a projection is not found, the past one is used.

  int idx;
  Eigen::VectorXd pr;
  Eigen::VectorXd projection;
  double min_distance = std::numeric_limits<double>::infinity();

  ConnectionPtr conn;
  for(unsigned int i=0;i<connections_.size();i++)
  {
    conn = connections_.at(i);

    double distance;
    bool in_connection;
    pr = projectOnConnection(point,conn,distance,in_connection);

    if(in_connection)
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
    projection = past_prj;
    //ROS_ERROR("projection on path not found");
  }
  else  n_conn = idx;

  return projection;
}

const Eigen::VectorXd Path::projectOnClosestConnectionKeepingCurvilinearAbscissa(const Eigen::VectorXd& point, Eigen::VectorXd &past_prj, double &new_abscissa, double &past_abscissa, int &n_conn, const bool &verbose)
{
  //The point is projected on the connection on which its projection is between the parent and the child and from which the distance is the smallest one.
  //The curvilinear abscissa of the point can't decrease. If it happens, the past projection is considered.

  int idx;
  Eigen::VectorXd pr;
  Eigen::VectorXd projection;
  double min_distance = std::numeric_limits<double>::infinity();

  if(verbose)
  {
    ROS_INFO_STREAM("last conn: "<<n_conn<<" point: "<<point.transpose()<<" past prj: "<<past_prj.transpose());
  }

  ConnectionPtr conn;
  for(unsigned int i=0;i<connections_.size();i++)
  {
    conn = connections_.at(i);

    double distance;
    bool in_connection;
    pr = projectOnConnection(point,conn,distance,in_connection);

    if(verbose) ROS_INFO_STREAM("conn number: "<< i<<" in_conn: "<<in_connection);

    if(in_connection)
    {
      if(i==n_conn || i==n_conn+1)
      {
        if(distance<min_distance)
        {
          double abscissa = curvilinearAbscissaOfPointGivenConnection(pr,i);

          if(verbose) ROS_INFO_STREAM("distance: "<<distance<<" min_dist: "<<min_distance<< " abscissa: "<<abscissa<< " past_abscissa"<< past_abscissa);

          if(abscissa>=past_abscissa)
          {
            if(verbose) ROS_INFO("New candidate");

            new_abscissa = abscissa;
            min_distance = distance;
            projection = pr;
            idx = i;
          }
          else if(verbose) ROS_WARN("Not a candidate");
        }
      }
    }

    if(verbose) ROS_INFO("------------------------------");
  }

  if(min_distance == std::numeric_limits<double>::infinity())
  {
    new_abscissa = past_abscissa;
    projection = past_prj;
    ROS_ERROR("projection on path not found");
    //ROS_INFO_STREAM("projection: "<<projection.transpose());
  }
  else  n_conn = idx;

  return projection;
}

NodePtr Path::addNodeAtCurrentConfig(const Eigen::VectorXd& configuration, ConnectionPtr& conn, const bool& rewire)
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
        tree_->addNode(actual_node);  //CHIEDI se necessario

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

NodePtr Path::findCloserNode(const Eigen::VectorXd& configuration, double &dist)
{
  if(connections_.size()<1)
  {
    ROS_ERROR("No connections");
    throw std::invalid_argument("No connections");
  }

  NodePtr closest_node=connections_.at(0)->getParent();
  double min_dist = (closest_node->getConfiguration()-configuration).norm();
  for (const ConnectionPtr& conn: connections_)
  {
    dist = (conn->getChild()->getConfiguration()-configuration).norm();
    if(dist<min_dist)
    {
      closest_node=conn->getChild();
      min_dist=dist;
    }
  }
  dist = min_dist;
  return closest_node;
}

NodePtr Path::findCloserNode(const Eigen::VectorXd& configuration)
{
  double distance;
  return findCloserNode(configuration,distance);
}

NodePtr Path::findCloserNode(const NodePtr& node)
{
  Eigen::VectorXd configuration = node->getConfiguration();
  return findCloserNode(configuration);
}

NodePtr Path::findCloserNode(const NodePtr& node, double &dist)
{
  Eigen::VectorXd configuration = node->getConfiguration();
  return findCloserNode(configuration,dist);
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
  bool simplified=false;
  bool reconnect_first_conn = false;

  if(connections_.size()>1)
  {
    double dist = (connections_.at(0)->getParent()->getConfiguration() - connections_.at(0)->getChild()->getConfiguration()).norm();
    if(dist < distance) reconnect_first_conn = true;
  }

  unsigned int ic = 1;
  while (ic < connections_.size())
  {
    double dist = (connections_.at(ic)->getParent()->getConfiguration() - connections_.at(ic)->getChild()->getConfiguration()).norm();
    if (dist > distance )
    {
      if(!(ic == 1 && reconnect_first_conn))
      {
        ic++;
        continue;
      }
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
      change_slip_parent_.erase(change_slip_parent_.begin() + ic);
      change_slip_child_.erase(change_slip_child_.begin() + ic);
      if (ic>1)
      {
        change_slip_child_.at(ic - 1) = 1;
        change_warp_.at(ic - 1) = 1;
        change_slip_parent_.at(ic - 1) = 1;
      }
      /*change_warp_.erase(change_warp_.begin() + ic-1);   //se da ancora problemi prova a usare questi anziche quelli sopra
      change_warp_.at(ic - 1) = 1;
      change_slip_parent_.erase(change_slip_parent_.begin() + ic-1);
      change_slip_parent_.at(ic - 1) = 1;
      change_slip_child_.erase(change_slip_child_.begin() + ic-1);
      change_slip_child_.at(ic - 1) = 1;*/

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


bool Path::isValid(const CollisionCheckerPtr &this_checker)
{
  CollisionCheckerPtr checker = checker_;
  if(this_checker != NULL) checker = this_checker;

  bool valid = isValidFromConn(connections_.at(0),checker);

  if(!valid) cost_ = std::numeric_limits<double>::infinity();
  else computeCost();

  return valid;
}

bool Path::isValidFromConn(const ConnectionPtr& this_conn, const CollisionCheckerPtr &this_checker)
{
  CollisionCheckerPtr checker = checker_;
  if(this_checker != NULL) checker = this_checker;

  bool valid = true;
  Eigen::VectorXd parent;
  Eigen::VectorXd child;
  double cost;
  bool from_here = false;

  for(const ConnectionPtr &conn : connections_)
  {
    if(this_conn == conn) from_here = true;

    if(from_here)
    {
      if(!checker->checkConnection(conn))
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
  }

  return valid;
}

bool Path::isValidFromConf(const Eigen::VectorXd &conf, const CollisionCheckerPtr &this_checker)
{
  int pos_closest_obs_from_goal = -1;
  bool validity = isValidFromConf(conf,pos_closest_obs_from_goal,this_checker);
  return validity;
}

bool Path::isValidFromConf(const Eigen::VectorXd &conf, int &pos_closest_obs_from_goal, const CollisionCheckerPtr &this_checker)
{
  CollisionCheckerPtr checker = checker_;
  if(this_checker != NULL) checker = this_checker;

  pos_closest_obs_from_goal = -1;
  bool validity = true;
  int idx;
  ConnectionPtr conn = findConnection(conf,idx);

  if(conn == NULL) assert(0);

  if(conf == conn->getParent()->getConfiguration())
  {
    validity = isValidFromConn(conn,checker);

    if(!validity)
    {
      for(int i = (connections_.size()-1);i>=idx;i--)
      {
        if(connections_.at(i)->getCost() == std::numeric_limits<double>::infinity()) pos_closest_obs_from_goal = connections_.size()-1-i;
      }
    }
  }
  else if(conf == conn->getChild()->getConfiguration())
  {
    if(idx<connections_.size()-1)
    {
      conn = connections_.at(idx+1);
      validity = isValidFromConn(conn,checker);

      if(!validity)
      {
        for(int i = (connections_.size()-1);i>=idx+1;i--)
        {
          if(connections_.at(i)->getCost() == std::numeric_limits<double>::infinity()) pos_closest_obs_from_goal = connections_.size()-1-i;
        }
      }
    }
    else
    {
      ROS_INFO("conf is equal to goal, no connection to validate from here");
      validity = true;
      assert(0);
    }
  }
  else
  {
    if(!checker->checkConnFromConf(conn,conf))
    {
      validity = false;
      conn->setCost(std::numeric_limits<double>::infinity());
      pos_closest_obs_from_goal = connections_.size()-1-idx;
    }

    if(idx<connections_.size()-1)  //also if the checker has failed, this check is important to update the cost of all the connections
    {
      if(!isValidFromConn(connections_.at(idx+1),checker))
      {
        validity = false;
        for(int i = (connections_.size()-1);i>=idx+1;i--)
        {
          if(connections_.at(i)->getCost() == std::numeric_limits<double>::infinity()) pos_closest_obs_from_goal = connections_.size()-1-i;
        }
      }
    }
  }

  return validity;
}

XmlRpc::XmlRpcValue Path::toXmlRpcValue(bool reverse) const
{
  XmlRpc::XmlRpcValue x;
  if (connections_.size()==0)
    return x;
  x.setSize(connections_.size()+1);

  if (not reverse)
  {
    x[0]=connections_.at(0)->getParent()->toXmlRpcValue();
    for (size_t idx=0;idx<connections_.size();idx++)
      x[idx+1]=connections_.at(idx)->getChild()->toXmlRpcValue();
  }
  else
  {
    x[0]=connections_.back()->getChild()->toXmlRpcValue();
    for (size_t idx=0;idx<connections_.size();idx++)
      x[idx+1]=connections_.at(connections_.size()-idx-1)->getParent()->toXmlRpcValue();
  }
  return x;
}

void Path::flip()
{
  for (ConnectionPtr conn: connections_)
    conn->flip();
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
