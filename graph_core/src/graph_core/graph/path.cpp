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
#include <iostream>
#include <graph_core/graph/path.h>

namespace graph
{
namespace core
{
Path::Path(std::vector<ConnectionPtr> connections,
           const MetricsPtr& metrics,
           const CollisionCheckerPtr& checker,
           const cnr_logger::TraceLoggerPtr &logger):
  connections_(connections),
  metrics_(metrics),
  checker_(checker),
  logger_(logger)
{
  assert(connections_.size() > 0);

  start_node_ = connections_.front()->getParent();
  goal_node_  = connections_.back ()->getChild ();

  cost_ = 0;

  NodePtr previous_child = nullptr;
  for (const ConnectionPtr& conn : connections_)
  {
    cost_ += conn->getCost();

    if(previous_child)
    {
      if(previous_child != conn->getParent())
      {
        for(const ConnectionPtr& c : connections_)
          CNR_WARN(logger_,*c);

        CNR_FATAL(logger_,"parent of a connection is different from the child of the previous connection!");
        throw std::runtime_error("parent of a connection is different from the child of the previous connection!");
      }
    }
    previous_child = conn->getChild();
  }
}

Path::Path(std::vector<NodePtr> nodes,
           const MetricsPtr& metrics,
           const CollisionCheckerPtr& checker,
           const cnr_logger::TraceLoggerPtr &logger):
  metrics_(metrics),
  checker_(checker),
  logger_(logger)
{
  assert(nodes.size() > 0);

  start_node_ = nodes.front();
  goal_node_  = nodes.back ();

  connections_.clear();
  cost_ = 0;
  for(unsigned int i=0;i<nodes.size()-1;i++)
  {
    NodePtr parent = nodes.at(i);
    NodePtr child  = nodes.at(i+1);

    ConnectionPtr conn;
    (child->getParentConnectionsSize() == 0)? (conn = std::make_shared<Connection>(parent,child,logger_,false)):
                                              (conn = std::make_shared<Connection>(parent,child,logger_,true));
    double cost = metrics->cost(parent,child);
    conn->setCost(cost);
    conn->add();

    assert(child->getParentConnectionsSize() == 1);

    connections_.push_back(conn);

    cost_ += cost;
  }
}

PathPtr Path::clone()
{
  ConnectionPtr conn;
  NodePtr start, parent, child;  //start keep alive the first node (not pointed by any shared ptr)
  std::vector<ConnectionPtr> new_conn_vector;

  std::vector<Eigen::VectorXd> wp = getWaypoints();
  start = std::make_shared<Node>(wp.at(0),logger_);
  parent = start;

  for(unsigned int i = 1;i<wp.size();i++)
  {
    child = std::make_shared<Node>(wp.at(i),logger_);

    conn = std::make_shared<Connection>(parent,child,logger_);
    conn->setCost(connections_.at(i-1)->getCost());  //update also the internal time of the cloned connection
    conn->setTimeCostUpdate(connections_.at(i-1)->getTimeCostUpdate());  //set the internal time to the value of the original connection
    conn->add();

    assert(child->getParentConnectionsSize()    == 1);
    assert(child->getNetParentConnectionsSize() == 0);
    assert(child->getChildConnectionsSize()     == 0);
    assert(child->getNetChildConnectionsSize()  == 0);

    new_conn_vector.push_back(conn);
    parent = child;                   //NB: parent of connection i+1 must be the child (same object) of connection i
  }

  //  PathPtr new_path = std::make_shared<Path>(new_conn_vector,metrics_,checker_);
  PathPtr new_path = std::make_shared<Path>(new_conn_vector,metrics_->clone(),checker_->clone(),logger_);

  new_path->setTree(nullptr);  //nodes are cloned, so the cloned path does not belong to the original tree

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
  ConnectionPtr conn;
  return pointOnCurvilinearAbscissa(abscissa,conn);
}

Eigen::VectorXd Path::pointOnCurvilinearAbscissa(const double& abscissa, ConnectionPtr& connection)
{
  assert(connections_.size() > 0);
  if (abscissa <= 0.0)
  {
    connection = connections_.front();
    return connection->getParent()->getConfiguration();
  }

  double point = abscissa*computeEuclideanNorm();
  double euclidean_norm = 0.0;
  for(const ConnectionPtr& conn : connections_)
  {
    if((euclidean_norm + conn->norm()) > point)
    {
      double ratio = (point - euclidean_norm)/conn->norm();
      assert(ratio>=0 && ratio<=1);

      connection = conn;
      return conn->getParent()->getConfiguration()+ratio*(conn->getChild()->getConfiguration()-conn->getParent()->getConfiguration());
    }
    euclidean_norm += conn->norm();
  }

  connection = connections_.back();
  return goal_node_->getConfiguration();
}

double Path::curvilinearAbscissaOfPoint(const Eigen::VectorXd& conf, size_t& idx)
{
  double abscissa = std::numeric_limits<double>::infinity();

  ConnectionPtr connection = findConnection(conf,idx);
  if(connection == nullptr)
  {
    CNR_ERROR(logger_, "The configuration does not belong to the path -> the curvilinear abscissa can not be computed");
    CNR_INFO (logger_, "conf: "<<conf.transpose());
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
  size_t idx;
  return curvilinearAbscissaOfPoint(conf,idx);
}

double Path::curvilinearAbscissaOfPointGivenConnection(const Eigen::VectorXd& conf, const size_t &conn_idx)
{
  double abscissa = std::numeric_limits<double>::infinity();

  if(conn_idx >= connections_.size())
  {
    CNR_ERROR(logger_,"The connection does not belong to the path -> the curvilinear abscissa can not be computed");
    CNR_INFO (logger_,"conn_idx: "<<conn_idx);
    return abscissa;
  }

  double euclidean_norm = 0.0;
  double euclidean_norm_to_parent = -1.0;
  for(size_t i=0;i<connections_.size();i++)
  {
    if(i == conn_idx)
      euclidean_norm_to_parent = euclidean_norm;

    euclidean_norm += connections_.at(i)->norm();
  }

  assert(euclidean_norm_to_parent >= 0);

  double dist = (connections_.at(conn_idx)->getParent()->getConfiguration()-conf).norm();
  abscissa = (euclidean_norm_to_parent+dist)/euclidean_norm;

  return abscissa;
}

double Path::getCostFromConf(const Eigen::VectorXd &conf)
{
  computeCost();
  double cost = 0;
  size_t idx;
  ConnectionPtr this_conn = findConnection(conf,idx);

  if(this_conn == nullptr)
  {
    CNR_ERROR(logger_,"cost can't be computed");
    return 0;
  }
  else
  {
    if(conf == connections_.at(0)->getParent()->getConfiguration())
      return cost_;

    if(idx < connections_.size()-1)
    {
      for(unsigned int i=idx+1;i<connections_.size();i++)
      {
        cost += connections_.at(i)->getCost();
        if(cost == std::numeric_limits<double>::infinity())
          return std::numeric_limits<double>::infinity();
      }
    }

    if(conf == this_conn->getParent()->getConfiguration())
      cost += this_conn->getCost();
    else if (conf == this_conn->getChild()->getConfiguration())
      cost += 0.0;
    else
    {
      if(this_conn->getCost() == std::numeric_limits<double>::infinity())
      {
        if(checker_->checkConnection(conf,this_conn->getChild()->getConfiguration()))
          cost += metrics_->cost(conf,this_conn->getChild()->getConfiguration());
        else
          cost = std::numeric_limits<double>::infinity();
      }
      else
        cost += metrics_->cost(conf,this_conn->getChild()->getConfiguration());
    }
  }
  return cost;
}

double Path::getNormFromConf(const Eigen::VectorXd &conf)
{
  double norm = 0;
  size_t idx;
  ConnectionPtr this_conn = findConnection(conf,idx);

  if(this_conn == nullptr)
  {
    CNR_ERROR(logger_,"norm can't be computed");
    assert(0);
    return 0;
  }
  else
  {
    norm += (conf-this_conn->getChild()->getConfiguration()).norm();
    if(idx < connections_.size()-1)
    {
      for(unsigned int i=idx+1;i<connections_.size();i++)
        norm += connections_.at(i)->norm();
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

std::vector<NodePtr> Path::getNodes() const
{
  std::vector<NodePtr> nodes;
  if (connections_.size() == 0)
    return nodes;

  nodes.push_back(connections_.at(0)->getParent());
  for (const ConnectionPtr& conn : connections_)
    nodes.push_back(conn->getChild());

  return nodes;
}

std::vector<Eigen::VectorXd> Path::getWaypoints() const
{
  std::vector<Eigen::VectorXd> wp;
  if(connections_.empty())
    return wp;

  wp.push_back(connections_.at(0)->getParent()->getConfiguration());
  for (const ConnectionPtr& conn : connections_)
    wp.push_back(conn->getChild()->getConfiguration());

  return wp;
}

ConnectionPtr Path::getConnection(const size_t& i)const
{
  if(i<0 || i>=connections_.size())
  {
    CNR_WARN(logger_,"idx exceeds 'connections_' bounds");
    return nullptr;
  }
  return connections_.at(i);
}

void Path::setConnections(const std::vector<ConnectionPtr>& conn)
{
  cost_ = 0;

  NodePtr child = nullptr;

  for(const ConnectionPtr& connection : conn)
  {
    cost_ += connection->getCost();

    if(child)
    {
      assert(child->getParentConnectionsSize() == 1);

      if(child != connection->getParent())
      {
        for(const ConnectionPtr& c : conn)
          CNR_WARN(logger_,*c);

        CNR_FATAL(logger_,"parent of a connection is different from the child of the previous connection!");
        throw std::runtime_error("parent of a connection is different from the child of the previous connection!");
      }
    }
    child = connection->getChild();
  }

  start_node_ = conn.front()->getParent();
  goal_node_  = conn.back ()->getChild ();

  connections_ = conn;
}

ConnectionPtr Path::findConnection(const Eigen::VectorXd& configuration)
{
  size_t idx;
  return findConnection(configuration,idx);
}

ConnectionPtr Path::findConnection(const Eigen::VectorXd& configuration, size_t& idx, bool verbose)
{
  ConnectionPtr conn = nullptr;

  // Check if a node corresponds to configuration. This is useful when you have overlapping connections
  std::vector<NodePtr> nodes = getNodes();
  std::vector<NodePtr>::iterator it = std::find_if(nodes.begin(),nodes.end(),[&](NodePtr& n) ->bool{
      return (configuration-n->getConfiguration()).norm()<=TOLERANCE;});

  if(it<nodes.end())
  {
    int it_distance = std::distance(nodes.begin(),it);
    if(it_distance == 0)
      idx = 0;
    else
      idx = it_distance-1;

    conn = connections_.at(idx);
    return conn;
  }

  // Check for connections
  Eigen::VectorXd parent;
  Eigen::VectorXd child;

  double dist, distP, distC, err;

  for(unsigned int i=0; i<connections_.size(); i++)
  {
    parent = connections_.at(i)->getParent()->getConfiguration();
    child  = connections_.at(i)->getChild() ->getConfiguration();

    dist  = (parent-child)        .norm();
    distP = (parent-configuration).norm();
    distC = (configuration-child) .norm();

    err  = std::abs(dist-distP-distC);

    if(verbose)
      CNR_INFO(logger_,"dist %f, distP %f, distC %f, err %lf",dist,distP,distC,err);

    if(err<1e-10)
    {
      conn = connections_.at(i);
      idx = i;
      return conn;
    }
  }

  if(verbose)
  {
    CNR_INFO(logger_, "Connection not found");
    CNR_INFO (logger_, "conf: "<<configuration.transpose());
    CNR_INFO (logger_, "parent0: "<<connections_.at(0)->getParent()->getConfiguration().transpose());
    CNR_INFO (logger_, "child0: "<<connections_.at(0)->getChild()->getConfiguration().transpose());
  }

  return nullptr;
}

Eigen::VectorXd Path::projectOnClosestConnection(const Eigen::VectorXd& point, const bool verbose)
{
  Eigen::VectorXd pr;
  Eigen::VectorXd projection;
  double min_distance = std::numeric_limits<double>::infinity();

  for(const ConnectionPtr& conn:connections_)
  {
    double distance;
    bool in_connection;
    pr = conn->projectOnConnection(point,distance,in_connection,verbose);

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
    CNR_ERROR(logger_,"projection on path not found");
  }

  return projection;
}

Eigen::VectorXd Path::projectOnPath(const Eigen::VectorXd& point, const bool& verbose)
{
  Eigen::VectorXd past_projection = start_node_->getConfiguration();
  return projectOnPath(point,past_projection,verbose);
}

Eigen::VectorXd Path::projectOnPath(const Eigen::VectorXd& point, const Eigen::VectorXd &past_projection, const bool& verbose)
{
  ConnectionPtr conn;
  return projectOnPath(point,past_projection,conn,verbose);
}

Eigen::VectorXd Path::projectOnPath(const Eigen::VectorXd& point, const Eigen::VectorXd &past_projection, ConnectionPtr& conn, const bool& verbose)
{
  ConnectionPtr candidate_connection, connection;
  conn = nullptr;
  Eigen::VectorXd candidate_projection, projection, precise_projection;
  double abscissa, candidate_abscissa, candidate_distance, min_distance, ds;

  bool last_run = false;

  if(verbose)
    CNR_INFO(logger_,"path "<<*this);

  ds = 0.001;

  candidate_abscissa = curvilinearAbscissaOfPoint(past_projection);
  min_distance = std::numeric_limits<double>::infinity();

  while(candidate_abscissa<=1)
  {
    candidate_projection = pointOnCurvilinearAbscissa(candidate_abscissa,candidate_connection);
    candidate_distance = (candidate_projection-point).norm();

    if(verbose)
      CNR_INFO(logger_,"dist from past projection "<<(candidate_projection-past_projection).norm()<<" s "<<candidate_abscissa<<" ds "<<ds<<" distance "<<candidate_distance<<" min distance "<<min_distance);

    if(candidate_distance<min_distance)
    {
      min_distance = candidate_distance;
      projection = candidate_projection;
      abscissa = candidate_abscissa;
      connection = candidate_connection;
    }

    if(last_run)
      break;

    candidate_abscissa = candidate_abscissa+ds;
    if(candidate_abscissa>1)
    {
      candidate_abscissa = 1;
      last_run = true;
    }
  }

  if(verbose)
  {
    CNR_INFO(logger_,"abscissa "<<abscissa<< " projection "<<projection.transpose());
    CNR_INFO(logger_,"projection with conn "<<*connection);
  }

  conn = connection;

  if(connection->norm()<1e-03)
  {
    if(verbose)
      CNR_INFO(logger_,"Connection too short, keep approximated projection "<<connection->norm());

    return projection;
  }

  bool in_conn;
  double distance;
  precise_projection = connection->projectOnConnection(point,distance,in_conn,verbose);
  if(not in_conn)
  {
    precise_projection = projection;
    if(verbose)
      CNR_INFO(logger_,"keep approximated projection");
  }

  return precise_projection;
}


bool Path::removeNodes(const double &toll)
{
  std::vector<NodePtr> deleted_nodes, white_list;
  return removeNodes(white_list, deleted_nodes,toll);
}

bool Path::removeNodes(const std::vector<NodePtr> &white_list, const double& toll)
{
  std::vector<NodePtr> deleted_nodes;
  return removeNodes(white_list,deleted_nodes,toll);
}

bool Path::removeNode(const NodePtr& node, const size_t& idx_conn, const std::vector<NodePtr> &white_list, ConnectionPtr& new_conn, const double& toll)
{
  if(node == start_node_ || node == goal_node_)
    return false;

  if(std::find(white_list.begin(),white_list.end(),node)<white_list.end())
    return false;

  if(idx_conn<0 || idx_conn>(connections_.size()-1))
  {
    CNR_INFO(logger_,"Node does not belong to the path");
    return false;
  }

  ConnectionPtr conn_parent_node = connections_.at(idx_conn);
  ConnectionPtr conn_node_child  = connections_.at(idx_conn+1);

  bool parallel = conn_parent_node->isParallel(conn_node_child, toll);
  bool parent_cond = ((node->getParentConnectionsSize()+node->getNetParentConnectionsSize()) == 1);
  bool child_cond = ((node->getChildConnectionsSize ()+node->getNetChildConnectionsSize ()) == 1);

  if(parallel && parent_cond && child_cond)
  {
    assert(not tree_ || node != tree_->getRoot()); //node must have 1 parent (root must have zero) and 1 child (root may have many)

    bool is_net = conn_node_child->isNet();
    new_conn = std::make_shared<Connection>(conn_parent_node->getParent(),conn_node_child->getChild(),logger_,is_net);
    double cost = conn_parent_node->getCost()+conn_node_child->getCost();
    new_conn->setCost(cost);
    new_conn->setTimeCostUpdate(std::min(conn_parent_node->getTimeCostUpdate(),conn_node_child->getTimeCostUpdate())); //consider the min between the two internal times
    new_conn->add();

    node->disconnect();

    assert(new_conn->getChild()->getParentConnectionsSize() == 1);

    if(tree_)
      tree_->removeNode(node);

    std::vector<ConnectionPtr>::iterator it = connections_.begin()+idx_conn;
    assert((*it)->getChild() == node);
    *it = new_conn;
    it = connections_.erase(it+1);

    setConnections(connections_);
    return true;
  }

  return false;
}

bool Path::removeNode(const NodePtr& node, const size_t& idx_conn, const std::vector<NodePtr> &white_list)
{
  ConnectionPtr new_conn;
  return removeNode(node,idx_conn,white_list,new_conn);
}

bool Path::removeNode(const NodePtr& node, const std::vector<NodePtr> &white_list)
{
  ConnectionPtr new_conn;
  return removeNode(node,white_list,new_conn);
}

bool Path::removeNode(const NodePtr& node, const std::vector<NodePtr> &white_list, ConnectionPtr& new_conn)
{
  if(node == start_node_ || node == goal_node_)
    return false;

  ConnectionPtr conn_parent_node,conn_node_child;
  int idx = -1;

  for(unsigned int i=0;i<connections_.size()-1;i++)
  {
    if(node == connections_.at(i)->getChild())
    {
      conn_parent_node = connections_.at(i);
      conn_node_child = connections_.at(i+1);
      idx = i;
    }
  }

  if(idx<0)
  {
    CNR_ERROR(logger_,"Node does not belong to the path");
    return false;
  }

  return removeNode(node,idx,white_list,new_conn);
}

bool Path::splitConnection(const ConnectionPtr& conn1, const ConnectionPtr& conn2, const std::vector<ConnectionPtr>::iterator& it)
{
  if((*it)->getParent() != conn1->getParent() ||
     (*it)->getChild () != conn2->getChild () ||
     conn1->getChild () != conn2->getParent())
  {
    CNR_FATAL(logger_,"no matching of parent and/or child nodes");
    throw std::invalid_argument("no matching of parent and/or child nodes");
  }

  if(it<connections_.end())
  {
    *it = conn1;
    connections_.insert(it+1,conn2);

    setConnections(connections_); //update members

    if(tree_)
      tree_->addNode(conn1->getChild(),true);

    return true;
  }
  return false;
}

bool Path::splitConnection(const ConnectionPtr& conn1, const ConnectionPtr& conn2, const ConnectionPtr& conn)
{
  std::vector<ConnectionPtr>::iterator it = std::find(connections_.begin(),connections_.end(),conn);
  if(it<connections_.end())
    return splitConnection(conn1,conn2,it);
  else
    return false;
}

bool Path::restoreConnection(const ConnectionPtr& conn, const NodePtr& node2remove)
{
  std::vector<ConnectionPtr>::iterator it = std::find_if(connections_.begin(),connections_.end(),[&](ConnectionPtr c)->bool{return (c->getChild() == node2remove);});

  if(it>=connections_.end())
    return false;

  int idx = std::distance(connections_.begin(),it);

  assert(idx>=0);
  assert(connections_.at(idx) == *it);

  if((conn->getParent() != connections_.at(idx)->getParent()) || (conn->getChild() != connections_.at(idx+1)->getChild()))
  {
    CNR_INFO(logger_,"the node to remove is not on the connection to restore!");
    CNR_INFO(logger_,"conn "<<*conn);
    CNR_INFO(logger_,"conn parent "<<*connections_.at(idx));
    CNR_INFO(logger_,"conn child "<<*connections_.at(idx+1));

    return false;
  }

  *it = conn;
  connections_.erase(it+1);

  setConnections(connections_);

  if(tree_)
    tree_->removeNode(node2remove);

  node2remove->disconnect();

  return true;
}

bool Path::removeNodes(const std::vector<NodePtr> &white_list, std::vector<NodePtr> &deleted_nodes, const double& toll)
{
  bool removed;
  NodePtr node;
  std::vector<NodePtr> void_list;
  bool at_least_one_removed = false;
  ConnectionPtr conn_parent_node, conn_node_child, new_conn;

  do
  {
    removed = false;

    for(size_t i=0;i<connections_.size()-1;i++)
    {
      conn_parent_node = connections_.at(i);
      conn_node_child  = connections_.at(i+1);

      node = conn_parent_node->getChild();
      assert(node == conn_node_child->getParent());

      if(std::find(white_list.begin(),white_list.end(),node)<white_list.end())
        continue;

      if(removeNode(node,i,void_list,new_conn,toll))
      {
        removed = true;
        at_least_one_removed = true;
        deleted_nodes.push_back(node);

        break; //because connections_ is changed
      }
    }
  }while(removed);

  return at_least_one_removed;
}

int Path::resample(const double& max_distance)
{
  unsigned int pos = 0;
  unsigned int nodes_added = 0;
  unsigned int initial_size = connections_.size();

  double length;
  bool is_a_new_node;
  ConnectionPtr conn;
  Eigen::VectorXd conf;

  while(pos<connections_.size())
  {
    conn = connections_.at(pos);
    length = conn->norm();

    if(length>max_distance)
    {
      is_a_new_node = false;
      conf = conn->getParent()->getConfiguration()+(conn->getChild()->getConfiguration()-conn->getParent()->getConfiguration())*(max_distance/length);
      addNodeAtCurrentConfig(conf,conn,true,is_a_new_node);

      if(is_a_new_node)
      {
        conn->remove();
        nodes_added++;
      }
    }
    pos++;
  }

  assert([&]() ->bool{
           if(nodes_added == 0)
           return true;
           else
           {
             if((connections_.size()-initial_size) == nodes_added)
             return true;
             else
             {
               CNR_INFO(logger_,"\nconnections_ "<<connections_.size()<<"\ninitial size "<<initial_size<<"\nnodes_added "<<nodes_added);
               return false;
             }
           }
         }());

  return nodes_added;
}

NodePtr Path::addNodeAtCurrentConfig(const Eigen::VectorXd& configuration, ConnectionPtr &conn, const bool &rewire)
{
  bool is_a_new_node;
  return addNodeAtCurrentConfig(configuration,conn,rewire,is_a_new_node);
}

NodePtr Path::addNodeAtCurrentConfig(const Eigen::VectorXd& configuration, const bool& rewire)
{
  ConnectionPtr conn = findConnection(configuration);
  return addNodeAtCurrentConfig(configuration,conn,rewire);
}

NodePtr Path::addNodeAtCurrentConfig(const Eigen::VectorXd& configuration, ConnectionPtr& conn, const bool& rewire, bool& is_a_new_node)
{
  is_a_new_node = false;

  if(conn)
  {
    std::vector<ConnectionPtr>::iterator it = std::find(connections_.begin(),connections_.end(),conn);
    if(it == connections_.end())
    {
      CNR_INFO(logger_,"Not a connection of this path");
      return nullptr;
    }

    NodePtr parent = conn->getParent();
    NodePtr child = conn->getChild();

    if((parent->getConfiguration() - configuration).norm()<=TOLERANCE)
    {
      is_a_new_node = false;
      return parent;
    }
    else if((child ->getConfiguration() - configuration).norm()<=TOLERANCE)
    {
      is_a_new_node = false;
      return child;
    }
    else
    {
      NodePtr actual_node = std::make_shared<Node>(configuration,logger_);
      is_a_new_node = true;

      if(rewire)
      {
        double cost_parent, cost_child;
        if(conn->getCost() == std::numeric_limits<double>::infinity())
        {
          if(not checker_->check(actual_node->getConfiguration()))
          {
            cost_parent = std::numeric_limits<double>::infinity();
            cost_child  = std::numeric_limits<double>::infinity();
          }
          else
          {
            if(not checker_->checkConnection(actual_node->getConfiguration(),child->getConfiguration()))
            {
              cost_child = std::numeric_limits<double>::infinity();

              checker_->checkConnection(actual_node->getConfiguration(),parent->getConfiguration())?
                    (cost_parent = metrics_->cost(parent->getConfiguration(),actual_node->getConfiguration())):
                    (cost_parent = std::numeric_limits<double>::infinity());
            }
            else
            {
              cost_parent = std::numeric_limits<double>::infinity();
              cost_child  = metrics_->cost(actual_node->getConfiguration(),child->getConfiguration());
            }
          }
        }
        else
        {
          cost_parent = metrics_->cost(parent     ->getConfiguration(), actual_node->getConfiguration());
          cost_child  = metrics_->cost(actual_node->getConfiguration(), child      ->getConfiguration());
        }

        if(tree_)
          tree_->addNode(actual_node);

        ConnectionPtr conn_parent = std::make_shared<Connection>(parent, actual_node, logger_, false);
        conn_parent->setCost(cost_parent);
        conn_parent->add();

        ConnectionPtr conn_child = std::make_shared<Connection>(actual_node,child,logger_,conn->isNet());
        conn_child->setCost(cost_child);
        conn_child->add();

        conn->remove();

        assert(actual_node->getParentConnectionsSize() == 1);
        assert(child      ->getParentConnectionsSize() == 1);

        unsigned int size_before = connections_.size();
        splitConnection(conn_parent,conn_child,it);
        assert(connections_.size() == (size_before+1));
      }

      return actual_node;
    }
  }

  CNR_ERROR(logger_,"Connection not found, the node can't be created");

  return nullptr;
}

NodePtr Path::findCloserNode(const Eigen::VectorXd& configuration, double &dist)
{
  if(connections_.size()<1)
  {
    CNR_FATAL(logger_,"No connections");
    throw std::runtime_error("No connections");
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

PathPtr Path::getSubpathToConf(const Eigen::VectorXd& conf, const bool clone)
{
  //If get_copy, the node is not real added to the path and a copy of the subpath is returned
  //If !get_copy, the node is really added to the path, the path and the tree are rewired and the real subpath is returned

  //Conf is a path waypoint
  for(const Eigen::VectorXd& wp: getWaypoints())
  {
    if(conf == wp)
    {
      if(not clone)
        return getSubpathToNode(conf);
      else
        return getSubpathToNode(conf)->clone();
    }
  }

  //Conf is not a path waypoint
  PathPtr subpath;
  NodePtr node;
  size_t idx_conn;
  ConnectionPtr conn = findConnection(conf,idx_conn);

  if(not conn)
  {
    CNR_FATAL(logger_,"Conf does not belong to the path, subpath to conf can not be computed");
    throw std::invalid_argument("Conf does not belong to the path, subpath to conf can not be computed");
  }

  bool is_net = conn->isNet();

  if(not clone)
  {
    node = addNodeAtCurrentConfig(conf,conn,true);
    subpath = getSubpathToNode(node);
  }
  else  //a copy of the subpath will be created (nodes, connections..)
  {
    node = addNodeAtCurrentConfig(conf,conn,false);

    NodePtr parent;
    PathPtr path_to_parent;
    std::vector<ConnectionPtr> connections_to_parent;
    if(idx_conn > 0) //conf is not on the first connection
    {
      path_to_parent = (getSubpathToNode(conn->getParent()))->clone();
      connections_to_parent = path_to_parent->getConnections();
      parent = connections_to_parent.back()->getChild();
    }
    else
      parent = std::make_shared<Node>(conn->getParent()->getConfiguration(),logger_);

    double cost;
    if(conn->getCost() == std::numeric_limits<double>::infinity())
    {
      checker_->checkConnection(conn->getParent()->getConfiguration(),node->getConfiguration())?
            (cost  = metrics_->cost(conn->getParent()->getConfiguration(),node->getConfiguration())):
            (cost = std::numeric_limits<double>::infinity());
    }
    else
      cost  = metrics_->cost(conn->getParent()->getConfiguration(),node->getConfiguration());

    ConnectionPtr conn_parent;
    conn_parent = std::make_shared<Connection>(parent,node,logger_,is_net);
    conn_parent->setCost(cost);
    conn_parent->add();

    assert(node->getParentConnectionsSize() == 1);

    std::vector<ConnectionPtr> connections_vector;
    if(not connections_to_parent.empty())
      connections_vector = connections_to_parent;

    connections_vector.push_back(conn_parent);

    subpath = std::make_shared<Path>(connections_vector,metrics_,checker_,logger_);
  }

  return subpath;
}

PathPtr Path::getSubpathFromConf(const Eigen::VectorXd& conf, const bool clone)
{
  //If get_copy, the node is not real added to the path and a copy of the subpath is returned
  //If !get_copy, the node is really added to the path, the path and the tree are rewired and the real subpath is returned

  //Conf is a path waypoint
  for(const Eigen::VectorXd& wp: getWaypoints())
  {
    if(conf == wp)
    {
      if(not clone)
        return getSubpathFromNode(conf);
      else
        return getSubpathFromNode(conf)->clone();
    }
  }

  //Conf is not a path waypoint
  PathPtr subpath;
  NodePtr node;
  size_t idx_conn;
  ConnectionPtr conn = findConnection(conf,idx_conn);
  if(not conn)
  {
    CNR_FATAL(logger_,"Conf does not belong to the path, subpath from conf can not be computed");
    throw std::invalid_argument("Conf does not belong to the path, subpath from conf can not be computed");
  }

  bool is_net = conn->isNet();

  if(not clone)
  {
    node = addNodeAtCurrentConfig(conf,conn,true);
    subpath = getSubpathFromNode(node);
  }
  else  //a copy of the subpath will be created (nodes, connections..)
  {
    node = addNodeAtCurrentConfig(conf,conn,false);

    NodePtr child;
    PathPtr path_from_child;
    std::vector<ConnectionPtr> connections_from_child;
    if(idx_conn < connections_.size()-1) //conf is not on the last connection
    {
      path_from_child = getSubpathFromNode(conn->getChild())->clone();
      connections_from_child = path_from_child->getConnections();
      child = connections_from_child.front()->getParent();
    }
    else
    {
      child = std::make_shared<Node>(conn->getChild()->getConfiguration(),logger_);
    }

    double cost;
    if(conn->getCost() == std::numeric_limits<double>::infinity())
    {
      checker_->checkConnection(node->getConfiguration(),conn->getChild()->getConfiguration())?
            (cost  = metrics_->cost(node->getConfiguration(),conn->getChild()->getConfiguration())):
            (cost = std::numeric_limits<double>::infinity());
    }
    else
      cost  = metrics_->cost(node->getConfiguration(),conn->getChild()->getConfiguration());

    ConnectionPtr conn_child = std::make_shared<Connection>(node,child,logger_,is_net);
    conn_child->setCost(cost);
    conn_child->add();

    assert(child->getParentConnectionsSize() == 1);

    std::vector<ConnectionPtr> connections_vector;
    connections_vector.push_back(conn_child);

    if(not connections_from_child.empty())
      connections_vector.insert(connections_vector.end(),connections_from_child.begin(),connections_from_child.end());

    subpath = std::make_shared<Path>(connections_vector,metrics_,checker_,logger_);
  }

  return subpath;
}

PathPtr Path::getSubpathToNode(const NodePtr& node)
{
  return getSubpathToNode(node->getConfiguration());
}

PathPtr Path::getSubpathToNode(const Eigen::VectorXd& conf)
{
  if((conf-start_node_->getConfiguration()).norm()<TOLERANCE)
  {
    CNR_INFO(logger_,"configuration: "<<conf.transpose());
    CNR_INFO(logger_,"path:\n"<<*this);
    CNR_FATAL(logger_,"No subpath available, the node is equal to the first node of the path");

    throw std::invalid_argument("No subpath available, the node is equal to the first node of the path");
  }

  if((conf-goal_node_->getConfiguration()).norm()<TOLERANCE)
  {
    return this->pointer();
  }

  PathPtr subpath;
  for(unsigned int idx=0; idx<connections_.size(); idx++)
  {
    if((conf-connections_.at(idx)->getChild()->getConfiguration()).norm()<TOLERANCE)
    {
      std::vector<ConnectionPtr> conn;
      conn.assign(connections_.begin(), connections_.begin()+idx+1); // to save the idx connections

      subpath = std::make_shared<Path>(conn, metrics_,checker_,logger_);
      return subpath;
    }
  }

  CNR_INFO(logger_,"configuration: "<<conf.transpose());
  CNR_INFO(logger_,"path:\n"<<*this);
  CNR_FATAL(logger_,"The node doesn to belong to this path");
  throw std::invalid_argument("The node doesn to belong to this path");
}

PathPtr Path::getSubpathFromNode(const NodePtr& node)
{
  return getSubpathFromNode(node->getConfiguration());
}

PathPtr Path::getSubpathFromNode(const Eigen::VectorXd& conf)
{
  if((conf-goal_node_->getConfiguration()).norm()<TOLERANCE)
  {
    CNR_INFO(logger_,"configuration: "<<conf.transpose());
    CNR_INFO(logger_,"path:\n"<<*this);
    CNR_ERROR(logger_,"No subpath available, the node is equal to the last node of the path");

    throw std::invalid_argument("No subpath available, the node is equal to the last node of the path");
  }

  if((conf-start_node_->getConfiguration()).norm()<TOLERANCE)
  {
    return this->pointer();
  }

  PathPtr subpath;
  for(unsigned int idx=0; idx<connections_.size(); idx++)
  {
    if((conf-connections_.at(idx)->getChild()->getConfiguration()).norm()<TOLERANCE)
    {
      std::vector<ConnectionPtr> conn;
      conn.assign(connections_.begin()+idx+1, connections_.end());

      subpath = std::make_shared<Path>(conn, metrics_,checker_,logger_);
      return subpath;
    }
  }

  CNR_ERROR(logger_,"The node doesn't belong to this path");
  CNR_INFO(logger_,"configuration: "<<conf.transpose());
  CNR_INFO(logger_,"path:\n"<<*this);

  throw std::invalid_argument("The node doesn't belong to this path");

  return nullptr;
}

bool Path::isValid(const CollisionCheckerPtr &this_checker)
{
  CollisionCheckerPtr checker = checker_;
  if(this_checker != nullptr)
    checker = this_checker;

  bool valid = isValidFromConn(connections_.front(),checker);

  if(not valid)
    cost_ = std::numeric_limits<double>::infinity();
  else
    computeCost();

  assert([&]() ->bool{
           if(valid && cost_ == std::numeric_limits<double>::infinity())
           {
             return false;
           }

           return true;
         }());

  return valid;
}

bool Path::isValidFromConn(const ConnectionPtr& this_conn, const CollisionCheckerPtr &this_checker)
{
  CollisionCheckerPtr checker = checker_;
  if(this_checker != nullptr)
    checker = this_checker;

  bool valid = true;
  double cost;
  bool from_here = false;

  for(const ConnectionPtr &conn : connections_)
  {
    if(this_conn == conn)
      from_here = true;

    if(from_here)
    {
      if(not checker->checkConnection(conn))
      {
        conn->setCost(std::numeric_limits<double>::infinity());
        valid = false;
      }
      else
      {
        cost = metrics_->cost(conn->getParent()->getConfiguration(),conn->getChild()->getConfiguration());
        conn->setCost(cost);
      }
    }
  }

  return valid;
}

bool Path::isValidFromConf(const Eigen::VectorXd &conf, const CollisionCheckerPtr &this_checker)
{
  int pos_closest_obs_from_goal = -1;
  return isValidFromConf(conf,pos_closest_obs_from_goal,this_checker);
}

bool Path::isValidFromConf(const Eigen::VectorXd &conf, const size_t &conn_idx, const CollisionCheckerPtr &this_checker)
{
  int pos_closest_obs_from_goal = -1;
  return isValidFromConf(conf,conn_idx,pos_closest_obs_from_goal,this_checker);
}

bool Path::isValidFromConf(const Eigen::VectorXd &conf, const size_t& conn_idx, int &pos_closest_obs_from_goal, const CollisionCheckerPtr &this_checker)
{
  assert(conn_idx >= 0);

  ConnectionPtr conn = connections_.at(conn_idx);

  CollisionCheckerPtr checker = checker_;
  if(this_checker != nullptr)
    checker = this_checker;

  pos_closest_obs_from_goal = -1;
  bool validity = true;

  if(conf == conn->getParent()->getConfiguration())
  {
    validity = isValidFromConn(conn,checker);

    if(not validity)
    {
      for(size_t i = (connections_.size()-1);i>=conn_idx;i--)
      {
        if(connections_.at(i)->getCost() == std::numeric_limits<double>::infinity())
          pos_closest_obs_from_goal = connections_.size()-1-i;
      }
    }
  }
  else if(conf == conn->getChild()->getConfiguration())
  {
    if(conn_idx<connections_.size()-1)
    {
      validity = isValidFromConn(connections_.at(conn_idx+1),checker);

      if(not validity)
      {
        for(size_t i = (connections_.size()-1);i>=conn_idx+1;i--)
        {
          if(connections_.at(i)->getCost() == std::numeric_limits<double>::infinity())
            pos_closest_obs_from_goal = connections_.size()-1-i;
        }
      }
    }
    else
    {
      CNR_INFO(logger_,"Conf is equal to goal, no connection to validate from here. Conf: "<<conf.transpose()<<" goal: "<<goal_node_->getConfiguration().transpose());
      validity = true;
      assert(0);
    }
  }
  else
  {
    if(not checker->checkConnFromConf(conn,conf))
    {
      validity = false;
      conn->setCost(std::numeric_limits<double>::infinity());
      pos_closest_obs_from_goal = connections_.size()-1-conn_idx;
    }
    else
    {
      conn->setCost(metrics_->cost(conn->getParent()->getConfiguration(),conn->getChild()->getConfiguration()));
    }

    if(conn_idx<connections_.size()-1)  //even if the checker has failed, this check is important to update the cost of all the connections
    {
      if(not isValidFromConn(connections_.at(conn_idx+1),checker))
      {
        validity = false;
        for(size_t i = (connections_.size()-1);i>=conn_idx+1;i--)
        {
          if(connections_.at(i)->getCost() == std::numeric_limits<double>::infinity())
            pos_closest_obs_from_goal = connections_.size()-1-i;
        }
      }
    }
  }

  return validity;
}

bool Path::isValidFromConf(const Eigen::VectorXd &conf, int &pos_closest_obs_from_goal, const CollisionCheckerPtr &this_checker)
{
  size_t conn_idx;
  findConnection(conf,conn_idx);
  return isValidFromConf(conf,conn_idx,pos_closest_obs_from_goal,this_checker);
}

void Path::toYAML(const std::string& file_name, const bool reverse) const
{
  std::ofstream out(file_name);
  if(out.is_open())
  {
    out << toYAML(reverse);
    out.close();
  }
  else
  {
    // Handle error opening the file
    CNR_ERROR(logger_, "Error opening file: " << file_name);
  }
}

YAML::Node Path::toYAML(const bool reverse) const
{
  YAML::Node yaml;
  if (connections_.empty())
    return yaml;

  yaml.push_back(reverse ? goal_node_->toYAML() : start_node_->toYAML());

  for (size_t idx = 0; idx < connections_.size(); idx++)
  {
    yaml.push_back(reverse ? connections_.at(connections_.size() - idx - 1)->getParent()->toYAML()
                           : connections_.at(idx)->getChild()->toYAML());
  }

  return yaml;
}

PathPtr Path::fromYAML(const YAML::Node& yaml, const MetricsPtr& metrics,
                       const CollisionCheckerPtr& checker, const cnr_logger::TraceLoggerPtr& logger)
{
  if (!yaml.IsSequence())
  {
    CNR_ERROR(logger, "Cannot load a path from YAML::Node which does not contain a sequence");
    return nullptr;
  }

  std::vector<NodePtr> nodes;

  for (const YAML::Node& node_yaml : yaml)
  {
    NodePtr node = Node::fromYAML(node_yaml, logger);
    if(!node)
    {
      // Error creating a Node from YAML
      CNR_ERROR(logger, "Error creating a Node from YAML");
      return nullptr;
    }

    nodes.push_back(node);
  }

  CNR_WARN(logger,"Path created from yaml but no tree is available, remember to add it!");

  return std::make_shared<Path>(nodes,metrics,checker,logger);
}

void Path::flip()
{
  std::reverse(connections_.begin(),connections_.end()); //must be before connections flip

  for (ConnectionPtr conn: connections_)
    conn->flip();

  start_node_ = connections_.front()->getParent();
  goal_node_  = connections_.back ()->getChild ();
}

bool Path::onLine(double toll)
{
  if(connections_.empty())
    throw std::invalid_argument("connections vector is empty!");

  if(connections_.size() == 1)
    return true;

  for(unsigned int i=0;i<connections_.size()-1;i++)
  {
    if(not connections_[i]->isParallel(connections_[i+1],toll))
      return false;
  }

  return true;
}

std::ostream& operator<<(std::ostream& os, const Path& path)
{
  os << "cost = " << path.cost_ << std::endl;
  if (path.connections_.size() == 0)
    os << "no waypoints";

  for(const ConnectionPtr& conn:path.connections_)
    os << *conn << std::endl;

  return os;
}

} //end namespace core
} // end namespace graph
