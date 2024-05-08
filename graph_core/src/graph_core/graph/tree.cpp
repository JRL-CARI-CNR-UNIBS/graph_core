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

#include <graph_core/graph/tree.h>

namespace graph
{
namespace core
{
Tree::Tree(const NodePtr& root,
           const double &max_distance,
           const CollisionCheckerPtr &checker,
           const MetricsPtr &metrics,
           const cnr_logger::TraceLoggerPtr& logger,
           const bool &use_kdtree):
  root_(root),
  use_kdtree_(use_kdtree),
  max_distance_(max_distance),
  metrics_(metrics),
  logger_(logger),
  checker_(checker)
{
  if (use_kdtree)
  {
    nodes_=std::make_shared<KdTree>(logger_);
  }
  else
  {
    nodes_=std::make_shared<Vector>(logger_);
  }
  nodes_->insert(root);
  double dimension=root->getConfiguration().size();
  k_rrt_=1.1*std::pow(2.0,dimension+1)*std::exp(1)*(1.0+1.0/dimension);
}

NodePtr Tree::findClosestNode(const Eigen::VectorXd &configuration)
{
  return nodes_->nearestNeighbor(configuration);
}

bool Tree::tryExtend(const Eigen::VectorXd &configuration,
                     Eigen::VectorXd &next_configuration,
                     NodePtr &closest_node)
{
  closest_node = findClosestNode(configuration);
  assert(closest_node);

  return tryExtendFromNode(configuration,next_configuration,closest_node);
}

bool Tree::tryExtendFromNode(const Eigen::VectorXd &configuration,
                             Eigen::VectorXd &next_configuration,
                             NodePtr& node)
{
  assert(node);
  double distance = selectNextConfiguration(configuration,next_configuration,node);

  if (distance < TOLERANCE)
    return true;
  else
    if (checker_->checkConnection(node->getConfiguration(), next_configuration))
      return true;

  return false;
}

bool Tree::tryExtendWithPathCheck(const Eigen::VectorXd &configuration,
                                  Eigen::VectorXd &next_configuration,
                                  NodePtr &closest_node,
                                  std::vector<ConnectionPtr> &checked_connections)
{
  closest_node = findClosestNode(configuration);
  assert(closest_node);

  return tryExtendFromNodeWithPathCheck(configuration,next_configuration,closest_node,checked_connections);
}

bool Tree::tryExtendFromNodeWithPathCheck(const Eigen::VectorXd &configuration,
                                          Eigen::VectorXd &next_configuration,
                                          NodePtr& node,
                                          std::vector<ConnectionPtr> &checked_connections)
{
  assert(node);

  if(checkPathToNode(node,checked_connections))
  {
    double distance = selectNextConfiguration(configuration,next_configuration,node);

    if (distance < TOLERANCE)
      return true;
    else
      if (checker_->checkConnection(node->getConfiguration(), next_configuration))
        return true;
  }

  return false;
}

double Tree::selectNextConfiguration(const Eigen::VectorXd& configuration,
                                     Eigen::VectorXd& next_configuration,
                                     const NodePtr& node)
{
  assert(node);

  double distance = (node->getConfiguration() - configuration).norm();

  if (distance < TOLERANCE)
  {
    next_configuration = configuration;
  }
  else if (distance < max_distance_)
  {
    next_configuration = configuration;
  }
  else
  {
    next_configuration = node->getConfiguration() + (configuration - node->getConfiguration()) / distance * max_distance_;
  }

  return distance;
}

bool Tree::extendOnly(NodePtr& closest_node, NodePtr &new_node, ConnectionPtr &connection)
{
  double cost = metrics_->cost(closest_node, new_node);
  connection = std::make_shared<Connection>(closest_node, new_node,logger_);
  connection->add();
  connection->setCost(cost);

  addNode(new_node,false);

  return true;
}

bool Tree::extend(const Eigen::VectorXd &configuration, NodePtr &new_node)
{
  ConnectionPtr conn;
  return extend(configuration,new_node,conn);
}

bool Tree::extend(const Eigen::VectorXd &configuration, NodePtr &new_node, ConnectionPtr &connection)
{
  NodePtr closest_node;
  Eigen::VectorXd next_configuration;
  if (!tryExtend(configuration,
                 next_configuration,
                 closest_node))
  {
    new_node = nullptr;
    connection = nullptr;
    return false;
  }

  new_node = std::make_shared<Node>(next_configuration,logger_);
  return extendOnly(closest_node,new_node,connection);
}

bool Tree::extendWithPathCheck(const Eigen::VectorXd &configuration, NodePtr &new_node, std::vector<ConnectionPtr> &checked_connections)
{
  ConnectionPtr conn;
  return extendWithPathCheck(configuration,new_node,conn,checked_connections);
}

bool Tree::extendWithPathCheck(const Eigen::VectorXd &configuration, NodePtr &new_node, ConnectionPtr &connection, std::vector<ConnectionPtr> &checked_connections)
{
  NodePtr closest_node;
  Eigen::VectorXd next_configuration;
  if (not tryExtendWithPathCheck(configuration,
                                 next_configuration,
                                 closest_node,
                                 checked_connections))
  {
    new_node = nullptr;
    connection = nullptr;
    return false;
  }

  new_node = std::make_shared<Node>(next_configuration, logger_);
  if(not extendOnly(closest_node,new_node,connection))
    return false;

  connection->setRecentlyChecked(true);
  checked_connections.push_back(connection);

  return true;
}

bool Tree::extendToNode(const NodePtr& node,
                        NodePtr& new_node)
{
  if (isInTree(node))
    return true;

  NodePtr closest_node;
  Eigen::VectorXd next_configuration;
  if (!tryExtend(node->getConfiguration(),
                 next_configuration,
                 closest_node))
  {
    return false;
  }

  if ((next_configuration - node->getConfiguration()).norm() < TOLERANCE)
  {
    new_node = node;
    addNode(node);
  }
  else
  {
    new_node = std::make_shared<Node>(next_configuration,logger_);
    addNode(new_node,false);
  }

  double cost = metrics_->cost(closest_node, new_node);
  ConnectionPtr conn = std::make_shared<Connection>(closest_node, new_node,logger_);
  conn->add();
  conn->setCost(cost);

  return true;
}

bool Tree::connect(const Eigen::VectorXd &configuration, NodePtr &new_node)
{
  bool success = true;
  NodePtr tmp_node;
  while (success)
  {
    tmp_node = nullptr;
    success = extend(configuration, tmp_node);
    if(success)
    {
      new_node = tmp_node;
      if ((new_node->getConfiguration() - configuration).norm() < TOLERANCE)
        return true;
    }
  }
  return false;
}

bool Tree::informedExtend(const Eigen::VectorXd &configuration, NodePtr &new_node,const Eigen::VectorXd &goal, const double& cost2beat, const double& bias)
{
  struct extension
  {
    NodePtr tree_node;
    Eigen::VectorXd new_conf;
    double distance;
  };

  std::multimap<double,NodePtr> closest_nodes_map = nearK(configuration);
  if(closest_nodes_map.size()==0)
  {
    CNR_FATAL(logger_,"closest nodes map is empty");
    throw std::runtime_error("closest nodes map is empty");
  }

  double heuristic, distance, cost2node;
  Eigen::VectorXd new_configuration;
  std::multimap<double,extension> best_nodes_map;

  for(const std::pair<const double,NodePtr>& n: closest_nodes_map)
  {
    distance = selectNextConfiguration(configuration,new_configuration,n.second);
    cost2node = costToNode(n.second);
    heuristic = bias*distance+(1-bias)*cost2node;

    extension ext;
    ext.tree_node = n.second;
    ext.new_conf = new_configuration;
    ext.distance = distance;

    if((cost2node+distance+(goal-new_configuration).norm())<cost2beat)  //if and only if the new conf underestimation of cost is less than the cost to beat it is added to the tree
      best_nodes_map.insert(std::pair<double,extension>(heuristic,ext));  //give priority to parents with the best heuristics
  }

  bool extend_ok = false;
  extension ext;
  for(const std::pair<const double,extension>& n: best_nodes_map)
  {
    ext = n.second;
    if(ext.distance < TOLERANCE)
    {
      extend_ok = true;
      break;
    }
    else
    {
      if (checker_->checkConnection(ext.tree_node->getConfiguration(), ext.new_conf))
      {
        extend_ok = true;
        break;
      }
    }
  }

  if(extend_ok)
  {
    ConnectionPtr connection;
    new_node = std::make_shared<Node>(ext.new_conf,logger_);
    return extendOnly(ext.tree_node,new_node,connection);
  }
  else
    return false;
}

bool Tree::connectToNode(const NodePtr &node, NodePtr &new_node, const double &max_time)
{
  if(max_time<=0.0)
    return false;
  std::chrono::time_point<std::chrono::system_clock> tic = std::chrono::system_clock::now();

  bool success = true;
  while (success)
  {
    NodePtr tmp_node;
    success = extendToNode(node, tmp_node);
    if(success)
    {
      new_node = tmp_node;

      if ((new_node->getConfiguration() - node->getConfiguration()).norm() < TOLERANCE)
        return true;
    }
    std::chrono::time_point<std::chrono::system_clock> now = std::chrono::system_clock::now();
    std::chrono::duration<double> difference = now - tic;
    if(difference.count() >= 0.98*max_time) break;
  }
  return false;
}

bool Tree::checkPathToNode(const NodePtr& node, std::vector<ConnectionPtr>& checked_connections)
{
  std::vector<ConnectionPtr> path_connections;
  return checkPathToNode(node,checked_connections,path_connections);
}

bool Tree::checkPathToNode(const NodePtr& node, std::vector<ConnectionPtr>& checked_connections, std::vector<ConnectionPtr>& path_connections)
{
  std::vector<ConnectionPtr> connections_to_check;

  path_connections.clear();
  path_connections = getConnectionToNode(node);

  for(const ConnectionPtr& conn: path_connections)
  {
    if(conn->isRecentlyChecked())
    {
      if(conn->getCost() == std::numeric_limits<double>::infinity())
        return false;
    }
    else
      connections_to_check.push_back(conn);
  }

  for(const ConnectionPtr& conn: connections_to_check)
  {
    if(not checker_->checkConnection(conn))
    {
      conn->setCost(std::numeric_limits<double>::infinity());
      conn->setRecentlyChecked(true);
      checked_connections.push_back(conn);

      return false;
    }
    else
    {
      conn->setCost(metrics_->cost(conn->getParent(),conn->getChild()));
      conn->setRecentlyChecked(true);
      checked_connections.push_back(conn);
    }
  }

  return true;
}

bool Tree::rewireOnly(NodePtr& node, double r_rewire, const int& what_rewire)
{
  std::vector<NodePtr> white_list;
  return rewireOnly(node,r_rewire,white_list,what_rewire);
}

bool Tree::rewireOnly(NodePtr& node, double r_rewire, const std::vector<NodePtr>& white_list, const int& what_rewire)
{
  if(what_rewire >2 || what_rewire <0)
  {
    CNR_ERROR(logger_,"what_rewire parameter should be 0,1 or 2");
    return false;
  }

  bool rewire_parent;
  bool rewire_children;

  switch(what_rewire)
  {
  case 0:
    rewire_parent   = true ;
    rewire_children = true ;
    break;
  case 1:
    rewire_parent   = true ;
    rewire_children = false;
    break;
  case 2:
    rewire_parent   = false;
    rewire_children = true ;
    break;
  }

  if(node == root_)
    rewire_parent = false;

  std::vector<NodePtr>::const_iterator it;
  if(rewire_parent)
  {
    it = std::find(white_list.begin(),white_list.end(),node);
    if(it<white_list.end())
      rewire_parent = false;
  }

  std::multimap<double,NodePtr> near_nodes;
  if(r_rewire<=0)
    near_nodes = nearK(node);
  else
    near_nodes = near(node, r_rewire);

  double cost_to_node = costToNode(node);
  bool improved = false;

  if(rewire_parent)
  {
    NodePtr nearest_node = node->getParents()[0];
    for(const std::pair<const double,NodePtr>& p : near_nodes)
    {
      const NodePtr& n = p.second;

      if (n == nearest_node)
        continue;
      if (n == node)
        continue;

      double cost_to_near = costToNode(n);

      if (cost_to_near >= cost_to_node)
        continue;

      double cost_near_to_node = metrics_->cost(n, node);

      if ((cost_to_near + cost_near_to_node) >= cost_to_node)
        continue;

      if (!checker_->checkConnection(n->getConfiguration(), node->getConfiguration()))
        continue;

      assert(node->parentConnection(0)->isValid());
      node->parentConnection(0)->remove();

      ConnectionPtr conn = std::make_shared<Connection>(n, node,logger_);
      conn->setCost(cost_near_to_node);
      conn->add();

      cost_to_node = cost_to_near + cost_near_to_node;
      improved = true;
    }
  }

  NodePtr parent = node->getParents()[0];
  if(rewire_children)
  {
    for (const std::pair<const double,NodePtr>& p : near_nodes)
    {
      const NodePtr& n = p.second;

      if(n == parent)
        continue;
      if(n == node)
        continue;
      if(n == root_)
        continue;

      it = std::find(white_list.begin(),white_list.end(),n); //if the near node is a white node its parent should not be changed
      if(it<white_list.end())
        continue;

      double cost_to_near = costToNode(n);
      if (cost_to_node >= cost_to_near)
        continue;

      double cost_node_to_near = metrics_->cost(node->getConfiguration(), n->getConfiguration());
      if ((cost_to_node + cost_node_to_near) >= cost_to_near)
        continue;

      if (!checker_->checkConnection(node->getConfiguration(), n->getConfiguration()))
        continue;

      assert(n->parentConnection(0)->isValid());
      n->parentConnection(0)->remove();

      ConnectionPtr conn = std::make_shared<Connection>(node, n,logger_);
      conn->setCost(cost_node_to_near);
      conn->add();

      improved = true;
    }
  }

  return improved;
}

bool Tree::rewireOnlyWithPathCheck(NodePtr& node, std::vector<ConnectionPtr> &checked_connections, double r_rewire, const int& what_rewire)
{
  std::vector<NodePtr> white_list;
  return rewireOnlyWithPathCheck(node,checked_connections,r_rewire,white_list,what_rewire);
}

bool Tree::rewireOnlyWithPathCheck(NodePtr& node, std::vector<ConnectionPtr>& checked_connections, double r_rewire, const std::vector<NodePtr>& white_list, const int& what_rewire)
{
  if(what_rewire >2 || what_rewire <0)
  {
    CNR_ERROR(logger_,"what_rewire parameter should be 0,1 or 2");
    assert(0);
    return false;
  }

  bool rewire_parent;
  bool rewire_children;

  switch(what_rewire)
  {
  case 0:
    rewire_parent   = true ;
    rewire_children = true ;
    break;
  case 1:
    rewire_parent   = true ;
    rewire_children = false;
    break;
  case 2:
    rewire_parent   = false;
    rewire_children = true ;
    break;
  }

  if(node == root_)
    rewire_parent = false;

  std::vector<NodePtr>::const_iterator it;
  if(rewire_parent)
  {
    it = std::find(white_list.begin(),white_list.end(),node);
    if(it<white_list.end())
      rewire_parent = false;
  }

  std::multimap<double,NodePtr> near_nodes;
  if(r_rewire<=0)
    near_nodes = nearK(node);
  else
    near_nodes = near(node, r_rewire);

  //validate connections to node
  double cost_to_node;
  (checkPathToNode(node,checked_connections))?
        (cost_to_node = costToNode(node)):
        (cost_to_node = std::numeric_limits<double>::infinity());

  bool improved = false;

  if(rewire_parent)
  {
    NodePtr nearest_node = node->getParents()[0];
    for (const std::pair<const double,NodePtr>& p : near_nodes)
    {
      const NodePtr& n = p.second;

      if (n == nearest_node)
        continue;
      if (n == node)
        continue;

      double cost_to_near = costToNode(n);

      if (cost_to_near >= cost_to_node)
        continue;

      double cost_near_to_node = metrics_->cost(n, node);
      if ((cost_to_near + cost_near_to_node) >= cost_to_node)
        continue;

      if (not checker_->checkConnection(n->getConfiguration(), node->getConfiguration()))
        continue;

      if(not checkPathToNode(n,checked_connections)) //validate connections to n
        continue;

      node->parentConnection(0)->remove();

      ConnectionPtr conn = std::make_shared<Connection>(n, node,logger_);
      conn->setCost(cost_near_to_node);
      conn->add();

      conn->setRecentlyChecked(true);
      checked_connections.push_back(conn);

      cost_to_node = cost_to_near + cost_near_to_node;
      improved = true;
    }
  }

  if(cost_to_node == std::numeric_limits<double>::infinity())
    rewire_children = false;

  NodePtr parent = node->getParents()[0];

  if(rewire_children)
  {
    for (const std::pair<const double,NodePtr>& p : near_nodes)
    {
      const NodePtr& n = p.second;

      if(n == parent)
        continue;
      if(n == node)
        continue;
      if(n == root_)
        continue;

      it = std::find(white_list.begin(),white_list.end(),n); //if the near node is a white node its parent should not be changed
      if(it<white_list.end())
        continue;

      double cost_to_near = costToNode(n);
      bool path_to_near_checked = false;

      if(cost_to_node >= cost_to_near)
      {
        if(checkPathToNode(n,checked_connections)) //if path to n is free, cost_to_node >= cost_to_near is really true
          continue;

        path_to_near_checked = true;
      }

      double cost_node_to_near = metrics_->cost(node, n);
      if ((cost_to_node + cost_node_to_near) >= cost_to_near)
      {
        if(path_to_near_checked)
          continue;
        else
        {
          if(checkPathToNode(n,checked_connections)) //if path to n is free, cost_to_node +cost_node_to_near >= cost_to_near is really true
            continue;
        }
      }

      if(not checker_->checkConnection(node->getConfiguration(), n->getConfiguration()))
        continue;

      n->parentConnection(0)->remove();

      ConnectionPtr conn = std::make_shared<Connection>(node, n,logger_);
      conn->setCost(cost_node_to_near);
      conn->add();

      conn->setRecentlyChecked(true);
      checked_connections.push_back(conn);

      improved = true;
    }
  }

  return improved;
}

bool Tree::rewireWithPathCheck(const Eigen::VectorXd &configuration, std::vector<ConnectionPtr> &checked_connections, double r_rewire, const std::vector<NodePtr> &white_list, NodePtr& new_node)
{
  ConnectionPtr new_conn;
  if (not extendWithPathCheck(configuration,new_node,new_conn,checked_connections))
  {
    return false;
  }

  return rewireOnlyWithPathCheck(new_node,checked_connections,r_rewire,white_list);
}

bool Tree::rewireWithPathCheck(const Eigen::VectorXd &configuration, std::vector<ConnectionPtr> &checked_connections, double r_rewire, NodePtr& new_node)
{
  std::vector<NodePtr> white_list;
  return rewireWithPathCheck(configuration,checked_connections,r_rewire,white_list,new_node);
}

bool Tree::rewireWithPathCheck(const Eigen::VectorXd &configuration, std::vector<ConnectionPtr> &checked_connections, double r_rewire)
{
  NodePtr new_node;
  std::vector<NodePtr> white_list;
  return rewireWithPathCheck(configuration,checked_connections,r_rewire,white_list,new_node);
}

bool Tree::rewire(const Eigen::VectorXd &configuration, double r_rewire, NodePtr& new_node)
{
  if (!extend(configuration, new_node))
  {
    return false;
  }

  return rewireOnly(new_node,r_rewire);
}

bool Tree::rewire(const Eigen::VectorXd &configuration, double r_rewire)
{
  NodePtr new_node;
  return rewire(configuration,r_rewire,new_node);
}


bool Tree::rewireToNode(const NodePtr& n, double r_rewire, NodePtr& new_node)
{
  if (!extendToNode(n, new_node))
  {
    return false;
  }

  return rewireOnly(new_node,r_rewire);
}

bool Tree::rewireToNode(const NodePtr& n, double r_rewire)
{
  NodePtr new_node;

  return rewireToNode(n,r_rewire,new_node);
}


std::multimap<double,NodePtr> Tree::near(const NodePtr &node, const double &radius)
{
  return nodes_->near(node->getConfiguration(),radius);
}

std::multimap<double,NodePtr> Tree::nearK(const NodePtr &node)
{
  return nearK(node->getConfiguration());
}

std::multimap<double,NodePtr> Tree::nearK(const Eigen::VectorXd &conf)
{
  size_t k=std::ceil(k_rrt_*std::log(nodes_->size()+1));
  return nodes_->kNearestNeighbors(conf,k);
}

double Tree::costToNode(NodePtr node)
{
  double cost = 0;
  while (node != root_)
  {
    if(node->getParentConnectionsSize() != 1)
    {
      CNR_ERROR(logger_,"a tree node should have exactly a parent!\n "<<*node);
      return std::numeric_limits<double>::infinity();
    }

    if(node->parentConnection(0)->getParent() == node)
    {
      CNR_FATAL(logger_,"node "<< node.get() <<"=\n" << *node);
      CNR_FATAL(logger_,"to parent\n" << * (node->parentConnection(0)));
      CNR_FATAL(logger_,"connection between the same node!");
      throw std::runtime_error("connection between the same node!");
    }
    cost += node->parentConnection(0)->getCost();
    node = node->parentConnection(0)->getParent();
  }
  return cost;
}

std::vector<ConnectionPtr> Tree::getConnectionToNode(const NodePtr &node)
{
  std::vector<ConnectionPtr> connections;
  NodePtr tmp_node = node;

  while (tmp_node != root_)
  {
    if(node->getParentConnectionsSize() != 1)
    {
      CNR_ERROR(logger_,"a tree node should have only a parent");
      CNR_ERROR(logger_,"node \n" << *node);
      CNR_INFO (logger_,"current root "<<root_);
      CNR_INFO (logger_,"node "<<node);

      throw std::runtime_error("a tree node should have only a parent");
    }
    connections.push_back(tmp_node->parentConnection(0));
    tmp_node = tmp_node->parentConnection(0)->getParent();
  }
  std::reverse(connections.begin(), connections.end());

  return connections;
}

void Tree::addNode(const NodePtr& node, const bool& check_if_present)
{
  if (!check_if_present || !isInTree(node))
    nodes_->insert(node);
}

void Tree::removeNode(const NodePtr& node)
{
  node->disconnect();
  nodes_->deleteNode(node);
}

bool Tree::keepOnlyThisBranch(const std::vector<ConnectionPtr>& connections)
{
  if (connections.size() == 0)
    return false;

  std::vector<NodePtr> branch_nodes;
  branch_nodes.push_back(connections.at(0)->getParent());
  for (const ConnectionPtr& conn : connections)
  {
    if(conn->getParent() != branch_nodes.back())
    {
      CNR_FATAL(logger_,"the branch should consist of contiguous connections");
      throw std::runtime_error("the branch should consist of contiguous connections");
    }

    branch_nodes.push_back(conn->getChild());
  }

  nodes_->disconnectNodes(branch_nodes);
  nodes_->clear();
  for (const NodePtr& n : branch_nodes)
    nodes_->insert(n);

  assert(nodes_->size() == branch_nodes.size());

  return true;
}

bool Tree::addBranch(const std::vector<ConnectionPtr> &connections)
{
  if (connections.size() == 0)
    return false;

  NodePtr start_node = connections.at(0)->getParent();
  if (!isInTree(start_node))
  {
    CNR_ERROR(logger_,"start node of the branch is not part of the tree");
    return false;
  }

  std::vector<NodePtr> branch_nodes;
  for (const ConnectionPtr& conn : connections)
    branch_nodes.push_back(conn->getChild());

  for (NodePtr& n : branch_nodes)
  {
    if (not nodes_->findNode(n))
      nodes_->insert(n);
  }
  return true;
}

bool Tree::addTree(TreePtr &additional_tree, const double &max_time)
{
  if (not isInTree(additional_tree->getRoot()))
  {
    NodePtr new_node;
    if (not connectToNode(additional_tree->getRoot(),new_node,max_time))
      return false;
  }

  std::vector<NodePtr> additional_nodes=additional_tree->getNodes();
  for (size_t inode=1;inode<additional_nodes.size();inode++)
  {
    addNode(additional_nodes.at(inode),false);
  }
  return true;
}


bool Tree::isInTree(const NodePtr &node)
{
  return nodes_->findNode(node);
}


unsigned int Tree::purgeNodesOutsideEllipsoid(const SamplerPtr& sampler, const std::vector<NodePtr>& white_list)
{
  if (nodes_->size() < maximum_nodes_)
    return 0;
  unsigned int removed_nodes = 0;

  purgeNodeOutsideEllipsoid(root_,sampler,white_list,removed_nodes);
  return removed_nodes;
}

unsigned int Tree::purgeNodesOutsideEllipsoids(const std::vector<SamplerPtr>& samplers, const std::vector<NodePtr>& white_list)
{
  if (nodes_->size() < maximum_nodes_)
    return 0;
  unsigned int removed_nodes = 0;

  purgeNodeOutsideEllipsoids(root_,samplers,white_list,removed_nodes);
  return removed_nodes;
}

void Tree::purgeNodeOutsideEllipsoid(NodePtr& node,
                                     const SamplerPtr& sampler,
                                     const std::vector<NodePtr>& white_list,
                                     unsigned int& removed_nodes)
{
  assert(node);

  // check if it is in the admissible informed set
  if(sampler->inBounds(node->getConfiguration()))
  {
    // if node is inside the admissible set, check its successors
    std::vector<NodePtr> successors;
    successors = node->getChildren();

    for(NodePtr& n : successors)
    {
      assert(n.get()!=node.get());
      purgeNodeOutsideEllipsoid(n,sampler,white_list,removed_nodes);
    }
  }
  else
  {
    // if node is outside the admissible set, remove it and its successors if they are not in the white list.
    if (std::find(white_list.begin(), white_list.end(), node) != white_list.end())
      return;
    purgeFromHere(node, white_list, removed_nodes);
  }
  return;
}

void Tree::purgeNodeOutsideEllipsoids(NodePtr& node,
                                      const std::vector<SamplerPtr>& samplers,
                                      const std::vector<NodePtr>& white_list,
                                      unsigned int& removed_nodes)
{
  if (nodes_->size() < 0.5*maximum_nodes_)
    return;
  assert(node);

  // check if it belongs to a admissible informed set or the white list
  bool inbound=std::find(white_list.begin(), white_list.end(), node) != white_list.end();
  for (const SamplerPtr& sampler: samplers)
    inbound = inbound || sampler->inBounds(node->getConfiguration());

  if (inbound)
  {
    // if node is inside the admissible set, check its successors
    std::vector<NodePtr> successors;
    successors = node->getChildren();

    for (NodePtr& n : successors)
    {
      assert(n.get()!=node.get());
      purgeNodeOutsideEllipsoids(n,samplers,white_list,removed_nodes);
    }
  }
  else
  {
    purgeFromHere(node, white_list, removed_nodes);
  }
  return;
}

unsigned int Tree::purgeNodes(const SamplerPtr& sampler, const std::vector<NodePtr>& white_list, const bool check_bounds)
{
  if (nodes_->size() < maximum_nodes_)
    return 0;

  unsigned int nodes_to_remove = nodes_->size() - maximum_nodes_;
  unsigned int removed_nodes = 0;
  unsigned int idx = 0;
  std::vector<NodePtr> nodes = nodes_->getNodes();
  while (idx < nodes_->size())
  {
    if (std::find(white_list.begin(), white_list.end(), nodes.at(idx)) != white_list.end())
    {
      idx++;
      continue;
    }
    if(check_bounds && !sampler->inBounds(nodes.at(idx)->getConfiguration()))
    {
      purgeFromHere(nodes.at(idx), white_list, removed_nodes);
      continue;
    }

    if (nodes_to_remove <= removed_nodes)
      break;
    if(nodes.at(idx)->getChildConnectionsSize() == 0)
    {
      removed_nodes++;
      nodes.at(idx)->disconnect();
      removeNode(nodes.at(idx));
      continue;
    }

    idx++;
  }
  return removed_nodes;
}

bool Tree::purgeFromHere(NodePtr& node)
{
  std::vector<NodePtr> white_list;
  unsigned int removed_nodes;

  return purgeFromHere(node,white_list,removed_nodes);
}

bool Tree::purgeFromHere(NodePtr& node, const std::vector<NodePtr>& white_list, unsigned int& removed_nodes)
{
  if (std::find(white_list.begin(), white_list.end(), node) != white_list.end())
  {
    CNR_INFO(logger_,"Node in white list: "<<*node);
    return false;
  }
  assert(node);
  std::vector<NodePtr> successors;

  successors = node->getChildren();

  bool disconnect = true;
  for (NodePtr& n : successors)
  {
    assert(n.get()!=node.get());
    if (!purgeFromHere(n,white_list,removed_nodes))
      disconnect = false;
  }

  if(disconnect)
    purgeThisNode(node,removed_nodes);

  return disconnect;
}

void Tree::purgeThisNode(NodePtr& node, unsigned int& removed_nodes)
{
  assert(node);
  node->disconnect();
  if (nodes_->deleteNode(node))
  {
    removed_nodes++;
  }
}

void Tree::cleanTree()
{
  std::vector<NodePtr> successors;
  std::vector<NodePtr> white_list;
  unsigned int removed_nodes;
  successors=root_->getChildren();
  for (NodePtr& n: successors)
  {
    if (isInTree(n))
      purgeFromHere(n,white_list,removed_nodes);
  }
}

void Tree::populateTreeFromNode(const NodePtr& node, const bool node_check)
{
  Eigen::VectorXd focus1 = node->getConfiguration();
  Eigen::VectorXd focus2 = node->getConfiguration();
  populateTreeFromNode(node, focus1, focus2, std::numeric_limits<double>::infinity(), node_check);
}

void Tree::populateTreeFromNode(const NodePtr& node, const std::vector<NodePtr>& black_list, const bool node_check)
{
  Eigen::VectorXd focus1 = node->getConfiguration();
  Eigen::VectorXd focus2 = node->getConfiguration();
  populateTreeFromNode(node, focus1, focus2, std::numeric_limits<double>::infinity(), black_list, node_check);
}

void Tree::populateTreeFromNode(const NodePtr& node, const Eigen::VectorXd& focus1, const Eigen::VectorXd& focus2, const double& cost, const bool node_check)
{
  std::vector<NodePtr> black_list;
  populateTreeFromNode(node, focus1, focus2, cost, black_list, node_check);
}

void Tree::populateTreeFromNode(const NodePtr& node, const Eigen::VectorXd& focus1, const Eigen::VectorXd& focus2, const double& cost, const std::vector<NodePtr> &black_list, const bool node_check)
{
  if (not nodes_->findNode(node))
  {
    CNR_FATAL(logger_,"node is not member of tree");
    throw std::invalid_argument("node is not member of tree");
  }

  for (const NodePtr& n: node->getChildren())
  {
    std::vector<NodePtr>::const_iterator it = std::find(black_list.begin(), black_list.end(), n);
    if(it != black_list.end())
    {
      continue;
    }
    else
    {
      if((metrics_->utopia(n->getConfiguration(),focus1) + metrics_->utopia(n->getConfiguration(),focus2)) < cost) //CHIEDI A MANUEL SE UTOPIA O NORMA
      {
        if(node_check)
        {
          if(!checker_->check(n->getConfiguration()))
          {
            //set the cost of the connetion to infinity?? CHIEDI A MANUEL
            continue;
          }
        }
        nodes_->insert(n);
        populateTreeFromNode(n,focus1,focus2,cost,black_list);
      }
    }
  }
}

void Tree::populateTreeFromNodeConsideringCost(const NodePtr& node, const Eigen::VectorXd& goal, const double& cost, const std::vector<NodePtr> &black_list, const bool node_check)
{
  if (not nodes_->findNode(node))
  {
    CNR_INFO(logger_,"node is not member of tree");
    throw std::invalid_argument("node is not member of tree");
  }

  double cost_to_node = 0;
  for(const ConnectionPtr& conn:getConnectionToNode(node))
    cost_to_node += conn->getCost();

  NodePtr child;
  ConnectionPtr conn;
  double cost_to_child;
  for(size_t i=0; i<node->getChildConnectionsSize();i++)
  {
    conn = node->childConnection(i);
    child = conn->getChild();

    std::vector<NodePtr>::const_iterator it = std::find(black_list.begin(), black_list.end(), child);
    if(it != black_list.end())
    {
      continue;
    }
    else
    {
      cost_to_child = cost_to_node+conn->getCost();

      if((cost_to_child + metrics_->utopia(child->getConfiguration(),goal)) < cost)
      {
        if(node_check)
        {
          if(not checker_->check(child->getConfiguration()))
            continue;
        }
        nodes_->insert(child);
        populateTreeFromNodeConsideringCost(child,goal,cost,black_list,node_check);
      }
    }
  }
}

void Tree::getLeaves(std::vector<NodePtr>& leaves)
{
  std::vector<NodePtr> nodes = getNodes();
  std::for_each(nodes.begin(),nodes.end(),[&](NodePtr n){
    assert(((n->getParentConnectionsSize() == 1) && (n!=root_)) || (n == root_));

    if((n!= root_) && (n->getChildConnectionsSize() == 0))
      leaves.push_back(n);
  });
}

YAML::Node Tree::toYAML() const
{
  YAML::Node tree;
  YAML::Node nodes;
  YAML::Node connections;

  std::vector<NodePtr> nodes_vector = nodes_->getNodes();
  for (std::size_t inode = 0; inode < nodes_vector.size(); ++inode)
  {
    const NodePtr& n = nodes_vector.at(inode);
    nodes.push_back(n->toYAML());

    std::vector<NodePtr> children = n->getChildren();

    for (std::size_t ichild = 0; ichild < children.size(); ++ichild)
    {
      for (std::size_t inodes = 0; inodes < nodes_vector.size(); ++inodes)
      {
        if (nodes_vector.at(inodes) == children.at(ichild))
        {
          YAML::Node connection;
          connection.SetStyle(YAML::EmitterStyle::Flow); // Set the style to flow style for a more compact representation

          connection.push_back(static_cast<int>(inode));
          connection.push_back(static_cast<int>(inodes));
          connections.push_back(connection);
          break;
        }
      }
    }
  }

  tree["max_distance"] = max_distance_;
  tree["use_kdtree"] = use_kdtree_;
  tree["nodes"] = nodes;
  tree["connections"] = connections;
  return tree;
}


void Tree::toYAML(const std::string& file_name) const
{
  std::ofstream out(file_name);
  if(out.is_open())
  {
    out << toYAML();
    out.close();
  }
  else
  {
    // Handle error opening the file
    CNR_ERROR(logger_, "Error opening file: " << file_name);
  }
}

std::ostream& operator<<(std::ostream& os, const Tree& tree)
{
  os << "number of nodes = " << tree.nodes_->size() << std::endl;
  os << "root = " << *tree.root_;

  if(tree.print_full_tree_)
  {
    os << "\nConnections list:";

    std::function<void(const NodePtr&)> fcn;
    fcn = [&](const NodePtr& n) ->void{
      for(const ConnectionPtr& c:n->getChildConnections())
      {
        os<<"\n"<<c<<" "<<c->getParent()->getConfiguration().transpose()<<" ("<<c->getParent()<<") --> "<<
            c->getChild()->getConfiguration().transpose()<<" ("<<c->getChild()<<")";

        fcn(c->getChild());
      }
      return;
    };

    fcn(tree.root_);
  }

  return os;
}

TreePtr Tree::fromYAML(const YAML::Node& yaml,
                       const MetricsPtr& metrics,
                       const CollisionCheckerPtr& checker,
                       const cnr_logger::TraceLoggerPtr& logger)
{
  if (!yaml["nodes"])
  {
    CNR_ERROR(logger, "Cannot load a tree from a YAML without 'nodes' field");
    return nullptr;
  }

  if (!yaml["connections"])
  {
    CNR_ERROR(logger, "Cannot load a tree from a YAML without 'connections' field");
    return nullptr;
  }

  bool use_kdtree = true;
  if (!yaml["use_kdtree"])
    CNR_WARN(logger, "No field 'use_kdtree' field found in YAML, set to true");
  else
    use_kdtree = yaml["use_kdtree"].as<bool>();

  int max_distance = -1;
  bool compute_max_distance = false;
  if (!yaml["max_distance"])
  {
    CNR_WARN(logger, "No field 'max_distance' field found in YAML, computed from the tree");
    compute_max_distance = true;
  }
  else
    max_distance = yaml["max_distance"].as<int>();

  YAML::Node nodes = yaml["nodes"];
  YAML::Node connections = yaml["connections"];

  if (!nodes.IsSequence())
  {
    CNR_ERROR(logger, "Cannot load a tree from YAML::Node where 'nodes' is not a sequence");
    return nullptr;
  }

  if (!connections.IsSequence())
  {
    CNR_ERROR(logger, "Cannot load a tree from YAML::Node where 'connections' is not a sequence");
    return nullptr;
  }

  std::vector<NodePtr> nodes_vector(nodes.size());

  NodePtr node;
  for(size_t in = 0; in<nodes.size(); in++)
  {
    node = Node::fromYAML(nodes[in],logger);

    if(!node)
    {
      // Error creating a Node from YAML
      CNR_ERROR(logger, "Error creating a Node from YAML");
      return nullptr;
    }

    nodes_vector[in] = node;
  }

  double length;
  int in1, in2;
  NodePtr n1, n2;
  ConnectionPtr conn;
  YAML::Node connection;
  for(size_t ic=0;ic<connections.size();ic++)
  {
    connection = connections[ic];
    if(!connection.IsSequence())
    {
      CNR_ERROR(logger,"Cannot load connections from YAML::Node if the field connections is not a sequence of sequence");
      return nullptr;
    }

    in1 = connection[0].as<int>();
    in2 = connection[1].as<int>();

    n1 = nodes_vector.at(in1);
    n2 = nodes_vector.at(in2);

    conn = std::make_shared<Connection>(n1, n2, logger);
    conn->setCost(metrics->cost(n1, n2));
    conn->add();

    if(compute_max_distance)
    {
      length = conn->norm();
      if(length>max_distance)
        max_distance = length;
    }
  }

  NodePtr root;
  for(const NodePtr& n: nodes_vector)
  {
    if(n->getParentConnectionsSize() == 0)
    {
      root = n;
      break;
    }
  }

  TreePtr tree = std::make_shared<Tree>(root, max_distance, checker, metrics, logger, use_kdtree);

  for (size_t inode = 1; inode < nodes_vector.size(); inode++)
  {
    tree->addNode(nodes_vector[inode], false);
  }

  return tree;
}

bool Tree::changeRoot(const NodePtr& node)
{
  if (not isInTree(node))
    return false;

  std::vector<ConnectionPtr> connections = getConnectionToNode(node);
  for (ConnectionPtr& conn: connections)
    conn->flip();

  root_=node;

  return true;
}

bool Tree::recheckCollision()
{
  return recheckCollisionFromNode(root_);
}
bool Tree::recheckCollisionFromNode(NodePtr& n)
{
  NodePtr child;
  unsigned int removed_nodes;
  std::vector<NodePtr> white_list;

  std::vector<ConnectionPtr> n_conns = n->getChildConnections();
  for(ConnectionPtr& conn:n_conns)
  {
    child=conn->getChild();

    if (not checker_->checkConnection(conn))
    {
      purgeFromHere(child,white_list,removed_nodes);
      return false;
    }
    if(not recheckCollisionFromNode(child))
      return false;
  }
  return true;
}

bool get_param(const cnr_logger::TraceLoggerPtr& logger, const std::string param_ns, const std::string param_name, TreePtr& param,
               const MetricsPtr& metrics, const CollisionCheckerPtr& checker)
{

  std::string what, full_param_name = param_ns+"/"+param_name;
  if(cnr::param::has(full_param_name, what))
  {
    YAML::Node yaml_node;
    if(not cnr::param::get(full_param_name, yaml_node, what))
    {
      CNR_ERROR(logger, "Cannot load " << full_param_name + " parameter.\n"<<what);
      throw std::invalid_argument("Cannot load " + full_param_name + " parameter.");
    }
    else
      param = graph::core::Tree::fromYAML(yaml_node,metrics,checker,logger);
  }
  else
  {
    CNR_WARN(logger, full_param_name + " parameter not available.\n"<<what);
    return false;
  }
  return true;
}

} //end namespace core
} // end namespace graph
