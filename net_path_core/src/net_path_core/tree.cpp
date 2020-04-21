#include <net_path_core/tree.h>

namespace ha_planner
{

Tree::Tree(const NodePtr &root_node,
           const NodeParams& node_parameters,
           const ConnectionParam& connection_parameters,
           const planning_scene::PlanningSceneConstPtr& planning_scene,
           const double& max_square_length, const Direction& direction,
           std::vector<NodePtr>& nodes,
           std::vector<NodePtr>& endnodes):
  m_node_parameters(node_parameters),
  m_connection_parameters(connection_parameters),
  m_planning_scene(planning_scene),
  m_max_square_length(max_square_length),
  m_direction(direction),
  m_nodes(nodes),
  m_end_nodes(endnodes)
{
  m_root_node=root_node;
  m_tree_nodes.push_back(root_node);
  m_max_length=std::sqrt(m_max_square_length);
  m_frontier_threshold=0.05*m_max_length;

  m_dof=root_node->getJoints().size();
  m_unscaling.resize(m_dof);
  for (unsigned int idx=0;idx<m_dof;idx++)
  {
    m_unscaling(idx)=1.0/m_connection_parameters.scaling.at(idx);
  }
}

// return true if all the nodes to q are added, false if there was a collision in the middle
bool Tree::createAndExtend(const std::vector<double> &q, NodePtr& last_add_node)
{
  NodePtr goal_node=std::make_shared<Node>(q,m_node_parameters,m_connection_parameters);
  computeOccupancy(goal_node);
  if (!extend(goal_node,last_add_node))
    return false;

  m_nodes.push_back(goal_node);

  return true;
}

bool Tree::extend(const NodePtr& n, NodePtr& last_add_node)
{
  // find closest node in the tree
  double min_distance=std::numeric_limits<double>::infinity();
  NodePtr closest_node;
  for (const NodePtr& node: m_tree_nodes)
  {
    double dist=squareDistance(n->getJoints(),node->getJoints());
    if (dist<min_distance)
    {
      closest_node=node;
      min_distance=dist;
    }
  }
  return extendFromNode(n,closest_node,last_add_node);

}

bool Tree::createAndExtendFromNode(const std::vector<double>& q, const NodePtr starting_node, NodePtr &last_add_node)
{
  NodePtr goal_node=std::make_shared<Node>(q,m_node_parameters,m_connection_parameters);
  computeOccupancy(goal_node);
  if (!extendFromNode(goal_node,starting_node,last_add_node))
    return false;

  m_nodes.push_back(goal_node);

  return true;
}

bool Tree::extendFromNode(const NodePtr& n, const NodePtr starting_node, NodePtr& last_add_node)
{

  // initialize last added node to closest one
  last_add_node=starting_node;

  ConnectionPtr new_conn;

  // check if there is already a connection
  if (starting_node->checkIfConnectedWith(n,new_conn))
  {
    if ( ((m_direction==Direction::Forward) && (new_conn->getChild()!=n)) || ((m_direction==Direction::Backward) && (new_conn->getParent()!=n)) )
    {
      ROS_DEBUG("this connection is already present in the other direction");
      return false;
    }
    else
    {
      ROS_DEBUG("this connection is already present");
      if (new_conn->isInCollision(m_planning_scene))
        return false;

      m_tree_nodes.push_back(n);
      m_tree_connections.push_back(new_conn);
      return true;
    }
  }

  double distance=std::sqrt(squareDistance(starting_node->getJoints(),n->getJoints()));

  if (m_expansion_control)
    if (!minExpansionControl(distance))
      return false;

  const std::vector<double>& start_point=starting_node->getJoints();
  unsigned int n_pnts=std::ceil(distance/m_max_length);

  NodePtr new_node;

  for (unsigned int ipnt=1;ipnt<=n_pnts;ipnt++)
  {

    if (ipnt<n_pnts)
    {
      std::vector<double> qstep(n->getJoints().size());
      for (size_t i_dof=0;i_dof<start_point.size();i_dof++)
      {
        qstep.at(i_dof)=start_point.at(i_dof)+(n->getJoints().at(i_dof)-start_point.at(i_dof))*((double)ipnt)/((double)n_pnts);
      }

      new_node=std::make_shared<Node>(qstep,m_node_parameters,m_connection_parameters);
      computeOccupancy(new_node);
    }
    else
    {
      new_node=n;
    }


    if (new_node->isInCollision(m_planning_scene))
    {
      break;
    }

    new_conn=std::make_shared<Connection>(last_add_node,new_node,m_connection_parameters);
    if (m_direction==Direction::Backward)
      new_conn->flipDirection();

    if (m_transition_test)
    {
      new_node->computeHeuristic(m_end_nodes);
      if (!transitionTest(new_conn))
      {
        break;
      }
    }

    if (new_conn->isInCollision(m_planning_scene))
    {
      break;
    }
    last_add_node=new_node;

    if (!m_connect_mode)
      break;

  }

  if (last_add_node==starting_node)
  {
    return false;
  }
  new_conn=std::make_shared<Connection>(starting_node,last_add_node,m_connection_parameters);
  if (m_direction==Direction::Backward)
    new_conn->flipDirection();
  new_conn->forceNotCollision();
  new_conn->registerConnection();

  m_tree_nodes.push_back(last_add_node);
  m_tree_connections.push_back(new_conn);
  if (last_add_node!=n)
  {
    m_nodes.push_back(last_add_node);
    return false;
  }

  return true;
}

bool Tree::getPathToNode(const NodePtr &n, Path &path) const
{
  if (std::find(m_tree_nodes.begin(),m_tree_nodes.end(),n)==m_tree_nodes.end())
  {
    ROS_ERROR("Node is not part of the tree");
    return false;
  }

  NodePtr actual_node=n;
  Path branch;

  while (actual_node!=m_root_node)
  {
    ConnectionPtr actual_connection;
    if (m_direction==Backward)
    {

      for (ConnectionPtr& conn: actual_node->m_child_connections)
      {
        // check if parent is in this tree
        if (std::find(m_tree_nodes.begin(),m_tree_nodes.end(),conn->getChild())!=m_tree_nodes.end())
        {
          actual_connection=conn;
          break;
        }
      }
    }
    else
    {
      for (ConnectionPtr& conn: actual_node->m_parent_connections)
      {
        // check if parent is in this tree
        if (std::find(m_tree_nodes.begin(),m_tree_nodes.end(),conn->getParent())!=m_tree_nodes.end())
        {
          actual_connection=conn;
          break;
        }
      }
    }
    if (!actual_connection)
    {
      ROS_ERROR("unable to find a valid connection");
      return false;
    }

    branch.push_back(actual_connection);
    actual_node=actual_connection->getOtherNode(actual_node);
  }

  for (unsigned int idx=0;idx<branch.size();idx++)
  {

    if (m_direction==Backward)
      path.push_back(branch.at(idx));
    else
      path.push_back(branch.at(branch.size()-1-idx));
  }
  return true;
}

void Tree::changeTreeDirection(const Direction &direction)
{
  if (m_direction!=direction)
  {
    m_direction=direction;
    for (ConnectionPtr& conn: m_tree_connections)
      conn->flipDirection();
  }
}

bool Tree::changeRoot(const NodePtr &node)
{
  std::vector<NodePtr>::iterator it;
  it=std::find(m_tree_nodes.begin(),m_tree_nodes.end(),node);
  if (it==m_tree_nodes.end())
  {
    ROS_ERROR("the desired new root is not part of the tree");
    return false;
  }

  Path path;
  if (!getPathToNode(node,path))
  {
    ROS_ERROR("unable to find a path between old root");
    return false;
  }

  for (ConnectionPtr& conn: path)
    conn->flipDirection();
  m_root_node=node;
  return true;
}

bool Tree::addSubTree(const TreePtr& subtree)
{
  std::vector<NodePtr>::iterator it;
  it=std::find(m_tree_nodes.begin(),m_tree_nodes.end(),subtree->getRoot());
  if (it==m_tree_nodes.end())
  {
    ROS_DEBUG("subtree root is not part of the main tree");
    return false;
  }

  if (subtree->m_direction!=m_direction)
  {
    ROS_DEBUG("subtree direction is not equal to the main tree's one");
    return false;
  }

  for (ConnectionPtr& conn: subtree->getConnections())
  {
    NodePtr node=subtree->getRoot();
    if (m_direction==Backward)
      node=conn->getParent();
    else
      node=conn->getChild();
    m_tree_nodes.push_back(node);
    m_tree_connections.push_back(conn);
  }
  return true;
}

bool Tree::minExpansionControl(const double& dist)
{

  if (dist > m_frontier_threshold)  // Exploration
  {
    ++m_frontier_count;
    return true;
  }
  // Refinement
  // Check the current ratio first before accepting it
  if ((double)m_non_frontier_count  > m_frontier_ratio*(double)m_frontier_count)
  {
    return false;
  }
  ++m_non_frontier_count;
  return true;
}

bool Tree::transitionTest(const ConnectionPtr &conn)
{
  double parent_cost=conn->getParent()->getCost();

  double child_cost=conn->getChild()->getCost();

  m_max_cost=std::max(m_max_cost,parent_cost);
  m_max_cost=std::max(m_max_cost,child_cost);

  m_min_cost=std::min(m_min_cost,parent_cost);
  m_min_cost=std::min(m_min_cost,child_cost);

  double ci,cj;

  if (m_direction==Forward)
  {
    ci=parent_cost;
    cj=child_cost;
  }
  else
  {
    cj=parent_cost;
    ci=child_cost;
  }

  if (cj<ci)
    return true;
  if (std::exp((ci-cj)/m_temperature)>0.5)
  {
    if (m_max_cost>m_min_cost)
      m_temperature/=std::exp2(0.5*(cj-ci)/(m_max_cost-m_min_cost));
    return true;
  }
  m_temperature*=m_temperature_rate;
  return false;
}

bool Tree::tryConnectWith(const NodePtr &n)
{
  if (std::find(m_tree_nodes.begin(),m_tree_nodes.end(),n)!=m_tree_nodes.end())
  {
    ROS_ERROR("the node cannot be part on the tree");
    return false;
  }
  for (NodePtr& node: m_tree_nodes)
  {
    if (squareDistance(node->getJoints(),n->getJoints())>m_max_square_length)
      continue;

    ConnectionPtr conn;
    if (m_direction==Forward)
      conn=std::make_shared<Connection>(node,n,m_connection_parameters);
    else
      conn=std::make_shared<Connection>(n,node,m_connection_parameters);


    if (conn->isInCollision(m_planning_scene))
      continue;

    conn->registerConnection();
    m_tree_nodes.push_back(n);
    m_tree_connections.push_back(conn);
    return true;
  }
  return false;
}


void Tree::computeOccupancy(NodePtr& node)
{

  if (m_human_filter && node)
  {
    Eigen::Map<const Eigen::VectorXd> q(node->getJoints().data(),m_dof);
    node->setCost(m_human_filter->occupancy(q.cwiseProduct(m_unscaling)));
  }

}
}

