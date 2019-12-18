#include <net_path_core/net_path_core.h>

namespace ha_planner {

Node::Node(const std::vector<double>& q, const NodeParams& node_parameters):
  m_params(node_parameters),
  m_conn_params(connection_parameters)
{
  m_q=q;
  m_is_collision_checked=false;
  m_is_in_collision=false;
}

bool Node::isInCollision(const planning_scene::PlanningSceneConstPtr &planning_scene)
{
  if (!m_is_collision_checked)
  {

    checkCollision(planning_scene);
    m_is_collision_checked=true;
  }

  return m_is_in_collision;
}


void Node::computeHeuristic(const std::vector<NodePtr >& end_points)
{

  double min_square_distance=std::numeric_limits<double>::infinity();
  for (const NodePtr& end_point: end_points)
  {
    min_square_distance=std::min(min_square_distance,squareDistance(end_point->getJoints(),m_q));
  }
  m_heuristic=1.0/(1e-3+std::sqrt(min_square_distance));
}

void Node::checkCollision(const planning_scene::PlanningSceneConstPtr &planning_scene)
{

  m_is_in_collision=false;

  collision_detection::CollisionRequest req;
  collision_detection::CollisionResult res;

  std::vector<double> q(m_q.size());

  for (unsigned int idof=0;idof<q.size();idof++)
    q.at(idof)*=m_conn_params.unscaling.at(idof);

  robot_state::RobotState state = planning_scene->getCurrentState();
  state.setJointGroupPositions(m_params.group_name,q);
  planning_scene->checkCollision(req, res, state);
  m_is_in_collision=res.collision;

}

int Node::rouletteWheel(const Direction direction)
{
  if (m_connections.size()==0)
  {
    ROS_DEBUG("no connection");
    return -1;
  }

  std::vector<double> p(m_connections.size(),0);

  for (unsigned int idx=0;idx<p.size();idx++)
  {
    double p_el=0;
    if (direction==Forward)
    {
      if (m_connections.at(idx)->getParent()==pointer())
        p_el=(m_connections.at(idx)->getHeuristic()+m_connections.at(idx)->getOtherNode(pointer())->getHeuristic())*m_connections.at(idx)->getPheromone();
    }
    else if (direction==Backward)
    {
      if (m_connections.at(idx)->getChild()==pointer())
        p_el=(m_connections.at(idx)->getHeuristic()+m_connections.at(idx)->getOtherNode(pointer())->getHeuristic())*m_connections.at(idx)->getPheromone();
    }



    if (idx>0)
      p.at(idx)=p.at(idx-1)+p_el;
    else
      p.at(idx)=p_el;

  }

  if (p.back()==0)
  {
    if (direction==Forward)
      ROS_DEBUG("This node has no descendents");
    else
      ROS_DEBUG("This node has no ascendents");
    return -2;
  }
  double random_value =  ((double)std::rand())/((double)RAND_MAX)*p.back();
  unsigned int idx=0;
  for (idx=0;idx<(p.size()-1);idx++)
    if (random_value<p.at(idx))
      break;
  return idx;
}

void Node::addConnection(const ConnectionPtr &connection)
{

  if (std::find(m_connections.begin(),m_connections.end(),connection)==m_connections.end())
  {
    m_connections.push_back(connection);
    if (connection->getParent()==pointer())
      m_child_connections.push_back(connection);
    else
      m_parent_connections.push_back(connection);
  }
}

bool Node::removeConnection(ConnectionPtr &connection)
{
  bool erased=false;
  std::vector<ConnectionPtr>::iterator it;
  it=std::find(m_connections.begin(),m_connections.end(),connection);
  if (it!=m_connections.end())
  {
    m_connections.erase(it);
    erased=true;
  }
  it=std::find(m_parent_connections.begin(),m_parent_connections.end(),connection);
  if (it!=m_parent_connections.end())
  {
    m_parent_connections.erase(it);
  }
  it=std::find(m_child_connections.begin(),m_child_connections.end(),connection);
  if (it!=m_child_connections.end())
  {
    m_child_connections.erase(it);
  }

//  for (size_t idx=0;idx<m_connections.size();idx++)
//  {
//    if (m_connections.at(idx)==connection)
//    {
//      m_connections.erase(m_connections.begin()+idx);
//      erased=true;
//    }
//  }
//  for (size_t idx=0;idx<m_inner_connections.size();idx++)
//  {
//    if (m_inner_connections.at(idx)==connection)
//    {
//      m_inner_connections.erase(m_inner_connections.begin()+idx);
//    }
//  }
//  for (size_t idx=0;idx<m_outer_connections.size();idx++)
//  {
//    if (m_outer_connections.at(idx)==connection)
//    {
//      m_outer_connections.erase(m_outer_connections.begin()+idx);
//    }
//  }

  return erased;
}

void Node::print()
{
  for (const double& p: m_q)
    std::cout << p << ",";
  std::cout << std::endl;
}

bool Node::isUnconnected(const planning_scene::PlanningSceneConstPtr &planning_scene)
{
  if (m_connections.size()==0)
    return true;
  if (getDescendants().size()==0)
    return true;
  if (getAncestors().size()==0)
    return true;

  unsigned int good_conn;
  for (const ConnectionPtr conn: m_connections)
  {
    if (conn->isCollisionChecked())
    {
      if (!conn->isInCollision(planning_scene))
        good_conn++;
    }
    else
      good_conn++;
  }
  return good_conn==0;
}

//ConnectionPtr Node::findConnectionToParent()
//{
//  for (const ConnectionPtr conn: m_connections)
//  {
//    if (conn->getChild()==pointer())
//      return conn;
//  }
//  return NULL;
//}

//const NodePtr& Node::findParent()
//{
//  return findConnectionToParent()->getParent();

//}

std::vector<NodePtr> Node::getAncestors(const unsigned int& level)
{
  std::vector<NodePtr> ancestors;
  for (const ConnectionPtr conn: m_parent_connections)
  {
    ancestors.push_back(conn->getParent());
    if (level>1)
    {
      std::vector<NodePtr> its_ancestors=conn->getParent()->getAncestors(level-1);
      for (const NodePtr& n: its_ancestors)
        if (std::find(ancestors.begin(),ancestors.end(),n)==ancestors.end())
          ancestors.push_back(n);
    }
  }
  return ancestors;
}


std::vector<NodePtr> Node::getDescendants(const unsigned int& level)
{
  std::vector<NodePtr> descendants;
  for (const ConnectionPtr conn: m_child_connections)
  {
    descendants.push_back(conn->getChild());
    if (level>1)
    {
      std::vector<NodePtr> its_decendants=conn->getChild()->getDescendants(level-1);
      for (const NodePtr& n: its_decendants)
        if (std::find(descendants.begin(),descendants.end(),n)==descendants.end())
          descendants.push_back(n);
    }
  }
  return descendants;
}

bool Node::checkIfConnectedWith(const NodePtr &node, ConnectionPtr& connection)
{
  for (const ConnectionPtr& conn: m_connections)
  {
    if (conn->getOtherNode(pointer())==node)
    {
      connection=conn;
      return true;
    }
  }
  return false;
}

}

