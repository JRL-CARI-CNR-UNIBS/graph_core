#include <graph_core/node.h>

namespace ha_planner {

Node::Node(const std::vector<double>& q, const NodeParams& node_parameters):
  m_params(node_parameters)
{
  m_q=q;
  m_is_collision_checked=false;
  m_is_in_collision=false;
}

bool Node::isInCollision(const planning_scene::PlanningSceneConstPtr &planning_scene)
{
  if (!m_is_collision_checked)
  {

    m_is_in_collision=false;

    collision_detection::CollisionRequest req;
    collision_detection::CollisionResult res;
    std::vector<double> q=multiply(m_q,m_params.unscaling);
    robot_state::RobotState state = planning_scene->getCurrentState();
    state.setJointGroupPositions(m_params.group_name,q);
    planning_scene->checkCollision(req, res, state);
    m_is_in_collision=res.collision;
    m_is_collision_checked=true;
  }

  return m_is_in_collision;
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

std::vector<double> Node::getJoints()
{
  return multiply(m_q,m_params.unscaling);
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

  return erased;
}

void Node::print()
{
  for (const double& p: m_q)
    std::cout << p << ",";
  std::cout << std::endl;
}


std::vector<NodePtr> Node::getParents()
{
  std::vector<NodePtr> parents;
  for (const ConnectionPtr conn: m_parent_connections)
    parents.push_back(conn->getParent());
  return parents;
}


std::vector<NodePtr> Node::getChilds()
{
  std::vector<NodePtr> childs;
  for (const ConnectionPtr conn: m_child_connections)
  {
    childs.push_back(conn->getChild());
  }
  return childs;
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

