#include <net_path_core/net_path_core.h>

namespace ha_planner {

Node::Node(const std::vector<double>& q, const NodeParams& node_parameters, const ConnectionParam& connection_parameters):
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

NodePtr Node::createConnections(const double& max_distance,
                                const double& min_distance,
                                const std::vector<NodePtr>& actual_nodes,
                                unsigned int &connections)
{
  for (const NodePtr& node: actual_nodes)
  {
    ConnectionPtr connection=std::make_shared<Connection>(node,pointer(),m_conn_params);
    if (connection->getLength()<min_distance)
    {
      ROS_DEBUG("this node is too close to another one, merging");
      for (ConnectionPtr& conn1: m_connections)
      {
        // conn1 is between this and conn1->getOtherNode(pointer())
        // check if conn1->getOtherNode(pointer()) is already connect with node
        bool is_already_present=false;
        for (ConnectionPtr& conn2: node->m_connections)
          if (conn1->getOtherNode(pointer())==conn2->getOtherNode(node))
            is_already_present=true;

        // if not, add a connection between node and conn1->getOtherNode(pointer())
        if (!is_already_present)
        {
          ConnectionPtr new_connection=std::make_shared<Connection>(node,conn1->getOtherNode(pointer()),m_conn_params);
          node->addConnection(new_connection);
          conn1->getOtherNode(pointer())->addConnection(new_connection);
        }

        // remove connection 1
        conn1->getOtherNode(pointer())->removeConnection(conn1);

      }

      connections=node->m_connections.size();
      return node;
    }
    if ((connection->getLength()<=max_distance))
    {
      connections++;
      addConnection(connection);
      node->addConnection(connection);

      ConnectionPtr connection_backward=std::make_shared<Connection>(pointer(),node,m_conn_params);
      connections++;
      addConnection(connection_backward);
      node->addConnection(connection_backward);

    }
  }
  return pointer();
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
  robot_state::RobotState state = planning_scene->getCurrentState();

  state.setJointGroupPositions(m_params.group_name,m_q);
  planning_scene->checkCollision(req, res, state);
  m_is_in_collision=res.collision;

}

int Node::rouletteWheel(const Direction direction)
{
  if (m_connections.size()==0)
    return -1;

  std::vector<double> p(m_connections.size(),0);

  for (unsigned int idx=0;idx<p.size();idx++)
  {
    double p_el=(m_connections.at(idx)->getHeuristic()+m_connections.at(idx)->getOtherNode(pointer())->getHeuristic())*m_connections.at(idx)->getPheromone();
    if (direction==Forward)
    {
      p_el*=m_connections.at(idx)->getParent()==pointer();
    }
    else if (direction==Backward)
    {
      p_el*=m_connections.at(idx)->getChild()==pointer();
    }


    if (idx>0)
      p.at(idx)=p.at(idx-1)+p_el;
    else
      p.at(idx)=p_el;
  }

  double random_value =  ((double)std::rand())/((double)RAND_MAX)*p.back();
  unsigned int idx=0;
  for (idx=0;idx<(p.size()-1);idx++)
    if (random_value<p.at(idx))
      break;

  return idx;
}

void Node::addConnection(ConnectionPtr &connection)
{
  m_connections.push_back(connection);
}

bool Node::removeConnection(ConnectionPtr &connection)
{
  for (size_t idx=0;idx<m_connections.size();idx++)
  {
    if (m_connections.at(idx)==connection)
    {
      m_connections.erase(m_connections.begin()+idx);
      return true;
    }
  }
  return false;
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
  if (m_is_collision_checked)
    if (m_is_in_collision)
      return true;
  unsigned int n_conn=0;
  for (const ConnectionPtr& conn: m_connections)
  {
    if (conn->isCollisionChecked())
    {
      if (!conn->isInCollision(planning_scene))
        n_conn++;
    }
    else
      n_conn++;
  }
  return n_conn==0;
}


ConnectionPtr Node::findConnectionToParent()
{
  for (const ConnectionPtr conn: m_connections)
  {
    if (conn->getChild()==pointer())
      return conn;
  }
}
const NodePtr& Node::findParent()
{
  return findConnectionToParent()->getParent();

}



}

