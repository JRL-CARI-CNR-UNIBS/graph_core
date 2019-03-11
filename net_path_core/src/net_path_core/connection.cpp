#include <net_path_core/net_path_core.h>

namespace ha_planner {


Connection::Connection(const NodePtr& node1,
                       const NodePtr& node2,
                       const ConnectionParam& connection_parameters):
  m_params(connection_parameters)
{
  m_node1=node1;
  m_node2=node2;

  m_is_collision_checked=false;
  m_is_in_collision=false;
  m_pheromone=0.5;
  m_collision_probability=0.5;
  computeLength();
  computeHeuristic();
}

double Connection::getHeuristic()
{
  if (m_is_collision_checked)
    if (m_is_in_collision)
      return 0;
    else
      return m_heuristic;
  else
    return m_heuristic/m_collision_probability;
}
bool Connection::isInCollision(const planning_scene::PlanningSceneConstPtr &planning_scene)
{
  if (!m_is_collision_checked)
  {
    m_is_collision_checked=true;

    if (m_node1->isInCollision(planning_scene) || m_node2->isInCollision(planning_scene))
    {
      m_is_in_collision=true;
      return m_is_in_collision;
    }
    checkCollision(planning_scene);
  }
  return m_is_in_collision;
}

void Connection::checkCollision(const planning_scene::PlanningSceneConstPtr& planning_scene)
{
  std::vector<std::vector<double>> intermediate_points=intermediatePoints(m_node1->getJoints(),m_node2->getJoints(),m_params.checking_collision_distance);
  m_is_in_collision=false;

  collision_detection::CollisionRequest req;
  collision_detection::CollisionResult res;
  robot_state::RobotState state = planning_scene->getCurrentState();

  for (unsigned int ipnt=0;ipnt<intermediate_points.size();ipnt++)
  {
    state.setJointGroupPositions(m_params.group_name,intermediate_points.at(ipnt));
    planning_scene->checkCollision(req, res, state);
    if (res.collision)
    {
      m_is_in_collision=true;
      return;
    }
  }

}

void Connection::computeLength()
{
  m_square_length = squareDistance(m_node1->getJoints(),m_node2->getJoints());
}

void Connection::computeHeuristic()
{
  m_heuristic=1.0/(std::sqrt(m_square_length)+1e-3) * (m_is_in_collision || !m_is_collision_checked);
}

void Connection::getNodes(NodePtr &node1, NodePtr &node2)
{
  node1=m_node1;
  node2=m_node2;
}

const NodePtr& Connection::getOtherNode(const NodePtr &node)
{
  if (node==m_node1)
    return m_node2;
  else if (node==m_node2)
    return m_node1;
  else
  {
    ROS_ERROR("broken connection");
    throw std::out_of_range("this connection is pointing to a broken node");
    return NULL;
  }
}

void Connection::updatePheromone(const double &new_pheromone)
{
  m_pheromone=new_pheromone;
}

void Connection::print()
{
  ROS_INFO("connection between nodes:");
  m_node1->print();
  std::cout << "->\n";
  m_node2->print();

}

Eigen::VectorXd Connection::versor()
{
  Eigen::Map<const Eigen::VectorXd> p1(m_node1->getJoints().data(),m_node1->getJoints().size());
  Eigen::Map<const Eigen::VectorXd> p2(m_node2->getJoints().data(),m_node1->getJoints().size());
  Eigen::VectorXd v=p2-p1;
  return v.normalized();
}

double Connection::dotProduct(const ConnectionPtr &conn)
{
  return versor().dot(conn->versor());
}

}

