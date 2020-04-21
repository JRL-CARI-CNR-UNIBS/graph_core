#include <net_path_core/net_path_core.h>

namespace ha_planner {


Connection::Connection(const NodePtr& parent,
                       const NodePtr& child,
                       const ConnectionParam& connection_parameters):
  m_params(connection_parameters)
{
  m_parent=parent;
  m_child=child;


  m_is_collision_checked=false;
  m_is_in_collision=false;
  m_pheromone=1;
  m_collision_probability=0.5;
  computeLength();
  computeHeuristic();


}

ConnectionPtr Connection::createFlippedConnection()
{
  ConnectionPtr new_conn=std::make_shared<Connection>(m_child,m_parent,m_params);
  if (m_is_collision_checked)
  {
    if (m_is_in_collision)
      new_conn->forceCollision();
    else
      new_conn->forceNotCollision();
  }
  new_conn->updatePheromone(m_pheromone);
  return new_conn;
}

double Connection::getHeuristic()
{
  if (m_is_collision_checked)
    if (m_is_in_collision)
      return 0;
    else
      return m_heuristic;
  else
    return m_heuristic/*/m_collision_probability*/;
}

bool Connection::isInCollision(const planning_scene::PlanningSceneConstPtr &planning_scene)
{
  if (!m_is_collision_checked)
  {
    m_is_collision_checked=true;

    if (m_parent->isInCollision(planning_scene) || m_child->isInCollision(planning_scene))
    {
      m_is_in_collision=true;
      return m_is_in_collision;
    }
    checkCollisionNew(planning_scene);
  }
  return m_is_in_collision;
}

double Connection::getLength()
{
  double length=std::sqrt(m_square_length)*(1+m_params.weight*m_child->getCost());
  return length;
}

void Connection::checkCollisionNew(const planning_scene::PlanningSceneConstPtr& planning_scene)
{
  std::vector<double> q1=m_parent->getJoints();
  std::vector<double> q2=m_child->getJoints();

  robot_state::RobotState state = planning_scene->getCurrentState();
  for (size_t idof=0;idof<q1.size();idof++)
  {
    q1.at(idof)*=m_params.unscaling.at(idof);
    q2.at(idof)*=m_params.unscaling.at(idof);
  }
  if (std::sqrt(squareDistance(q1,q2))<m_params.checking_collision_distance)
  {
    m_is_in_collision=false;
    return;
  }
  m_is_in_collision=checkCollisionIteration(planning_scene,q1,q2,state);
}

bool Connection::checkCollisionIteration(const planning_scene::PlanningSceneConstPtr &planning_scene, const std::vector<double> &q1, const std::vector<double> &q2, robot_state::RobotState& state)
{
  std::vector<double> qi(q1.size());
  bool below_check_distance=true;
  for (size_t idof=0;idof<q1.size();idof++)
  {
    qi.at(idof)=0.5*(q1.at(idof)+q2.at(idof));
    if (below_check_distance)
      if (std::abs(q1.at(idof)-q2.at(idof))>m_params.checking_collision_distance)
        below_check_distance=false;
  }


  state.setJointGroupPositions(m_params.group_name,qi);
  state.updateCollisionBodyTransforms();

//  if (!planning_scene->isStateValid(state));
//    return true;
  collision_detection::CollisionRequest req;
  collision_detection::CollisionResult res;
  planning_scene->checkCollision(req, res, state);
  if (res.collision)
    return true;

    if (!planning_scene->isStateValid(state))
    {
      ROS_ERROR("Sometimes wrong, analyzing...");
      if (planning_scene->isStateColliding(state))
        ROS_ERROR("state in collision");
      else
        ROS_INFO("state not in collision");
      if (!planning_scene->isStateFeasible(state))
        ROS_ERROR("path is not feasible");
      else
        ROS_INFO("path is feasible");
      static const moveit_msgs::Constraints emp_constraints;
      if (!planning_scene->isStateConstrained(state,emp_constraints))
        ROS_ERROR("path is out of constraints");
      else
        ROS_INFO("path is constrained");
    }

  if (!below_check_distance)
    return checkCollisionIteration(planning_scene,q1,qi,state) || checkCollisionIteration(planning_scene,qi,q2,state);

  return false;
}

void Connection::computeLength()
{
  m_square_length = squareDistance(m_parent->getJoints(),m_child->getJoints());
}

void Connection::computeHeuristic()
{
  m_heuristic=1.0/(std::sqrt(m_square_length)+1e-3) * (m_is_in_collision || !m_is_collision_checked);
}

void Connection::getNodes(NodePtr &parent, NodePtr &child)
{
  parent=m_parent;
  child=m_child;
}

const NodePtr& Connection::getOtherNode(const NodePtr &node)
{
  if (node==m_parent)
    return m_child;
  else if (node==m_child)
    return m_parent;
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
  m_parent->print();
  std::cout << "->\n";
  m_child->print();

}

Eigen::VectorXd Connection::versor()
{
  Eigen::Map<const Eigen::VectorXd> p1(m_parent->getJoints().data(),m_parent->getJoints().size());
  Eigen::Map<const Eigen::VectorXd> p2(m_child->getJoints().data(),m_parent->getJoints().size());
  Eigen::VectorXd v=p2-p1;
  return v.normalized();
}

double Connection::dotProduct(const ConnectionPtr &conn)
{
  return versor().dot(conn->versor());
}

void Connection::flipDirection()
{
  std::vector<ConnectionPtr>::iterator it;
  it=std::find(m_parent->m_child_connections.begin(),m_parent->m_child_connections.end(),pointer());
  if (it!=m_parent->m_child_connections.end())
  {
    m_parent->m_child_connections.erase(it);
    m_parent->m_parent_connections.push_back(pointer());
  }

  it=std::find(m_child->m_parent_connections.begin(),m_child->m_parent_connections.end(),pointer());
  if (it!=m_child->m_parent_connections.end())
  {
    m_child->m_parent_connections.erase(it);
    m_child->m_child_connections.push_back(pointer());
  }
  m_parent.swap(m_child);

}

void Connection::registerConnection()
{
  m_parent->addConnection(pointer());
  m_child->addConnection(pointer());
}
}

