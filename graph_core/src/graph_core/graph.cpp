#include <graph_core/graph.h>

namespace ha_planner {


Graph::Graph(const unsigned int& dof,
         const std::string& group_name,
         const planning_scene::PlanningSceneConstPtr& planning_scene,
         const std::vector<double> scaling,
         const std::vector<double> lb,
         const std::vector<double> ub):
  m_planning_scene(planning_scene)
{

  m_dof=dof;
  m_lb.resize(m_dof,0);
  m_ub.resize(m_dof,0);


  m_node_params.group_name=group_name;
  m_node_params.group_name=m_node_params.group_name;
  m_node_params.checking_collision_distance=0.06;
  m_node_params.scaling=scaling;
  m_node_params.unscaling.resize(scaling.size());
  m_node_params.weigth=50;

  if (scaling.size()!=m_dof)
    ROS_ERROR("scaling dimension are wrong (%zu instead of %u)",scaling.size(),m_dof);
  m_unscaling.resize(m_dof);
  for (unsigned int idx=0;idx<scaling.size();idx++)
  {
    if (scaling.at(idx)<=0)
    {
      ROS_ERROR("scaling should be positive");
      throw std::invalid_argument("scaling should be positive");
    }
    m_node_params.unscaling.at(idx)=1.0/scaling.at(idx);
    m_unscaling(idx)=1.0/scaling.at(idx);
    m_lb.at(idx)=lb.at(idx)*scaling.at(idx);
    m_ub.at(idx)=ub.at(idx)*scaling.at(idx);
  }
}


NodePtr Graph::searchClosestNode(const std::vector<double>& q)
{
  double min_square_distance=std::numeric_limits<double>::infinity();
  NodePtr closest_node;
  for (const NodePtr& node: m_nodes)
  {
    double square_dist=squareDistance(q,node->getJoints());
    if (square_dist<min_square_distance)
    {
      if (node->getParents().size()>0 && node->getChilds().size()>0)
      {
        closest_node=node;
        min_square_distance=square_dist;
      }
    }
  }
  if (!closest_node)
    ROS_DEBUG("unable to find a node");
  return closest_node;
}

NodePtr Graph::addNode(const std::vector<double>& q)
{

  NodePtr new_node=std::make_shared<Node>(multiply(q,m_node_params.scaling),m_node_params);
  m_nodes.push_back(new_node);
  return new_node;
}

ConnectionPtr Graph::addConnection(const NodePtr &parent, const NodePtr &child)
{
  ConnectionPtr conn=std::make_shared<Connection>(parent,child,m_node_params);
  return conn;
}
NodePtr Graph::addChildNode(const NodePtr& parent_node, const std::vector<double>& q)
{
  NodePtr node=addNode(q);
  ConnectionPtr conn=addConnection(parent_node,node);
  conn->registerConnection();
  return node;
}

NodePtr Graph::addParentNode(const NodePtr& child_node, const std::vector<double>& q)
{
  NodePtr node=addNode(q);
  ConnectionPtr conn=addConnection(node,child_node);
  conn->registerConnection();
  return node;
}

NodePtr Graph::addConnectedNode(const NodePtr &node, const std::vector<double> &q, Direction dir)
{
  if (dir==Forward)
    return addChildNode(node,q);
  else
    return addParentNode(node,q);
}

unsigned int Graph::removeUnconnectedNodes()
{
  unsigned int num_of_removed_nodes=0;
  for (unsigned int in=0;in<m_nodes.size();in++)
  {
    assert(0);
//    if (m_nodes.at(in)==m_start_node)
//      continue;
//    if (std::find(m_end_nodes.begin(),m_end_nodes.end(),m_nodes.at(in))!=m_end_nodes.end())
//      continue;

    if (m_nodes.at(in)->isUnconnected(m_planning_scene))
    {
      removeNodeWithConnections(m_nodes.at(in));
      num_of_removed_nodes++;
    }
  }
  return num_of_removed_nodes;
}

void Graph::removeNodeWithConnections(NodePtr &node)
{
  for (ConnectionPtr conn: node->m_connections)
  {
    conn->getOtherNode(node)->removeConnection(conn);
  }
  node->m_connections.clear();

  std::vector<NodePtr>::iterator el=std::find(m_nodes.begin(),m_nodes.end(),node);
  if (el!=m_nodes.end())
    m_nodes.erase(el,el+1);
}

}

