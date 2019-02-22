#include <net_path_core/net_path_core.h>

namespace ha_planner {

double squareDistance(const std::vector<double> &q1, const std::vector<double> &q2)
{
  assert(q1.size()==q2.size());
  double square_length=0;
  for (size_t idx=0;idx<q1.size();idx++)
  {
    square_length+=std::pow((q1.at(idx)-q2.at(idx)),2.0);
  }
  return square_length;
}

Node::Node(const std::vector<double>& q, const NodeParams& node_parameters, const ConnectionParam& connection_parameters):
  m_params(node_parameters),
  m_conn_params(connection_parameters)
{
  m_q=q;
  m_is_collision_checked=false;
  m_is_in_collision=false;
  m_heuristic=1;
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

void Node::createConnections(const double& max_distance,
                             const double& min_distance,
                             const unsigned int& max_connections,
                             const std::vector<std::shared_ptr<Node>> &actual_nodes,
                             unsigned int &connections,
                             bool force_add)
{
  for (const std::shared_ptr<Node>& node: actual_nodes)
  {
    std::shared_ptr<Connection> connection=std::make_shared<Connection>(pointer(),node,m_conn_params);
    if (!force_add && (connection->getLength()<min_distance))
    {
      ROS_FATAL("this node is too close to another one");
      m_connections.resize(0);
      connections=false;
      return;
    }
    if ((connection->getLength()<=max_distance))
    {
      connections++;
      addConnection(connection,max_connections);
      node->addConnection(connection,max_connections);
    }
  }

}

void Node::computeHeuristic(const std::vector<std::shared_ptr<Node> >& end_points)
{

  double min_square_distance=std::numeric_limits<double>::infinity();
  for (const std::shared_ptr<Node>& end_point: end_points)
  {
    min_square_distance=std::min(min_square_distance,squareDistance(end_point->getJoints(),m_q));
  }
  m_heuristic=1.0/(1e-3+min_square_distance);
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

int Node::rouletteWheel()
{
  if (m_connections.size()==0)
    return -1;

  std::vector<double> p(m_connections.size(),0);
  p.at(0)=(m_connections.at(0)->getHeuristic()+m_connections.at(0)->getOtherNode(pointer())->getHeuristic())*m_connections.at(0)->getPheromone();

  for (unsigned int idx=1;idx<p.size();idx++)
    p.at(idx)=p.at(idx-1)+(m_connections.at(idx)->getHeuristic()+m_connections.at(idx)->getOtherNode(pointer())->getHeuristic())*m_connections.at(idx)->getPheromone();

  double random_value =  ((double)std::rand())/((double)RAND_MAX)*p.back();
  unsigned int idx=0;
  for (idx=0;idx<p.size();idx++)
    if (random_value<p.at(idx))
      break;
  return idx;
}

void Node::addConnection(std::shared_ptr<Connection> &connection, unsigned int max_connections)
{
  if ((m_connections.size())<max_connections)
    m_connections.push_back(connection);
  else
  {
    (m_connections.at(rouletteWheel())->getOtherNode(pointer()))->removeConnection(m_connections.at(rouletteWheel()));
    m_connections.at(rouletteWheel())=connection;
  }
}

bool Node::removeConnection(std::shared_ptr<Connection> &connection)
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

Connection::Connection(const std::shared_ptr<Node> &node1,
                       const std::shared_ptr<Node> &node2,
                       const ConnectionParam& connection_parameters):
  m_params(connection_parameters)
{
  m_node1=node1;
  m_node2=node2;

  m_is_collision_checked=false;
  m_is_in_collision=false;
  m_pheromone=1;
  computeLength();
  computeHeuristic();
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
  m_is_in_collision=false;
}

void Connection::computeLength()
{
  m_square_length = squareDistance(m_node1->getJoints(),m_node2->getJoints());
}

void Connection::computeHeuristic()
{
  m_heuristic=1.0/(m_square_length+1e-3) * (m_is_in_collision || !m_is_collision_checked);
}

const std::shared_ptr<Node>& Connection::getOtherNode(const std::shared_ptr<Node> &node)
{
  if (node==m_node1)
    return m_node2;
  else if (node==m_node2)
    return m_node1;
  else
    return NULL;
}

void Connection::updatePheromone(const double &new_pheromone)
{
  m_pheromone=new_pheromone;
}

Net::Net(const unsigned int& dof, const planning_scene::PlanningSceneConstPtr& planning_scene):
  m_planning_scene(planning_scene)
{
  m_dof=dof;
  m_min_square_length=0.05;
  m_max_square_length=1;
  m_max_connections_per_node=20;
  m_grid_global_nodes=0;

  m_sqrt_evaporation_ratio=std::sqrt(0.95);
  m_max_pheromone=10;
  m_min_pheromone=0.5;
  m_node_params.group_name="ur10";
  m_conn_params.group_name=m_node_params.group_name;

  m_best_cost = std::numeric_limits<double>::infinity();
  m_best_path.resize(0);


}

void Net::generateNodesFromGrid(const unsigned int &number_of_nodes, const std::vector<double> &lower_bound, const std::vector<double> &upper_bound)
{
  assert(upper_bound.size()==m_dof);
  if (m_nodes.size()>0)
  {
    ROS_WARN("generateNodesFromGrid should be call before other generation methods, removing all old note to reinitialize");
    m_nodes.resize(0);
  }
  m_points_per_dimension=std::ceil(std::pow(number_of_nodes,1.0/m_dof) );

  m_grid_global_nodes = std::pow(m_points_per_dimension,m_dof);
  std::vector<std::vector<double>> qgrid;
  for (size_t idof=0;idof<m_dof;idof++)
  {
    double delta=(upper_bound.at(idof)-lower_bound.at(idof))/(double)(m_points_per_dimension-1);
    std::vector<double> idim(m_points_per_dimension);
    ROS_FATAL("delta=%f",delta);

    idim.at(0)=lower_bound.at(idof);
    for (unsigned int idx=1;idx<m_points_per_dimension;idx++)
      idim.at(idx)=idim.at(idx-1)+delta;
    qgrid.push_back(idim);
  }

  std::vector<double> q(m_dof);
  std::vector<unsigned int> indices(m_dof,0);
  for (unsigned int in=0; in<m_grid_global_nodes;in++)
  {
    for (size_t idof=0;idof<m_dof;idof++)
      q.at(idof)=qgrid.at(idof).at(indices.at(idof));

    if (in%1000==0)
      ROS_FATAL("Creating a net of %u nodes, now the net has %zu nodes",m_grid_global_nodes,m_nodes.size());

    addNodeWithConnections(q);

    for (size_t idof=0;idof<m_dof;idof++)
    {

      indices.at(idof)++;
      if (indices.at(idof)>=m_points_per_dimension) // if true, the next for iteration increment the following indices
        indices.at(idof)=0;
      else  // if false, do not increment next indices
        break;
    }
  }

  ROS_FATAL("Created a net of %zu nodes",m_nodes.size());

}

void Net::generateNodesFromStartAndEndPoints(const std::vector<double> &start_point, const std::vector<std::vector<double> > &end_points)
{
  assert(m_dof==start_point.size());

  m_start_node=addNodeWithConnections(start_point,true);

  m_end_nodes.resize(0);

  std::shared_ptr<Node> actual_node=m_start_node;

  for (const std::vector<double>& end_point: end_points)
  {
    assert(m_dof==end_point.size());
    std::vector<unsigned int> path;
    double m_square_length=0;
    for (size_t idx=0;idx<start_point.size();idx++)
      m_square_length+=std::pow((start_point.at(idx)-end_point.at(idx)),2.0);

    unsigned int n_pnts=std::ceil(m_square_length/m_max_square_length);

    bool valid_path=true;

    for (unsigned int ipnt=0;ipnt<=n_pnts;ipnt++)
    {
      std::vector<double> q(start_point.size());
      for (size_t idx=0;idx<start_point.size();idx++)
      {
        q.at(idx)=start_point.at(idx)+(end_point.at(idx)-start_point.at(idx))*((double)ipnt)/((double)n_pnts);
      }

      std::shared_ptr<Node> new_node=addNodeWithConnections(q,true);

      bool found;
      for (unsigned int iconn=0;iconn<actual_node->m_connections.size();iconn++)
      {
        if (actual_node->m_connections.at(iconn)->getOtherNode(actual_node)==new_node)
        {
          path.push_back(iconn);
          found=true;
          break;
        }
      }
      valid_path=valid_path && found;

      actual_node=new_node;

      if (ipnt==n_pnts)
      {
        m_end_nodes.push_back(new_node);


        double cost=computePathCost(path);
        if (cost<m_best_cost)
        {
          if (checkCollision(path))
          {
            m_best_cost=cost;
            m_best_path=path;
            ROS_FATAL("this path improves the cost %f",cost);
          }
        }

      }



    }
  }


}

std::shared_ptr<Node> Net::addNodeWithConnections(const std::vector<double>& q, const bool& force_add)
{
  std::shared_ptr<ha_planner::Node> node=std::make_shared<ha_planner::Node>(q,m_node_params,m_conn_params);

  unsigned int new_connections;
  node->createConnections(m_max_square_length,
                          m_min_square_length,
                          m_max_connections_per_node,
                          m_nodes,
                          new_connections,
                          force_add);

  if (new_connections>0 || m_nodes.size()==0 || force_add)
  {
    m_nodes.push_back(node);
    return node;
  }
  return NULL;
}

bool Net::sendAnt(std::vector<unsigned int>& path)
{
  path.resize(0);
  std::shared_ptr<Node> actual_node=m_start_node;

  std::vector<Node*> last_nodes_ptr;

  for (unsigned idx=0;idx<1000;idx++)
  {
    last_nodes_ptr.push_back(actual_node.get());
    if (!actual_node)
      return false;
    for (const std::shared_ptr<Node> end_node: m_end_nodes)
    {
      if (actual_node==end_node)
      {
        ROS_DEBUG("Reach final state");
        return true;
      }
    }

    int next_conn;
    std::vector<Node*>::iterator it;
    unsigned int trial=0;
    do
    {

      next_conn=actual_node->rouletteWheel();
      if (next_conn<0)
      {
        ROS_DEBUG("node without connection");
        return false;
      }
      Node* next_ptr=actual_node->m_connections.at(next_conn)->getOtherNode(actual_node).get();

      it = std::find (last_nodes_ptr.begin(), last_nodes_ptr.end(), next_ptr);
    }
    while ((it != last_nodes_ptr.end()) && (++trial<actual_node->m_connections.size()));
    if (trial>=actual_node->m_connections.size())
    {
      ROS_DEBUG("circular path");
      return false;
    }
    path.push_back(next_conn);
    actual_node=actual_node->m_connections.at(path.back())->getOtherNode(actual_node);
  }
  ROS_DEBUG("unable to find solution");
  return false;
}

double Net::computePathCost(const std::vector<unsigned int> &path)
{
  std::shared_ptr<Node> actual_node=m_start_node;
  double cost=0;
  for (size_t idx=0;idx<path.size();idx++)
  {
    if (path.at(idx)>=actual_node->m_connections.size())
    {
      ROS_FATAL("invalid path (try to access to connection %u, while connection number is %zu",path.at(idx),actual_node->m_connections.size());
      return std::numeric_limits<double>::infinity();
    }
    assert(path.at(idx)<actual_node->m_connections.size());
    std::shared_ptr<Connection>& conn=actual_node->m_connections.at(path.at(idx));
    cost+=std::sqrt(conn->getLength());
    actual_node=conn->getOtherNode(actual_node);
  }
  return cost;
}

void Net::updateNodeHeuristic()
{
  for (std::shared_ptr<Node>& node: m_nodes)
    node->computeHeuristic(m_end_nodes);
}

void Net::evaporatePheromone()
{
  for (std::shared_ptr<Node>& node: m_nodes)
    for (std::shared_ptr<Connection> conn: node->m_connections)
      conn->updatePheromone(std::max(conn->getPheromone()*m_sqrt_evaporation_ratio,m_min_pheromone));
}

void Net::distributePheromone(const double &gain)
{
  std::shared_ptr<Node> actual_node=m_start_node;
  for (size_t idx=0;idx<m_best_path.size();idx++)
  {
//    ROS_INFO("connection %zu",idx);
    std::shared_ptr<Connection>& conn=actual_node->m_connections.at(m_best_path.at(idx));
    conn->updatePheromone(std::min(conn->getPheromone()+gain,m_max_pheromone));
    actual_node=conn->getOtherNode(actual_node);
  }
}

// run *n_ants* and check if an ant improve the best one.
bool Net::runAntCycle(const unsigned int &n_ants)
{

  std::map<double,std::vector<unsigned int>> best_ants; // automatically sorted

  for (unsigned int idx=0;idx<n_ants;idx++)
  {
    std::vector<unsigned int> path;
    if (sendAnt(path))
    {
      double cost=computePathCost(path);
      if (cost<m_best_cost)
      {
        best_ants.insert(std::pair<double,std::vector<unsigned int>>(cost,path));
      }
    }
  }

  if (best_ants.size()==0)
    return false;

  for (const std::pair<double,std::vector<unsigned int>>& candidate: best_ants)
  {
    if (checkCollision(candidate.second))
    {
      ROS_FATAL("this ant improves the cost %f",candidate.first);
      m_best_cost=candidate.first;
      m_best_path=candidate.second;


      return true;
    }
  }

  return false;
}

bool Net::checkCollision(const std::vector<unsigned int> &path)
{
  std::shared_ptr<Node> actual_node=m_start_node;
  for (size_t idx=0;idx<path.size();idx++)
  {
    std::shared_ptr<Connection>& conn=actual_node->m_connections.at(path.at(idx));

    if (conn->isInCollision(m_planning_scene))
    {
      ROS_FATAL("invalid path");
      return false;
    }
    actual_node=conn->getOtherNode(actual_node);
  }


  return true;
}

void Net::printBestPath()
{
  ROS_INFO("best path:\ncost:=%f\nnumber_of_connections: %zu\npoints:\n",m_best_cost,m_best_path.size());
  std::shared_ptr<Node> actual_node=m_start_node;
  for (size_t idx=0;idx<m_best_path.size();idx++)
  {
    actual_node->print();
    std::shared_ptr<Connection>& conn=actual_node->m_connections.at(m_best_path.at(idx));
    actual_node=conn->getOtherNode(actual_node);
  }
  actual_node->print();
}

}

