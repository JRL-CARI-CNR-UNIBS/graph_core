#include <net_path_core/net_path_core.h>

namespace ha_planner {


Eigen::MatrixXd computeRotationMatrix(const std::vector<double>& p1, const std::vector<double>& p2)
{
  assert(p1.size()==p2.size());
  unsigned int dof=p1.size();
  Eigen::MatrixXd rot_matrix(dof,dof);
  rot_matrix.setIdentity();
  const Eigen::Map<const Eigen::VectorXd> x1(p1.data(),dof);
  const Eigen::Map<const Eigen::VectorXd> x2(p2.data(),dof);
  Eigen::VectorXd main_versor =(x1-x2)/(x1-x2).norm();

  bool is_standard_base=false;
  for (unsigned int ic=0;ic<rot_matrix.cols();ic++)
  {
    if (std::abs(main_versor.dot(rot_matrix.col(ic)))>0.999)
    {
      is_standard_base=true;
      // rot_matrix is already orthonormal, put this direction as first
      Eigen::VectorXd tmp=rot_matrix.col(ic);
      rot_matrix.col(ic)=rot_matrix.col(0);
      rot_matrix.col(0)=tmp;
      break;
    }
  }

  if (!is_standard_base)
  {
    rot_matrix.col(0)=main_versor;
    // orthonormalization
    for (unsigned int ic=1;ic<rot_matrix.cols();ic++)
    {
      for (unsigned int il=0;il<ic;il++)
      {
        rot_matrix.col(ic)-= (rot_matrix.col(ic).dot(rot_matrix.col(il)))*rot_matrix.col(il);
      }
      rot_matrix.col(ic)/=rot_matrix.col(ic).norm();
    }
  }
  return rot_matrix;
}


Eigen::VectorXd computeEllipsoid(const std::vector<double> &p1,
                                 const std::vector<double> &p2,
                                 const double &cost)
{
  //  rot_matrix*semiaxes.asDiagonalMatrix()*ball+xcenter
  //  ball is a set of random number in a ball
  //  xcenter is the mean point betwen p1 and p2
  //  semiaxes is the vector of ellipsoid semiaxes
  //  rot_matrix is a n-dimensional rotational matrix orientated as the ellipsoid

  double min_dist=std::sqrt(squareDistance(p1,p2));
  double major_semiaxis;
  double minor_semiaxis;

  if (cost<std::numeric_limits<double>::infinity())
  {
    major_semiaxis=cost*0.5;
    minor_semiaxis=std::sqrt(std::pow(cost,2.0)-std::pow(min_dist,2.0))*0.5;
  }
  else
  {
    major_semiaxis=min_dist*40;
    minor_semiaxis=min_dist*40;
  }

  Eigen::VectorXd semiaxes(p1.size());
  semiaxes.setConstant(minor_semiaxis);
  semiaxes(0)=major_semiaxis;

  return semiaxes;
}


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

std::vector<std::vector<double>> intermediatePoints(const std::vector<double> &q1, const std::vector<double> &q2, const double& distance_step)
{
  assert(distance_step>0);
  std::vector<std::vector<double>> output;
  double distance=std::sqrt(squareDistance(q1,q2));
  if (distance<=distance_step)
    return output;
  unsigned int npnt=std::ceil(distance/distance_step);

  for (unsigned int ipnt=1;ipnt<npnt;ipnt++)
  {
    std::vector<double> q(q1.size());
    for (size_t idx=0;idx<q1.size();idx++)
      q.at(idx)=q1.at(idx)+(q2.at(idx)-q1.at(idx))*((double)ipnt)/((double)npnt);

    output.push_back(q);
  }
  return output;
}

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
    ConnectionPtr connection=std::make_shared<Connection>(pointer(),node,m_conn_params);
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

int Node::rouletteWheel()
{
  if (m_connections.size()==0)
    return -1;

  std::vector<double> p(m_connections.size(),0);
  p.at(0)=( m_connections.at(0)->getHeuristic()+m_connections.at(0)->getOtherNode(pointer())->getHeuristic())*m_connections.at(0)->getPheromone();

  for (unsigned int idx=1;idx<p.size();idx++)
    p.at(idx)=p.at(idx-1)+(m_connections.at(idx)->getHeuristic()+m_connections.at(idx)->getOtherNode(pointer())->getHeuristic())*m_connections.at(idx)->getPheromone();

  double random_value =  ((double)std::rand())/((double)RAND_MAX)*p.back();
  unsigned int idx=0;
  for (idx=0;idx<p.size();idx++)
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
  m_node2->print();

}

Net::Net(const unsigned int& dof, const planning_scene::PlanningSceneConstPtr& planning_scene):
  m_planning_scene(planning_scene)
{
  m_dof=dof;
  m_lb.resize(m_dof,-M_PI);
  m_ub.resize(m_dof,M_PI);

  m_min_square_length=0.02;
  m_max_square_length=3;
  m_grid_global_nodes=0;

  m_sqrt_evaporation_ratio=std::sqrt(0.93);
  m_max_pheromone=2;
  m_min_pheromone=0.4;
  m_node_params.group_name="ur10";
  m_conn_params.group_name=m_node_params.group_name;
  m_conn_params.checking_collision_distance=0.02;

  m_best_cost = std::numeric_limits<double>::infinity();
  m_best_path.resize(0);


}

void Net::generateNodesFromGrid(const unsigned int &number_of_nodes, const std::vector<double> &lower_bound, const std::vector<double> &upper_bound)
{
  assert(upper_bound.size()==m_dof);
  assert(lower_bound.size()==m_dof);
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

    addNodeWithConnections(q,true);

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

void Net::generateRandomNodesFromGrid(const unsigned int &number_of_nodes, const std::vector<double> &lower_bound, const std::vector<double> &upper_bound)
{
  std::random_device rd{};
  std::mt19937 gen{rd()};
  std::normal_distribution<double> nd(0.0,1.0);
  std::uniform_real_distribution<double> ud(0,1);
  size_t init_n_nodes=m_nodes.size();

  //  for (unsigned int in=0;in<number_of_nodes;in++)
  while (m_nodes.size()<=(init_n_nodes+number_of_nodes))
  {
    std::vector<double> q(m_dof);
    for (unsigned int idof=0;idof<m_dof;idof++)
      q.at(idof)=m_lb.at(idof)+(m_ub.at(idof)-m_lb.at(idof))*ud(gen);
    addNodeWithConnections(q);
  }
  size_t final_n_nodes=m_nodes.size();
  ROS_FATAL("add %zu nodes",final_n_nodes-init_n_nodes);

}

void Net::generateNodesFromStartAndEndPoints(const std::vector<double> &start_point, const std::vector<std::vector<double> > &end_points)
{
  assert(m_dof==start_point.size());

  m_start_node=addNodeWithConnections(start_point,true);

  m_end_nodes.resize(0);
  m_best_path_per_goal.clear();
  m_rot_matrix.resize(0);

  NodePtr actual_node=m_start_node;

  for (const std::vector<double>& end_point: end_points)
  {
    NodePtr end_node=std::make_shared<Node>(end_point,m_node_params,m_conn_params);
    if (end_node->isInCollision(m_planning_scene))
    {
      ROS_WARN("this end point is in collision, skipping it");
      end_node->print();
      continue;
    }
    NodePtr actual_node=m_start_node;
    m_rot_matrix.push_back(computeRotationMatrix(m_start_node->getJoints(),end_point));

    assert(m_dof==end_point.size());
    Path path;
    double m_square_length=0;
    for (size_t idx=0;idx<start_point.size();idx++)
      m_square_length+=std::pow((start_point.at(idx)-end_point.at(idx)),2.0);

    unsigned int n_pnts=std::ceil(m_square_length/m_max_square_length);


    for (unsigned int ipnt=1;ipnt<=n_pnts;ipnt++)
    {
      std::vector<double> q(start_point.size());
      for (size_t idx=0;idx<start_point.size();idx++)
      {
        q.at(idx)=start_point.at(idx)+(end_point.at(idx)-start_point.at(idx))*((double)ipnt)/((double)n_pnts);
      }

      NodePtr new_node=addNodeWithConnections(q,true);
      if (!new_node)
        throw std::out_of_range("error creating node");

      bool found;
      for (ConnectionPtr conn: actual_node->m_connections)
      {
        if (conn->getOtherNode(actual_node)==new_node)
        {
          path.push_back(conn);
          found=true;
          break;
        }
      }
      if (!found)
      {
        ConnectionPtr conn=std::make_shared<Connection>(actual_node,new_node,m_conn_params);
        path.push_back(conn);

      }
      actual_node=new_node;
      if (ipnt==n_pnts)
      {
        m_end_nodes.push_back(new_node);


        double cost=computePathCost(path);
        if (!checkCollision(path))
          cost=std::numeric_limits<double>::infinity();

        std::pair<double,Path> tuple(cost,path);
        std::pair<NodePtr,std::pair<double,Path>> entry(new_node,tuple);
        m_best_path_per_goal.insert(entry);
        ROS_FATAL("is reached in %zu step",path.size());
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

NodePtr Net::addNodeWithConnections(const std::vector<double>& q, const bool& force_add)
{
  std::shared_ptr<ha_planner::Node> node=std::make_shared<ha_planner::Node>(q,m_node_params,m_conn_params);

  unsigned int new_connections;
  node=node->createConnections(m_max_square_length,
                               m_min_square_length,
                               m_nodes,
                               new_connections);

  if (new_connections>0 || m_nodes.size()==0 || force_add)
  {
    node->computeHeuristic(m_end_nodes);
    m_nodes.push_back(node);
    return node;
  }
  return NULL;
}

// return true if all the nodes to q are added, false if there was a collision in the middle
bool Net::addNodeToTheClosestNodeOfTheTree(const std::vector<double> &q, std::vector<NodePtr> &tree)
{
  double min_distance=std::numeric_limits<double>::infinity();
  NodePtr closed_node;
  for (const NodePtr& node: tree)
  {
    double dist=squareDistance(q,node->getJoints());
    if (dist<min_distance)
    {
      closed_node=node;
      min_distance=dist;
    }
  }

  const std::vector<double>& start_point=closed_node->getJoints();
  unsigned int n_pnts=std::ceil(min_distance/m_max_square_length);

  NodePtr actual_node=closed_node;

  for (unsigned int ipnt=1;ipnt<=n_pnts;ipnt++)
  {
    std::vector<double> qstep(q.size());
    for (size_t idx=0;idx<start_point.size();idx++)
    {
      qstep.at(idx)=start_point.at(idx)+(q.at(idx)-start_point.at(idx))*((double)ipnt)/((double)n_pnts);
    }

    NodePtr new_node=std::make_shared<Node>(qstep,m_node_params,m_conn_params);
    if (new_node->isInCollision(m_planning_scene))
      return false;

    ConnectionPtr new_conn=std::make_shared<Connection>(actual_node,new_node);
    if (new_conn->isInCollision(m_planning_scene))
      return false;

    m_nodes.push_back(new_node);
    new_node->addConnection(new_conn);
    actual_node->addConnection(new_conn);
    tree.push_back(new_node);
    actual_node=new_node;
  }
  return true;

}

bool Net::sendAnt(Path& path)
{
  path.resize(0);
  NodePtr actual_node=m_start_node;

  for (unsigned idx=0;idx<500;idx++)
  {
    if (!actual_node)
      return false;
    for (const NodePtr end_node: m_end_nodes)
    {
      if (actual_node==end_node)
      {
        ROS_DEBUG("Reach final state");
        return true;
      }
    }

    int next_conn;

    next_conn=actual_node->rouletteWheel();
    if (next_conn<0)
    {
      ROS_DEBUG("node without connection");
      return false;
    }
    path.push_back(actual_node->m_connections.at(next_conn));
    actual_node=path.back()->getOtherNode(actual_node);
  }
  ROS_DEBUG("unable to find solution");
  return false;
}

double Net::computePathCost(const Path &path)
{
  NodePtr actual_node=m_start_node;
  double cost=0;
  for (const ConnectionPtr& conn: path)
  {
    cost+=std::sqrt(conn->getLength());
    actual_node=conn->getOtherNode(actual_node);
  }
  return cost;
}

void Net::updateNodeHeuristic()
{
  for (NodePtr& node: m_nodes)
    node->computeHeuristic(m_end_nodes);
}

void Net::evaporatePheromone()
{
  for (NodePtr& node: m_nodes)
    for (ConnectionPtr conn: node->m_connections)
      conn->updatePheromone(std::max(conn->getPheromone()*m_sqrt_evaporation_ratio,m_min_pheromone));
}

void Net::distributePheromone(const double &gain)
{
  for (size_t idx=0;idx<m_best_path.size();idx++)
  {
    //    ROS_INFO("connection %zu",idx);
    ConnectionPtr& conn=m_best_path.at(idx);
    conn->updatePheromone(std::min(conn->getPheromone()+gain,m_max_pheromone));
  }
  for (const NodePtr& end_node: m_end_nodes)
  {
    Path& path=m_best_path_per_goal.at(end_node).second;
    for (size_t idx=0;idx<path.size();idx++)
    {
      //    ROS_INFO("connection %zu",idx);
      ConnectionPtr& conn=path.at(idx);
      conn->updatePheromone(std::min(conn->getPheromone()+gain,m_max_pheromone));
    }
  }

}

// run *n_ants* and check if an ant improve the best one.
bool Net::runAntCycle(const unsigned int &n_ants)
{

  std::map<NodePtr,std::map<double,Path>> best_ants_per_each_goal; // automatically sorted

  for (unsigned int idx=0;idx<n_ants;idx++)
  {
    Path path;
    if (sendAnt(path))
    {
      double cost=computePathCost(path);

      for (const NodePtr& end_node: m_end_nodes)
      {
        if (path.back()->isMember(end_node))
        {
          if (cost < m_best_path_per_goal.at(end_node).first)
          {
            std::pair<double,Path> pair(cost,path);
            if (best_ants_per_each_goal.count(end_node)>0)
              best_ants_per_each_goal.at(end_node).insert(pair);
            else
            {
              std::map<double,Path> mapp;
              mapp.insert(std::pair<double,Path>(cost,path));
              best_ants_per_each_goal.insert(std::pair<NodePtr,std::map<double,Path>>(end_node,mapp));
            }
          }
        }
      }

    }

  }

  for (const NodePtr& end_node: m_end_nodes)
  {
    if (best_ants_per_each_goal.count(end_node)==0)
      continue;
    std::map<double,Path>& best_ants=best_ants_per_each_goal.at(end_node); // automatically sorted
    for (const std::pair<double,Path>& candidate: best_ants)
    {
      if (checkCollision(candidate.second))
      {
        if (candidate.first<m_best_cost)
        {
          ROS_FATAL("this ant improves the cost %f",candidate.first);
          m_best_cost=candidate.first;
          m_best_path=candidate.second;
        }
        if (candidate.first<m_best_path_per_goal.at(end_node).first)
        {
          ROS_FATAL("this ant improves the cost (%f) for the specific goal",candidate.first);
          m_best_path_per_goal.at(end_node)=candidate;
        }
        return true;
      }
    }
  }
  return false;
}

bool Net::checkCollision(const Path& path)
{
  NodePtr actual_node=m_start_node;
  for (const ConnectionPtr& conn: path)
  {
    if (conn->isInCollision(m_planning_scene))
    {
      return false;
    }
    actual_node=conn->getOtherNode(actual_node);
  }


  return true;
}

void Net::printBestPath()
{
  ROS_INFO("best path:\ncost:=%f",m_best_cost);
  printPath(m_best_path);

  ROS_INFO("best path for each goal");
  for (const std::pair<NodePtr,std::pair<double,Path>>& el: m_best_path_per_goal)
  {
    ROS_INFO("\ncost: %f", el.second.first);
    printPath(el.second.second);

  }

}

void Net::printPath(const Path &path)
{
  ROS_INFO("path:\nnumber_of_connections: %zu\npoints:",path.size());
  NodePtr actual_node=m_start_node;
  for (size_t idx=0;idx<path.size();idx++)
  {
    actual_node->print();
    actual_node=path.at(idx)->getOtherNode(actual_node);
  }
  actual_node->print();
}

void Net::generateNodesFromEllipsoid(const unsigned int &number_of_nodes)
{
  if (m_best_cost==std::numeric_limits<double>::infinity())
  {
    generateRandomNodesFromGrid(number_of_nodes,m_lb,m_ub);
    return;
  }

  double sum_of_inv_costs=0;
  for (unsigned int ien=0;ien<m_end_nodes.size();ien++)
    sum_of_inv_costs+=1.0/std::min(std::min(m_best_cost,100.0)*5,m_best_path_per_goal.at(m_end_nodes.at(ien)).first);


  std::random_device rd{};
  std::mt19937 gen{rd()};
  std::normal_distribution<double> nd(0.0,1.0);
  std::uniform_real_distribution<double> ud(0,1);
  for  (unsigned int ien=0;ien<m_end_nodes.size();ien++)
  {

    Eigen::VectorXd semiaxes=computeEllipsoid(m_start_node->getJoints(),
                                              m_end_nodes.at(ien)->getJoints(),
                                              m_best_path_per_goal.at(m_end_nodes.at(ien)).first);
    Eigen::Map<const Eigen::VectorXd> p1(m_start_node->getJoints().data(),m_dof);
    Eigen::Map<const Eigen::VectorXd> p2(m_end_nodes.at(ien)->getJoints().data(),m_dof);
    Eigen::VectorXd center=(p1+p2)/2.0;

    unsigned int np=number_of_nodes*1.0/std::min(std::min(m_best_cost,100.0)*5,m_best_path_per_goal.at(m_end_nodes.at(ien)).first)/sum_of_inv_costs;
    unsigned int ip=0;
    unsigned int itrial=0;
    while ((ip<np) && (itrial<10*np))
    {
      itrial++;
      std::vector<double> p(m_dof,0);
      for (double& el: p)
        el=nd(gen);

      Eigen::Map<Eigen::VectorXd> vet(p.data(),m_dof);
      vet*=ud(gen)/vet.norm();

      std::vector<double> joint(m_dof,0);
      Eigen::VectorXd::Map(joint.data(),m_dof)=m_rot_matrix.at(ien)*semiaxes.asDiagonal()*vet+center;

      bool out_of_bounds=false;
      for (unsigned int iax=0;iax<m_dof;iax++)
      {
        if ((joint.at(iax)<m_lb.at(iax)) || joint.at(iax)>m_ub.at(iax))
          out_of_bounds=true;
      }
      if (out_of_bounds)
        continue;
      addNodeWithConnections(joint);
      ip++;

    }
  }
}


void Net::print()
{
  ROS_INFO("net of %zu nodes", m_nodes.size());
  for (unsigned int ir=0;ir<m_nodes.size();ir++)
  {
    for (unsigned int ic=0;ic<m_nodes.size();ic++)
    {
      bool is_starting=false;
      bool is_ending=false;
      bool is_in_collision=false;

      if (m_nodes.at(ir)==m_nodes.at(ic))
      {
        if (m_nodes.at(ir)==m_start_node)
          is_starting=true;

        for (const NodePtr& ep: m_end_nodes)
        {
          if (m_nodes.at(ir)==ep)
            is_ending=true;
        }
      }

      bool found=false;
      {
        for (const ConnectionPtr& conn: m_nodes.at(ir)->m_connections)
          if (conn->getOtherNode(m_nodes.at(ir))==m_nodes.at(ic))
          {
            found=true;
            is_in_collision=conn->isInCollision(m_planning_scene);
            break;
          }
      }

      if (is_starting)
        std::cout << "S";
      else if (is_ending)
        std::cout << "E";
      else if (found && !is_in_collision)
        std::cout << "*";
      else
        std::cout << " ";
    }
    std::cout << std::endl;
  }
}

unsigned int Net::removeUnconnectedNodes()
{
  unsigned int num_of_removed_nodes=0;
  for (unsigned int in=0;in<m_nodes.size();in++)
  {
    if (m_nodes.at(in)->isUnconnected(m_planning_scene))
    {
      removeNodeWithConnections(m_nodes.at(in));
      num_of_removed_nodes++;
    }
  }
  return num_of_removed_nodes;
}

void Net::removeNodeWithConnections(NodePtr &node)
{
  for (ConnectionPtr conn: node->m_connections)
  {
    conn->getOtherNode(node)->removeConnection(conn);
  }
  node->m_connections.clear();

  std::vector<NodePtr>::iterator el=std::find(m_nodes.begin(),m_nodes.end(),node);
  if (el!=m_nodes.end())
    m_nodes.erase(el);
}

unsigned int Net::removeLowPheromoneConnections(const unsigned int &number_of_nodes_to_check)
{
  std::random_device rd{};
  std::mt19937 gen{rd()};
  std::uniform_int_distribution<unsigned int> dis(0,m_nodes.size()-1);
  std::vector<unsigned int> elements_to_check;

  unsigned int removed_connections=0;
  while (elements_to_check.size()<std::min(number_of_nodes_to_check,(unsigned int)m_nodes.size()))
  {
    unsigned int el=dis(gen);
    std::vector<unsigned int>::iterator it=std::find(elements_to_check.begin(),elements_to_check.end(),el);
    if (it==elements_to_check.end())
      elements_to_check.push_back(el);
  }

  for (const unsigned int el: elements_to_check)
  {
    std::vector<ConnectionPtr> conn_to_be_removed;
    for (ConnectionPtr& conn: m_nodes.at(el)->m_connections)
    {
      if (conn->getPheromone()==m_min_pheromone)
      {
        //        if (conn->isCollisionChecked())
        //          if (!conn->isInCollision(m_planning_scene))
        //            continue;
        bool is_in_some_best_path=false;
        for (const std::pair<NodePtr,std::pair<double,Path>>& path: m_best_path_per_goal)
        {
          if (std::find(path.second.second.begin(),path.second.second.end(),conn)!=path.second.second.end())
          {
            is_in_some_best_path=true;
            break;
          }
        }
        if (!is_in_some_best_path)
        {
          conn_to_be_removed.push_back(conn);
        }
      }

    }

    for (ConnectionPtr& conn: conn_to_be_removed)
    {
      NodePtr node1;
      NodePtr node2;
      conn->getNodes(node1,node2);
      node1->removeConnection(conn);
      node2->removeConnection(conn);
      removed_connections++;
    }

  }
  return removed_connections;
}

std::vector<std::vector<double>> Net::getPath(const Path &path)
{
  std::vector<std::vector<double>> waypoints;
  NodePtr actual_node=m_start_node;
  for (size_t idx=0;idx<path.size();idx++)
  {
    waypoints.push_back(actual_node->getJoints());
    actual_node=path.at(idx)->getOtherNode(actual_node);
  }
  waypoints.push_back(actual_node->getJoints());
  return waypoints;
}

std::vector<std::vector<double>> Net::getBestPath()
{
  return getPath(m_best_path);
}

bool Net::rrtConnectSingleGoal(const NodePtr &node)
{
  std::vector<NodePtr> start_tree;
  std::vector<NodePtr> goal_tree;


  std::random_device rd{};
  std::mt19937 gen{rd()};
  std::normal_distribution<double> nd(0.0,1.0);
  std::uniform_real_distribution<double> ud(0,1);

  bool success=false;
  bool fail=false;
  while (!success && !fail)
  {

    unsigned int itrial=0;
    while (!fail)
    {

      if (++itrial>10)
        fail=true;

      std::vector<double> joint(m_dof,0);
      for (double& el: joint)
        el=nd(gen);

      bool out_of_bounds=false;
      for (unsigned int iax=0;iax<m_dof;iax++)
      {
        if ((joint.at(iax)<m_lb.at(iax)) || joint.at(iax)>m_ub.at(iax))
          out_of_bounds=true;
      }
      if (out_of_bounds)
        continue;

      bool full_added_to_start_tree=addNodeToTheClosestNodeOfTheTree(joint,start_tree);
      bool full_added_to_stop_tree=addNodeToTheClosestNodeOfTheTree(joint,goal_tree);

      if (full_added_to_start_tree && full_added_to_stop_tree)
      {
        solution found, need to attach tree (implement node->findFather).
        joint node is added twist, needs to remove one of the
      }
      add  failing condition
    }
  }

  return success;
}


}

