#include <net_path_core/net_path_core.h>

namespace ha_planner {


Net::Net(const unsigned int& dof, const planning_scene::PlanningSceneConstPtr& planning_scene):
  m_planning_scene(planning_scene)
{
  m_dof=dof;
  m_lb.resize(m_dof,-M_PI);
  m_ub.resize(m_dof,M_PI);

  m_min_square_length=0.05;
  m_max_square_length=10;
  m_grid_global_nodes=0;

//  m_sqrt_evaporation_ratio=std::sqrt(0.93);
  m_sqrt_evaporation_ratio=std::sqrt(0.8);
  m_max_pheromone=2;
  m_min_pheromone=0.4;
  m_node_params.group_name="ur10";
  m_conn_params.group_name=m_node_params.group_name;
  m_conn_params.checking_collision_distance=0.05;

  m_best_cost = std::numeric_limits<double>::infinity();
  m_best_path.resize(0);


}

void Net::generateNodesFromGrid(const int &number_of_nodes, const std::vector<double> &lower_bound, const std::vector<double> &upper_bound)
{
  if (number_of_nodes<=0)
    return;

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

void Net::generateRandomNodesFromGrid(const int &number_of_nodes, const std::vector<double> &lower_bound, const std::vector<double> &upper_bound)
{
  if (number_of_nodes<0)
    return;
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

    ConnectionPtr new_conn=std::make_shared<Connection>(actual_node,new_node,m_conn_params);
    if (new_conn->isInCollision(m_planning_scene))
      return false;

//  DOPO  m_nodes.push_back(new_node);
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

    Direction dir=Forward;
    next_conn=actual_node->rouletteWheel(dir);
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
//      ROS_FATAL("--candidate cost=%f, best end goal=%f, global best=%f",candidate.first,m_best_path_per_goal.at(end_node).first,m_best_cost);
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

void Net::generateNodesFromEllipsoid(const int &number_of_nodes)
{
  if (number_of_nodes<=0)
    return;

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

const Path& Net::getBestPathRef(){return m_best_path;}

std::vector<std::vector<double>> Net::getBestPath()
{
  return getPath(m_best_path);
}

bool Net::rrtConnectSingleGoal(const NodePtr &node)
{
  std::vector<NodePtr> start_tree;
  std::vector<NodePtr> goal_tree;

  start_tree.push_back(m_start_node);
  goal_tree.push_back(node);

  std::random_device rd{};
  std::mt19937 gen{rd()};
  std::uniform_real_distribution<double> ud(0,1);

  bool success=false;
  bool fail=false;
  unsigned int iter=0;
  while (!success && !fail)
  {
    ROS_DEBUG("Iteration %u",iter);
    if (iter++>100)
      fail=true;

    std::vector<double> joint(m_dof,0);
    for (unsigned int idof=0;idof<m_dof;idof++)
      joint.at(idof)=m_lb.at(idof)+(m_ub.at(idof)-m_lb.at(idof))*ud(gen);

    ROS_DEBUG("adding node to trees");
    bool full_added_to_start_tree=addNodeToTheClosestNodeOfTheTree(joint,start_tree);
    ROS_DEBUG("full add to start tree? %s",full_added_to_start_tree? "yes" : "no");
    bool full_added_to_stop_tree=addNodeToTheClosestNodeOfTheTree(joint,goal_tree);
    ROS_DEBUG("full add to stop tree? %s",full_added_to_stop_tree? "yes" : "no");


    if (full_added_to_start_tree && full_added_to_stop_tree)
    {
      ROS_DEBUG("removing duplicated node from goal tree");
      removeNodeWithConnections(goal_tree.back());
      goal_tree.pop_back();

      ROS_DEBUG("creating connection between the two trees");
      ConnectionPtr new_conn=std::make_shared<Connection>(start_tree.back(),goal_tree.back(),m_conn_params);
      if (new_conn->isInCollision(m_planning_scene))
      {
        ROS_ERROR("this connection should be ok, something strange");
        return false;
      }
      start_tree.back()->addConnection(new_conn);
      goal_tree.back()->addConnection(new_conn);

      Path path;

      ROS_DEBUG("insert in the path the branch of the start_tree");
      ConnectionPtr actual_connection=new_conn;
      do
      {
        path.insert(path.begin(),actual_connection);
        actual_connection=actual_connection->getParent()->findConnectionToParent();
      }
      while (actual_connection->getParent()!=m_start_node);
      path.insert(path.begin(),actual_connection);

      ROS_DEBUG("insert in the path the branch of the goal_tree");
      actual_connection=goal_tree.back()->findConnectionToParent();
      do
      {
        path.push_back(actual_connection);
        actual_connection=actual_connection->getParent()->findConnectionToParent();
        path.back()->flipDirection();
      }
      while (actual_connection->getParent()!=node);
      path.push_back(actual_connection);
      path.back()->flipDirection();

      path=simplifyIntermidiatePoint(path);

      m_nodes.push_back(path.front()->getParent());
      for (const ConnectionPtr& conn: path)
        m_nodes.push_back(conn->getChild());

      double cost=computePathCost(path);
      ROS_DEBUG("find solution with cost=%f",cost);
      if (cost < m_best_path_per_goal.at(node).first)
      {
        m_best_path_per_goal.at(node).first=cost;
        m_best_path_per_goal.at(node).second=path;

        if (cost<m_best_cost)
        {
          ROS_INFO("RRTConnect improved the overall best path (cost %f)",cost);
          m_best_cost=cost;
          m_best_path=path;
        }
        else
          ROS_INFO("RRTConnect improved the best path to goal (cost %f)",cost);

      }
      success=true;
    }
  }


  return success;
}

bool Net::rrtConnect()
{
  for (const NodePtr end_node: m_end_nodes)
    rrtConnectSingleGoal(end_node);
  return true;
}

bool Net::isSolutionFound()
{
  for (const NodePtr end_node: m_end_nodes)
  {
    if (m_best_path_per_goal.at(end_node).first>=std::numeric_limits<double>::infinity())
      return false;
  }
  return true;
}

Path Net::simplifyIntermidiatePoint(const Path& path)
{
  Path simplified_path;

  NodePtr parent=path.front()->getParent();

  for (unsigned int idx=1;idx<path.size();idx++)
  {
    if (path.at(idx-1)->dotProduct(path.at(idx))<0.9999)
    {
      NodePtr child=path.at(idx-1)->getChild();
      ConnectionPtr conn=std::make_shared<Connection>(parent,child,m_conn_params);
      conn->forceNotCollision();
      simplified_path.push_back(conn);
      parent->addConnection(conn);
      child->addConnection(conn);
      parent=child;
      conn->print();
    }

  }

  path.back()->getChild()->print();
  path.back()->getParent()->print();

  NodePtr child=path.back()->getChild();
  ConnectionPtr conn=std::make_shared<Connection>(parent,child,m_conn_params);
  conn->forceNotCollision();
  parent->addConnection(conn);
  child->addConnection(conn);
  conn->print();

  simplified_path.push_back(conn);

  return simplified_path;
}

}

