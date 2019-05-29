#include <net_path_core/net_path_core.h>

namespace ha_planner {


Net::Net(const unsigned int& dof, const std::string& group_name, const planning_scene::PlanningSceneConstPtr& planning_scene, const std::vector<double> scaling):
  m_planning_scene(planning_scene),
  m_gen(time(0))
{

  m_ud=std::uniform_real_distribution<double>(0,1);

  m_dof=dof;
  m_lb.resize(m_dof,-M_PI);
  m_ub.resize(m_dof,M_PI);

  m_min_length=0.05;
  m_max_length=1;

  m_min_square_length=std::pow(m_min_length,2);
  m_max_square_length=std::pow(m_max_length,2);


  //  m_sqrt_evaporation_ratio=std::sqrt(0.93);
  //  m_sqrt_evaporation_ratio=std::sqrt(0.8);
  m_sqrt_evaporation_ratio=std::sqrt(0.9);
  m_max_pheromone=2;
  m_min_pheromone=0.4;
  m_node_params.group_name=group_name;
  m_conn_params.group_name=m_node_params.group_name;
  m_conn_params.checking_collision_distance=0.06;
  m_conn_params.scaling=scaling;
  m_conn_params.unscaling.resize(scaling.size());
  for (unsigned int idx=0;idx<scaling.size();idx++)
  {
    if (scaling.at(idx)<=0)
    {
      ROS_ERROR("scaling should be positive");
      throw std::invalid_argument("scaling should be positive");
    }
    m_conn_params.unscaling.at(idx)=1.0/scaling.at(idx);
    ROS_FATAL("scaling joint %u by %f",idx,scaling.at(idx));
  }
  m_best_cost = std::numeric_limits<double>::infinity();
  m_best_path.resize(0);


}

Path Net::warpPath(const unsigned int& number_of_trials, const Path& path)
{
  double old_cost=computePathCost(path);
  double best_cost=old_cost;
  Path best_path=path;
  for (unsigned int i_trial=0;i_trial<number_of_trials;i_trial++)
  {

    Eigen::VectorXd perturbation(m_dof);

    for (unsigned int idx=0;idx<(path.size()-1);idx++)
    {
      Path new_path=path;

      Eigen::Map<const Eigen::VectorXd> p1(path.at(idx)->getParent()->getJoints().data(),m_dof);
      Eigen::Map<const Eigen::VectorXd> p3(path.at(idx+1)->getChild()->getJoints().data(),m_dof);
      Eigen::VectorXd v13=(p3-p1).normalized();

      Eigen::VectorXd v12;
      Eigen::VectorXd v23;
      Eigen::VectorXd prominence;


      Eigen::Map<const Eigen::VectorXd> p2(path.at(idx)->getChild()->getJoints().data(),m_dof);

      v12=path.at(idx)->versor();
      v23=path.at(idx+1)->versor();
      prominence=v12-(v12.dot(v13))*v13;

      if (prominence.norm()<0.001)
      {
        continue;
      }
      prominence=prominence.normalized();
      double distance=std::abs((p2-p1).dot(prominence));

      double max_distance=distance;
      double min_distance=0;

      NodePtr node=path.at(idx)->getChild();

      unsigned int iter=0;
      while(iter<100 && (max_distance-min_distance)>m_min_length)
      {
        perturbation=-distance*prominence;

        Eigen::VectorXd p2new=p2+perturbation;


        std::vector<double> p(m_dof,0);
        for (unsigned idof=0;idof<m_dof;idof++)
          p.at(idof)=p2new(idof);
        NodePtr new_node=addNodeToTheGrid(p,node);

        ConnectionPtr conn_12;
        if (!new_node->checkIfConnectedWith(path.at(idx)->getParent(),conn_12))
          ROS_ERROR("Thes nodes should be connected");
        ConnectionPtr conn_23;
        if (!new_node->checkIfConnectedWith(path.at(idx+1)->getChild(),conn_23))
          ROS_ERROR("Thes nodes should be connected");

        new_path.at(idx)=conn_12;
        new_path.at(idx+1)=conn_23;
        double cost=computePathCost(new_path);

        if (isCollisionFree(new_path))
          storeIfImproveCost(new_path,cost);
        else
          cost=std::numeric_limits<double>::infinity();

        if (cost<best_cost)
        {
          best_cost=cost;
          best_path=new_path;

          if (iter==0)
            break;

          min_distance=distance;
          distance=0.5*(max_distance+distance);
        }
        else
        {
          removeNodeWithConnections(new_node);
          max_distance=distance;
          distance=0.5*(min_distance+distance);
        }
        if (isWrong())
          ROS_ERROR("ehiehi");

      }
    }
    if (best_cost>=0.999*old_cost)
      break;
    else
      old_cost=best_cost;
  }

  return best_path;
}


bool Net::warpPath2(const unsigned int& number_of_trials)
{

  double old_cost=m_best_cost;
  warpPath(number_of_trials,m_best_path);
  return m_best_cost<old_cost;

}

Path Net::splitPath(const unsigned int& number_of_trials, const Path& path)
{
  bool changed=true;

  unsigned int itrial=0;
  unsigned int idx=0;

  double old_cost=computePathCost(path);
  double best_cost=old_cost;
  Path best_path=path;

  while(itrial<number_of_trials)
  {

    if (idx>=(path.size()-1))
    {
      if (best_cost>=0.99*old_cost)
      {
        return best_path;
      }
      idx=0;
      itrial++;
      old_cost=best_cost;
    }
    Eigen::Map<const Eigen::VectorXd> p1(path.at(idx)->getParent()->getJoints().data(),m_dof);
    Eigen::Map<const Eigen::VectorXd> p2(path.at(idx)->getChild()->getJoints().data(),m_dof);
    Eigen::Map<const Eigen::VectorXd> p3(path.at(idx+1)->getChild()->getJoints().data(),m_dof);

    Eigen::VectorXd v12=p2-p1;
    Eigen::VectorXd v23=p3-p2;

    NodePtr node=path.at(idx)->getChild();

    double alpha=0.5;
    double min_alpha=0;
    double max_alpha=1;
    double delta_alpha=m_min_length/std::min(p1.norm(),p2.norm());
    unsigned i_bis=0;
    while ( (i_bis++<100) && (max_alpha-min_alpha)>delta_alpha)
    {
      Path new_path=path;

      Eigen::VectorXd p2a=p1+v12*alpha;
      Eigen::VectorXd p2b=p3-v23*alpha;

      std::vector<std::vector<double>> points;
      std::vector<double> p(m_dof,0);
      for (unsigned idof=0;idof<m_dof;idof++)
        p.at(idof)=p2a(idof);

      points.push_back(p);
      for (unsigned idof=0;idof<m_dof;idof++)
        p.at(idof)=p2b(idof);
      points.push_back(p);

      Path subpath=addSubGridToTheGrid(points,path.at(idx)->getChild());


      ConnectionPtr conn_12;
      if (!subpath.front()->getParent()->checkIfConnectedWith(path.at(idx)->getParent(),conn_12))
      {
        ROS_ERROR("Thes nodes should be connected");
        throw std::invalid_argument("These path should be connected");
      }
      ConnectionPtr conn_23;
      if (!subpath.back()->getChild()->checkIfConnectedWith(path.at(idx+1)->getChild(),conn_23))
      {
        ROS_ERROR("These path should be connected");
        throw std::invalid_argument("These path should be connected");
      }

      new_path.at(idx)=conn_12;
      new_path.at(idx+1)=conn_23;

      new_path.insert(new_path.begin()+idx+1,subpath.begin(),subpath.end());

      double cost=computePathCost(new_path);

      if (isCollisionFree(new_path))
        storeIfImproveCost(new_path,cost);
      else
        cost=std::numeric_limits<double>::infinity();

      if (cost<best_cost)
      {
        best_cost=cost;
        best_path=new_path;
        if (alpha==max_alpha)
          break;
        min_alpha=alpha;
        alpha=0.5*(max_alpha+alpha);

      }
      else
      {
        std::vector<NodePtr> nodes;
        for (const ConnectionPtr& conn: subpath)
        {
          nodes.push_back(conn->getParent());
        }
        nodes.push_back(subpath.back()->getChild());

        for (NodePtr& node: nodes)
          removeNodeWithConnections(node);
        max_alpha=alpha;
        alpha=0.5*(min_alpha+alpha);
      }
      if (isWrong())
        ROS_ERROR("ehiehi");

    }

    idx+=1;
  }
  return best_path;
}

bool Net::splitPath2(const unsigned int& number_of_trials)
{
  double old_cost=m_best_cost;
  splitPath(number_of_trials,m_best_path);
  return m_best_cost<old_cost;
}


std::vector<double> Net::sample(const NodePtr& endnode)
{
  ROS_FATAL("QUI");
  std::vector<std::vector<double>> solutions=getSamples(endnode,1);
  ROS_FATAL("QUI");
  return solutions.at(0);
}

std::vector<std::vector<double>> Net::getSamples(const NodePtr& endnode, const unsigned int& number_of_samples)
{
  std::vector<std::vector<double>> samples;

  std::vector<double> joints(m_dof,0);
  Eigen::Map<Eigen::VectorXd> q(joints.data(),m_dof);


  if (m_best_cost<std::numeric_limits<double>::infinity())
  {
    Eigen::VectorXd semiaxes=computeEllipsoid(m_start_node->getJoints(),
                                              endnode->getJoints(),
                                              m_best_cost);
    Eigen::Map<const Eigen::VectorXd> p1(m_start_node->getJoints().data(),m_dof);
    Eigen::Map<const Eigen::VectorXd> p2(endnode->getJoints().data(),m_dof);
    Eigen::VectorXd center=(p1+p2)/2.0;

    Eigen::VectorXd ball(m_dof);

    for (unsigned int is=0;is<number_of_samples;is++)
    {
      bool out_of_bounds=true;
      while(out_of_bounds)
      {
        ROS_FATAL("QUI");
        ball.setRandom();
        ball*=std::pow(m_ud(m_gen),1.0/(double)m_dof)/ball.norm();
        q=m_rot_matrix.at(endnode)*semiaxes.asDiagonal()*ball+center;

        out_of_bounds=false;
        for (unsigned int iax=0;iax<m_dof;iax++)
          if ((joints.at(iax)<m_lb.at(iax)) || joints.at(iax)>m_ub.at(iax))
            out_of_bounds=true;
      }
      samples.push_back(joints);
      ROS_FATAL("QUI");
    }
  }
  else
  {
    ROS_FATAL("QUI");
    for (unsigned int is=0;is<number_of_samples;is++)
    {
      for (unsigned int idof=0;idof<m_dof;idof++)
        joints.at(idof)=m_lb.at(idof)+(m_ub.at(idof)-m_lb.at(idof))*m_ud(m_gen);
      samples.push_back(joints);
      ROS_FATAL("QUI");
    }
  }
  return samples;
}

void Net::generateNodesFromStartAndEndPoints(const std::vector<double> &start_point, const std::vector<std::vector<double> > &end_points)
{
  assert(m_dof==start_point.size());

  std::vector<double> scaled_start_point(m_dof);
  for (unsigned int idof=0;idof<m_dof;idof++)
    scaled_start_point.at(idof)=start_point.at(idof)*m_conn_params.scaling.at(idof);

  m_start_node=std::make_shared<Node>(scaled_start_point,m_node_params,m_conn_params);
  m_nodes.push_back(m_start_node);

  m_end_nodes.resize(0);
  m_best_path_per_goal.clear();
  m_rot_matrix.clear();

  NodePtr actual_node=m_start_node;

  m_start_tree =  std::make_shared<Tree>(m_start_node,
                                         m_node_params,
                                         m_conn_params,
                                         m_planning_scene,
                                         m_max_square_length,
                                         Direction::Forward,
                                         m_nodes,
                                         m_end_nodes);



  m_estimated_cost=0;
  for (const std::vector<double>& end_point: end_points)
  {
    assert(m_dof==end_point.size());


    std::vector<double> scaled_end_point(m_dof);
    for (unsigned int idof=0;idof<m_dof;idof++)
      scaled_end_point.at(idof)=end_point.at(idof)*m_conn_params.scaling.at(idof);

    m_estimated_cost=std::max(m_estimated_cost,1.5*sqrt(squareDistance(scaled_start_point,scaled_end_point)));
    NodePtr end_node=std::make_shared<Node>(scaled_end_point,m_node_params,m_conn_params);
    if (end_node->isInCollision(m_planning_scene))
    {
      ROS_WARN("this end point is in collision, skipping it");
      end_node->print();
      continue;
    }
    m_nodes.push_back(end_node);
    m_end_nodes.push_back(end_node);
    m_rot_matrix.insert(std::pair<NodePtr,Eigen::MatrixXd>(end_node,computeRotationMatrix(m_start_node->getJoints(),scaled_end_point)));
  }

  for (const NodePtr& end_node: m_end_nodes)
  {
    double cost=std::numeric_limits<double>::infinity();
    Path path;
    std::pair<double,Path> tuple(cost,path);
    std::pair<NodePtr,std::pair<double,Path>> entry(end_node,tuple);
    m_best_path_per_goal.insert(entry);
  }

  m_goal_trees.resize(m_end_nodes.size());
  for (unsigned int idx=0;idx<m_end_nodes.size();idx++)
  {
    m_goal_trees.at(idx)= std::make_shared<Tree>(m_end_nodes.at(idx),
                                                 m_node_params,
                                                 m_conn_params,
                                                 m_planning_scene,
                                                 m_max_square_length,
                                                 Direction::Backward,
                                                 m_nodes,
                                                 m_end_nodes);
    NodePtr last_node_add_to_start_tree;
    bool full_added_to_start_tree=m_start_tree->extend(m_end_nodes.at(idx),last_node_add_to_start_tree);
    if (!full_added_to_start_tree)
    {
      ROS_DEBUG("no direct connection between start and end goal");
      NodePtr last_node_add_to_goal_tree;
      if (last_node_add_to_start_tree)
        m_goal_trees.at(idx)->extend(last_node_add_to_start_tree,last_node_add_to_goal_tree);
      else
        m_goal_trees.at(idx)->extend(m_start_node,last_node_add_to_goal_tree);

    }
    else
    {
      ROS_DEBUG("direct connection between start and end goal");
      Path path;
      if (!m_start_tree->getPathToNode(m_end_nodes.at(idx),path))
      {
        ROS_ERROR("not able to find a path, something wrong");
      }

      storeIfImproveCost(path);
      if (isWrong())
        print();

      //      path=simplifyIntermidiatePoint(path);
      path=pruningPath(path);
      storeIfImproveCost(path);
      if (isWrong())
        print();

      std::vector<Path> paths;
      for (const std::pair<NodePtr,std::pair<double,Path>> p: m_best_path_per_goal)
      {
        if (p.second.first<std::numeric_limits<double>::infinity())
          paths.push_back(p.second.second);
      }
      pruning(paths);
      if (isWrong())
        print();

    }
  }






}

NodePtr Net::searchClosestNode(const std::vector<double>& q)
{
  double min_square_distance=std::numeric_limits<double>::infinity();
  NodePtr closest_node;
  for (const NodePtr& node: m_nodes)
  {
    if (node==m_start_node)
      continue;
    if (std::find(m_end_nodes.begin(),m_end_nodes.end(),node)!=m_end_nodes.end())
      continue;


    double square_dist=squareDistance(q,node->getJoints());
    if (square_dist<min_square_distance)
    {
      if (node->getAncestors().size()>0 && node->getDescendants().size()>0)
      {
        closest_node=node;
        min_square_distance=square_dist;
      }
    }
  }
  if (!closest_node)
    ROS_DEBUG("unable to find a node");
  if (min_square_distance<m_min_square_length)
    return NULL;

  return closest_node;
}

NodePtr Net::addNodeToTheGrid(const std::vector<double>& q)
{

  NodePtr closest_node=searchClosestNode(q);
  if (!closest_node)
    return closest_node;


  NodePtr new_node= addNodeToTheGrid(q,closest_node);
  return new_node;
}

NodePtr Net::addNodeToTheGrid(const std::vector<double>& q, const NodePtr& closest_node)
{

  std::shared_ptr<ha_planner::Node> node=std::make_shared<ha_planner::Node>(q,m_node_params,m_conn_params);

  for (ConnectionPtr& closest_node_conn: closest_node->m_parent_connections)
  {

    const NodePtr& parent=closest_node_conn->getParent();

    ConnectionPtr conn;
    if (parent->checkIfConnectedWith(node,conn))
    {
      if (conn->getParent()!=parent)
        conn=conn->createFlippedConnection();
    }
    else
    {
      conn=std::make_shared<Connection>(parent,node,m_conn_params);
      conn->updatePheromone(closest_node_conn->getPheromone());
      parent->addConnection(conn);
      node->addConnection(conn);

    }
  }
  for (ConnectionPtr& closest_node_conn: closest_node->m_child_connections)
  {

    const NodePtr& child=closest_node_conn->getChild();

    ConnectionPtr conn;
    if (child->checkIfConnectedWith(node,conn))
    {
      if (conn->getChild()!=child)
        conn=conn->createFlippedConnection();
    }
    else
    {
      conn=std::make_shared<Connection>(node,child,m_conn_params);
      conn->updatePheromone(closest_node_conn->getPheromone());
      child->addConnection(conn);
      node->addConnection(conn);
    }
  }
  m_nodes.push_back(node);
  return node;
}

Path Net::addSubGridToTheGrid(const std::vector<std::vector<double>>& q, const NodePtr& closest_node)
{
  if (closest_node->getAncestors().size()==0 || closest_node->getDescendants().size()==0)
  {
    ROS_ERROR("The closest node is unconnected");
    throw std::invalid_argument("closest node is unconnected");
  }
  Path subpath;
  std::vector<std::shared_ptr<ha_planner::Node>> nodes(q.size());
  for (unsigned in=0;in<q.size();in++)
  {
    nodes.at(in)=std::make_shared<ha_planner::Node>(q.at(in),m_node_params,m_conn_params);
    m_nodes.push_back(nodes.at(in));
  }

  for (unsigned in=0;in<(q.size()-1);in++)
  {
    ConnectionPtr conn=std::make_shared<Connection>(nodes.at(in),nodes.at(in+1),m_conn_params);
    conn->updatePheromone(m_max_pheromone);
    nodes.at(in)->addConnection(conn);
    nodes.at(in+1)->addConnection(conn);
    subpath.push_back(conn);
  }

  for (ConnectionPtr& closest_node_conn: closest_node->m_parent_connections)
  {

    const NodePtr& parent=closest_node_conn->getParent();

    ConnectionPtr conn;
    if (parent->checkIfConnectedWith(nodes.at(0),conn))
    {
      if (conn->getParent()!=parent)
      {
        ROS_ERROR("qui");
        conn=conn->createFlippedConnection();
      }
    }
    else
    {
      conn=std::make_shared<Connection>(parent,nodes.at(0),m_conn_params);
      conn->updatePheromone(closest_node_conn->getPheromone());
      parent->addConnection(conn);
      nodes.at(0)->addConnection(conn);
    }
  }

  for (ConnectionPtr& closest_node_conn: closest_node->m_child_connections)
  {

    const NodePtr& child=closest_node_conn->getChild();

    ConnectionPtr conn;
    if (child->checkIfConnectedWith(nodes.back(),conn))
    {
      if (conn->getChild()!=child)
      {
        ROS_ERROR("qui");
        conn=conn->createFlippedConnection();
      }
    }
    else
    {
      conn=std::make_shared<Connection>(nodes.back(),child,m_conn_params);
      conn->updatePheromone(closest_node_conn->getPheromone());
      child->addConnection(conn);
      nodes.back()->addConnection(conn);
    }
  }

  return subpath;
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
  Direction dir=Forward;

  for (unsigned idx=0;idx<500;idx++)
  {


    if (!actual_node)
    {
      ROS_ERROR("ant is not on the net");
      return false;
    }

    if (std::find(m_end_nodes.begin(),m_end_nodes.end(),actual_node)!=m_end_nodes.end())
    {
      //Reach final state
      return true;
    }

    int next_conn;
    next_conn=actual_node->rouletteWheel(dir);
    if (next_conn<0)
    {
      ROS_DEBUG("dead ant");
      removeNodeWithConnections(actual_node);
      return false;
    }
    ConnectionPtr conn=actual_node->m_connections.at(next_conn);

    if (std::find(path.begin(),path.end(),conn)!=path.end())
    {
      ROS_DEBUG("circular path");
      throw std::invalid_argument("circular path");
    }


    path.push_back(conn);
    if (dir==Forward)
      actual_node=conn->getChild();
    else
      actual_node=conn->getParent();


  }
  ROS_DEBUG("unable to find solution");

  return false;
}

double Net::computePathCost(const Path &path)
{
  if (path.size()==0)
    return 0;

  NodePtr actual_node=path.front()->getParent();
  double cost=0;
  for (const ConnectionPtr& conn: path)
  {
    if (actual_node!=conn->getParent())
    {
      ROS_ERROR("the path is broken");
      return std::numeric_limits<double>::infinity();
    }
    cost+=std::sqrt(conn->getLength());
    actual_node=conn->getChild();
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
      if (checkIfImproveCost(path,cost))
      {
        std::pair<double,Path> pair(cost,path);

        const NodePtr& end_node=path.back()->getChild();
        if (best_ants_per_each_goal.count(end_node)>0)
        {
          best_ants_per_each_goal.at(end_node).insert(pair);
        }
        else
        {
          std::map<double,Path> map;
          map.insert(std::pair<double,Path>(cost,path));
          best_ants_per_each_goal.insert(std::pair<NodePtr,std::map<double,Path>>(end_node,map));
        }
      }
    }
  }

  bool improvement=false;
  for (const NodePtr& end_node: m_end_nodes)
  {
    if (best_ants_per_each_goal.count(end_node)==0)
      continue;
    std::map<double,Path>& best_ants=best_ants_per_each_goal.at(end_node); // automatically sorted
    //    ROS_DEBUG("path to goal - there are %zu candidates",best_ants.size());

    for (const std::pair<double,Path>& candidate: best_ants)
    {
      //      ROS_DEBUG("check if the candidate improves the cost and it is feasible. %f",candidate.first);
      if (storeIfImproveCost(candidate.second,candidate.first))
      {
        ROS_DEBUG("cost improvement");
        improvement=true;
        continue;
      }
    }
  }
  return improvement;
}

bool Net::isCollisionFree(const Path& path)
{
  for (const ConnectionPtr& conn: path)
  {
    if (conn->isInCollision(m_planning_scene))
    {
      return false;
    }
  }

  return true;
}

void Net::printBestPath()
{
  ROS_INFO("best path:\ncost:=%f",m_best_cost);
  if (m_best_cost==std::numeric_limits<double>::infinity())
    return;
  printPath(m_best_path);

  ROS_INFO("best path for each goal");
  for (const std::pair<NodePtr,std::pair<double,Path>>& el: m_best_path_per_goal)
  {
    ROS_INFO("\ncost: %f", el.second.first);
    if (el.second.first<=std::numeric_limits<double>::infinity())
      printPath(el.second.second);

  }

}

void Net::printPath(const Path &path)
{
  ROS_INFO("path:\nnumber_of_connections: %zu\npoints:",path.size());
  NodePtr actual_node=path.front()->getParent();
  for (size_t idx=0;idx<path.size();idx++)
  {
    actual_node->print();
    actual_node=path.at(idx)->getChild();
  }
  actual_node->print();
}

void Net::generateNodesFromEllipsoid(const int &number_of_nodes)
{
  if (number_of_nodes<=0)
    return;

  m_estimated_cost=m_best_cost;
  if (m_best_cost==std::numeric_limits<double>::infinity())
  {
    m_estimated_cost=1.01*m_estimated_cost;
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

    unsigned int np=number_of_nodes*1.0/std::min(std::min(m_best_cost,100.0)*5,m_best_path_per_goal.at(m_end_nodes.at(ien)).first)/sum_of_inv_costs;

    std::vector<std::vector<double>> samples=getSamples(m_end_nodes.at(ien),np);
    for (const std::vector<double>& joint: samples)
    {
      NodePtr new_node=addNodeToTheGrid(joint);
      ConnectionPtr conn;
      //      if (!new_node->checkIfConnectedWith(m_end_nodes.at(ien),conn))
      //      {
      //        conn=std::make_shared<Connection>(new_node,m_end_nodes.at(ien),m_conn_params);
      //        new_node->addConnection(conn);
      //        m_end_nodes.at(ien)->addConnection(conn);
      //      }
      //      if (!new_node->checkIfConnectedWith(m_start_node,conn))
      //      {
      //        conn=std::make_shared<Connection>(m_start_node,new_node,m_conn_params);
      //        new_node->addConnection(conn);
      //        m_start_node->addConnection(conn);
      //      }
    }
  }
}


void Net::curvilinearPath()
{
  if (m_best_cost>=std::numeric_limits<double>::infinity())
    return;

  for (const TreePtr &goal_tree: m_goal_trees)
  {
    NodePtr endnode=goal_tree->getRoot();
    double focus=0.5*std::sqrt(squareDistance(m_start_node->getJoints(),
                                              endnode->getJoints()));
    Eigen::Map<const Eigen::VectorXd> p1(m_start_node->getJoints().data(),m_dof);
    Eigen::Map<const Eigen::VectorXd> p2(endnode->getJoints().data(),m_dof);
    Eigen::VectorXd center=(p1+p2)/2.0;

    Eigen::VectorXd semiaxes=computeEllipsoid(m_start_node->getJoints(),
                                              endnode->getJoints(),
                                              m_best_cost);

    Eigen::VectorXd p(m_dof);
    p.setZero();
    p(0)=focus;
    if ((m_rot_matrix.at(endnode)*p+center-p1).norm()<(m_rot_matrix.at(endnode)*p+center-p2).norm())
      focus*=-1;


    Eigen::VectorXd v=4*semiaxes.asDiagonal()*Eigen::MatrixXd::Random(m_dof,1);
    Eigen::VectorXd w=4*semiaxes.asDiagonal()*Eigen::MatrixXd::Random(m_dof,1);
    v(0)=std::abs(v(0));
    w=-v;w(0)=v(0);

    Eigen::VectorXd t=Eigen::VectorXd::LinSpaced(11,0,1);

    Eigen::MatrixXd par(4,m_dof);
    for (unsigned int idof=0;idof<m_dof;idof++)
    {
      if (idof==0)
      {
        par(0,idof)=v(0)+w(0)-4*focus;
        par(1,idof)=6*focus-w(0)-2*v(0);
        par(2,idof)=v(0);
        par(3,idof)=-focus;
      }
      else
      {
        par(0,idof)=v(idof)+w(idof);
        par(1,idof)=-w(idof)-2*v(idof);
        par(2,idof)=v(idof);
        par(3,idof)=0;
      }
    }

    double distance=0;
    Eigen::VectorXd old_p(m_dof);
    std::vector<std::vector<double>> points;
    for (unsigned int idx=0;idx<t.rows();idx++)
    {
      for (size_t idof=0;idof<m_dof;idof++)
      {

        p(idof)=par(0,idof)*std::pow(t(idx),3)+
                par(1,idof)*std::pow(t(idx),2)+
                par(2,idof)*t(idx)+
                par(3,idof);
      }
      p=m_rot_matrix.at(endnode)*p+center;
      std::vector<double> joints(&p[0], p.data()+p.cols()*p.rows());
      points.push_back(joints);
      if (idx>0)
        distance+=(p-old_p).norm();
      old_p=p;

    }

    NodePtr last_added_node=m_start_node;

    if (distance<m_best_cost)
    {
      bool full_added;
      unsigned int istart=1;
      for (;istart<t.rows();istart++)
      {
        if (istart<(t.rows()-1))
          full_added=m_start_tree->createAndExtendFromNode(points.at(istart),last_added_node,last_added_node);
        else
          full_added=m_start_tree->extendFromNode(endnode,last_added_node,last_added_node);

        if (full_added==false)
        {
          break;

        }
      }
      if (full_added)
      {
        Path path;
        m_start_tree->getPathToNode(endnode,path);
        double cost=computePathCost(path);
        storeIfImproveCost(path,cost);
        ROS_FATAL("so far so good (cost=%f == %f < %f)",distance,cost,m_best_cost);

      }
      else
      {
        last_added_node=endnode;
        for (unsigned int iend=t.rows()-2;iend>=istart;iend--)
        {
          full_added=goal_tree->createAndExtendFromNode(points.at(iend),last_added_node,last_added_node);

          if (full_added==false)
          {
            ROS_FATAL("stopped between points %u-%u",istart,iend);
            break;
          }
        }
      }
    }
  }
}

void Net::print()
{
  ROS_INFO("net of %zu nodes", m_nodes.size());
  for (unsigned int ir=0;ir<m_nodes.size();ir++)
  {
    ROS_DEBUG("Node %u has %u connections",ir,m_nodes.at(ir)->getConnectionsNumber());
    for (const ConnectionPtr& conn: m_nodes.at(ir)->m_connections)
    {
      NodePtr other=conn->getOtherNode(m_nodes.at(ir));
      std::vector<NodePtr>::iterator it=std::find(m_nodes.begin(),m_nodes.end(),other);
      if (it==m_nodes.end())
      {
        ROS_ERROR("this connection involves a node outside the net");
        return;
      }
      if (conn->getParent()==other)
        std::cout << "From: ";
      else
        std::cout << "To:";
      std::cout << std::distance(m_nodes.begin(),it) << ";";
    }
    std::cout << std::endl;

  }
  std::cout<< "  ";
  for (unsigned int ir=0;ir<m_nodes.size();ir++)
    std::cout << ir%10 << " ";
  std::cout<<std::endl;

  for (unsigned int ir=0;ir<m_nodes.size();ir++)
  {
    std::cout << ir%10 << " ";
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
      bool best_path=false;
      bool duplicated=false;
      bool conn_error=false;
      bool to_child=false;
      for (const ConnectionPtr& conn: m_nodes.at(ir)->m_connections)
      {
        if (ir==ic)
          continue;

        if (conn->getChild()==m_nodes.at(ic))
        {
          to_child=true;
          if (conn->getParent()!=m_nodes.at(ir))
          {
            conn_error=true;
          }
          if (!best_path)
            best_path= (std::find(m_best_path.begin(),m_best_path.end(),conn)!=m_best_path.end());
          if (found)
          {
            duplicated=true;
          }
          found=true;
          is_in_collision=conn->isInCollision(m_planning_scene);

        }
        else if (conn->getParent()==m_nodes.at(ic))
        {
          if (conn->getChild()!=m_nodes.at(ir))
          {
            conn_error=true;
          }
          if (!best_path)
            best_path= (std::find(m_best_path.begin(),m_best_path.end(),conn)!=m_best_path.end());
          if (found)
          {
            duplicated=true;
          }
          found=true;
          is_in_collision=conn->isInCollision(m_planning_scene);

        }
      }

      if (is_starting)
        std::cout << "S";
      else if (is_ending)
        std::cout << "E";
      else if (found && !is_in_collision)
      {
        if (conn_error)
          std::cout << "\033[0;41m";
        else if (duplicated)
          std::cout << "\033[0;35m";
        else if (to_child)
          std::cout << "\033[0;36m";

        if (best_path)
          std::cout << "B";
        else
          std::cout << "*";
        if (duplicated || conn_error || to_child)
          std::cout << "\033[0m";
      }
      else
        std::cout << " ";
      std::cout << "|";
    }
    std::cout << std::endl;
    for (unsigned int ir=0;ir<m_nodes.size();ir++)
      std::cout << "- ";
    std::cout << std::endl;

  }
}

unsigned int Net::removeUnconnectedNodes()
{
  unsigned int num_of_removed_nodes=0;
  for (unsigned int in=0;in<m_nodes.size();in++)
  {
    if (m_nodes.at(in)==m_start_node)
      continue;
    if (std::find(m_end_nodes.begin(),m_end_nodes.end(),m_nodes.at(in))!=m_end_nodes.end())
      continue;

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
    m_nodes.erase(el,el+1);
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
        if (conn->isCollisionChecked())
          continue;
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
    for (unsigned int idof=0;idof<m_dof;idof++)
      waypoints.back().at(idof)*=m_conn_params.unscaling.at(idof);

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


bool Net::runRRTConnect()
{



  bool success=false;
  bool fail=false;
  unsigned int iter=0;
  std::vector<Path> paths;
  for (const std::pair<NodePtr,std::pair<double,Path>> p: m_best_path_per_goal)
  {
    if (p.second.first<std::numeric_limits<double>::infinity())
      paths.push_back(p.second.second);
  }

  TreePtr start_tree;
  std::vector<TreePtr> goal_trees(m_end_nodes.size());
  start_tree =  std::make_shared<Tree>(m_start_node,
                                       m_node_params,
                                       m_conn_params,
                                       m_planning_scene,
                                       m_max_square_length,
                                       Direction::Forward,
                                       m_nodes,
                                       m_end_nodes);

  for (unsigned int idx=0;idx<m_end_nodes.size();idx++)
  {
    goal_trees.at(idx)= std::make_shared<Tree>(m_end_nodes.at(idx),
                                               m_node_params,
                                               m_conn_params,
                                               m_planning_scene,
                                               m_max_square_length,
                                               Direction::Backward,
                                               m_nodes,
                                               m_end_nodes);
  }

  unsigned int ien=0;

  double sample_time=0;
  double start_tree_time=0;
  double goal_tree_time=0;
  double pruning_time=0;
  double warp_time=0;
  double split_time=0;


  while (!success && !fail)
  {
    ros::Time ta=ros::Time::now();

    if (m_best_cost<std::numeric_limits<double>::infinity())
      if (iter++>200)
        fail=true;
      else
        if (iter++>1000)
          fail=true;

    ROS_FATAL("QUI");
    std::vector<double> joint=sample(m_end_nodes.at(ien));
    ROS_FATAL("QUI");
    if (++ien>=m_end_nodes.size())
      ien=0;

    ros::Time tb=ros::Time::now();
    sample_time+=(tb-ta).toSec();

    NodePtr last_node_add_to_start_tree;
    bool full_added_to_start_tree=start_tree->createAndExtend(joint,last_node_add_to_start_tree);
    if (isWrong())
    {
      ROS_ERROR("ehi");
      print();
      return false;
    }

    ros::Time tc=ros::Time::now();
    start_tree_time+=(tc-tb).toSec();

    for (TreePtr& goal_tree: goal_trees)
    {
      NodePtr last_node_add_to_goal_tree;


      bool full_added_to_stop_tree=goal_tree->extend(last_node_add_to_start_tree,last_node_add_to_goal_tree);

      if (!full_added_to_start_tree && !full_added_to_stop_tree)
      {
        if (start_tree->tryConnectWith(last_node_add_to_goal_tree))
          full_added_to_stop_tree=true;
        else
        {
          goal_tree->createAndExtend(joint,last_node_add_to_goal_tree);

          if (isWrong())
          {
            ROS_ERROR("ehi");
            print();
            return false;
          }

          if (goal_tree->tryConnectWith(last_node_add_to_start_tree))
            full_added_to_stop_tree=true;
          else if (start_tree->tryConnectWith(last_node_add_to_goal_tree))
            full_added_to_stop_tree=true;
        }
      }

      ros::Time td2=ros::Time::now();
      goal_tree_time+=(td2-tc).toSec();

      if (isWrong())
      {
        ROS_ERROR("ehi");
        print();
        return false;
      }
      if (full_added_to_stop_tree)
      {

        Path path;

        //        ROS_DEBUG("insert in the path the branch of the start_tree");
        start_tree->getPathToNode(last_node_add_to_start_tree,path);

        //        ROS_DEBUG("insert in the path the branch of the goal_tree");
        goal_tree->getPathToNode(last_node_add_to_start_tree,path);


        unsigned int pathsize1=path.size();
        path=pruningPath(/*simplifyIntermidiatePoint*/(path));

        ros::Time te=ros::Time::now();
        pruning_time+=(te-td2).toSec();

        path=warpPath(1,path);

        ros::Time tf=ros::Time::now();
        warp_time+=(tf-te).toSec();

        path=splitPath(2,path);

        ros::Time tg=ros::Time::now();
        split_time+=(tg-tf).toSec();

        double cost=computePathCost(path);
        unsigned int pathsize2=path.size();

        ROS_INFO("ITER=%u,start tree=%u, goal tree=%u, cost=%f, path_length=%u->%u",iter,start_tree->size(),goal_tree->size(),cost,pathsize1,pathsize2);

        if (storeIfImproveCost(path,cost))
        {
          if (isWrong())
          {
            ROS_ERROR("ehi");
            print();
            return false;
          }
          for (TreePtr& old_goal_tree: m_goal_trees)
          {
            if (old_goal_tree->getRoot()==goal_tree->getRoot())
              old_goal_tree=goal_tree;
          }

          m_start_tree=start_tree;
        }

        paths.push_back(path);
        success=true;
      }
    }

  }

  ROS_INFO("take sample       %f ms",sample_time*1e3);
  ROS_INFO("extend start tree %f ms",start_tree_time*1e3);
  ROS_INFO("extend goal tree  %f ms",goal_tree_time*1e3);
  ROS_INFO("pruningPath       %f ms",pruning_time*1e3);
  ROS_INFO("warpPath          %f ms",warp_time*1e3);
  ROS_INFO("splitPath         %f ms",split_time*1e3);


  for (const std::pair<NodePtr,std::pair<double,Path>> p: m_best_path_per_goal)
  {
    if (p.second.first<std::numeric_limits<double>::infinity())
      paths.push_back(p.second.second);
  }
  pruning(paths);

  if (!success)
    ROS_FATAL("unable to find a solution");


  return success;

}

bool Net::addTree(const NodePtr& end_node, Path& new_path)
{
  bool found_valid=false;
  NodePtr node;
  for (unsigned int idx=0;idx<100;idx++)
  {
    std::vector<double> joints=sample(end_node);
    node=std::make_shared<Node>(joints,m_node_params,m_conn_params);
    if (!node->isInCollision(m_planning_scene))
    {
      found_valid=true;
      break;
    }
  }
  if (!found_valid)
  {
    ROS_ERROR("this node is in collision");
    return false;
  }
  m_nodes.push_back(node);
  TreePtr forward_tree =  std::make_shared<Tree>(node,
                                                 m_node_params,
                                                 m_conn_params,
                                                 m_planning_scene,
                                                 m_max_square_length,
                                                 Direction::Forward,
                                                 m_nodes,
                                                 m_end_nodes);


  std::vector<ConnectionPtr>::iterator first_rand_it;
  Path first_path;
  unsigned int iter=0;
  for (; iter<100;iter++)
  {
    first_rand_it=m_best_path_per_goal.at(end_node).second.begin();
    // prefer first parts
    int rand_advance=std::pow(m_ud(m_gen),2.0)*(m_best_path_per_goal.at(end_node).second.size()-1);
    std::advance(first_rand_it, rand_advance);


    NodePtr last_node;
    bool is_full_added=forward_tree->extend((*first_rand_it)->getParent(),last_node);
    if (is_full_added)
    {
      if (forward_tree->getPathToNode(last_node,first_path))
        break;
      first_path.clear();
    }
    else
    {
      forward_tree->createAndExtend(sample(end_node),last_node);
    }
  }
  ROS_INFO("first path iter=%u, %s",iter,(first_path.size()==0)?"fail":"true");
  if (first_path.size()==0)
    return false;

  int last_portion=std::distance(first_rand_it, m_best_path_per_goal.at(end_node).second.end())-1;
  std::vector<ConnectionPtr>::iterator second_rand_it;
  Path second_path;
  iter=0;
  for (; iter<100;iter++)
  {
    second_rand_it=first_rand_it+1;
    // prefer last part
    int rand_advance=std::pow(m_ud(m_gen),0.5)*last_portion;
    std::advance(second_rand_it, rand_advance);

    NodePtr last_node;
    bool is_full_added=forward_tree->extend((*second_rand_it)->getChild(),last_node);
    if (is_full_added)
    {
      if (forward_tree->getPathToNode(last_node,second_path))
        break;
      second_path.clear();
    }
    else
    {
      ROS_INFO("random extend");
      forward_tree->createAndExtend(sample(end_node),last_node);
    }
  }
  ROS_INFO("second path iter=%u, %s",iter,(first_path.size()==0)?"fail":"true");
  if (second_path.size()==0)
    return false;


  new_path= Path(m_best_path_per_goal.at(end_node).second.begin(),first_rand_it);

  // remove the common parts
  int common_elements=0;
  for (common_elements=0;common_elements<std::min(first_path.size(),second_path.size());common_elements++)
  {
    if (first_path.at(common_elements)!=second_path.at(common_elements))
    {
      break;
    }
  }



  // add first path
  for (int idx=(first_path.size()-1);idx>=common_elements;idx--)
  {
    first_path.at(idx)->flipDirection();
    new_path.push_back(first_path.at(idx));
  }
  // add second path
  for (int idx=common_elements;idx<second_path.size();idx++)
    new_path.push_back(second_path.at(idx));


  Path last_portion_path= Path(second_rand_it+1,m_best_path_per_goal.at(end_node).second.end());

  for (ConnectionPtr& conn: last_portion_path)
    new_path.push_back(conn);


  new_path=pruningPath(new_path);
  double new_cost=computePathCost(new_path);

  storeIfImproveCost(new_path,new_cost);
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
  ConnectionPtr conn=path.at(0);
  bool nedd_append=false;
  for (unsigned int idx=1;idx<path.size();idx++)
  {
    NodePtr child=path.at(idx)->getChild();
    if (conn->dotProduct(path.at(idx))>0.999)
    {
      if (parent->checkIfConnectedWith(child,conn))
      {
        if (conn->getParent()!=parent)
          throw std::invalid_argument("circular path");
      }
      else
      {
        conn=std::make_shared<Connection>(parent,child,m_conn_params);
        parent->addConnection(conn);
        child->addConnection(conn);
        conn->forceNotCollision();
      }
      nedd_append=true;
    }
    else
    {
      nedd_append=false;
      parent=child;
      simplified_path.push_back(conn);
    }
  }
  if (nedd_append)
    simplified_path.push_back(conn);

  ROS_INFO("simplified from %zu to %zu step",path.size(),simplified_path.size());

  return simplified_path;
}

void Net::pruning(const std::vector<Path>& paths)
{
  std::vector<NodePtr> pruned_nodes;
  pruned_nodes.push_back(m_start_node);
  for (NodePtr& node: m_end_nodes)
    pruned_nodes.push_back(node);

  for (const Path& path : paths)
  {
    if (path.size()==0)
    {
      ROS_WARN("ask to prune with a empty path");
      continue;
    }
    NodePtr n=path.at(0)->getParent();
    for (const ConnectionPtr& conn: path)
    {
      if (std::find(pruned_nodes.begin(),pruned_nodes.end(),n)==pruned_nodes.end())
        pruned_nodes.push_back(n);
      n=conn->getChild();
    }
    if (std::find(pruned_nodes.begin(),pruned_nodes.end(),n)==pruned_nodes.end())
      pruned_nodes.push_back(n);

  }
  ROS_DEBUG("pruning %zu nodes", m_nodes.size()-pruned_nodes.size());
  unsigned int idx=0;
  while (idx<m_nodes.size())
  {
    if (std::find(pruned_nodes.begin(),pruned_nodes.end(),m_nodes.at(idx))==pruned_nodes.end())
      removeNodeWithConnections(m_nodes.at(idx));
    else
      idx++;
  }
}

bool Net::isWrong()
{
  for (unsigned int ir=0;ir<m_nodes.size();ir++)
  {
    double duplicated=false;
    for (unsigned int is=ir+1;is<m_nodes.size();is++)
    {
      if (m_nodes.at(is)==m_nodes.at(ir))
      {
        duplicated=true;
        ROS_ERROR("elements %u and %u are pointing on the same node",ir,is);
      }
    }
    if (duplicated)
      return true;

    for(ConnectionPtr& conn: m_nodes.at(ir)->m_connections)
    {
      if (conn->getChild()!=m_nodes.at(ir) && conn->getParent()!=m_nodes.at(ir) )
      {
        ROS_ERROR("this connection is not involving this node");
        return true;
      }
      NodePtr other=conn->getOtherNode(m_nodes.at(ir));
      if (other==m_nodes.at(ir))
      {
        ROS_ERROR("This connection is involving twice the same node");
        return true;
      }

      if (std::find(m_nodes.begin(),m_nodes.end(),other)==m_nodes.end())
      {
        ROS_ERROR("This connection is with a untracked node");
        return true;
      }
    }
  }
  return false;
}

bool Net::checkIfImproveCost(const Path &path)
{
  double cost=computePathCost(path);
  return checkIfImproveCost(path,cost);
}
bool Net::checkIfImproveCost(const Path &path, const double &cost)
{

  NodePtr  n=path.back()->getChild();
  if (m_best_path_per_goal.count(n)==0)
    return false;
  for (ConnectionPtr conn: path)
  {
    if (conn->isCollisionChecked())
      if (conn->isInCollision(m_planning_scene))
        return false;
  }

  if (cost < m_best_path_per_goal.at(n).first)
    return true;

  return false;
}


bool Net::storeIfImproveCost(const Path& path)
{
  double cost=computePathCost(path);
  return storeIfImproveCost(path,cost);
}

bool Net::storeIfImproveCost(const Path &path, const double &cost)
{

  NodePtr  n=path.back()->getChild();
  if (m_best_path_per_goal.count(n)==0)
  {
    ROS_ERROR("path is not going to an end node");
    return false;
  }
  if (path.front()->getParent()!=m_start_node)
  {
    ROS_ERROR("path is not going to an end node");
    return false;
  }
  if (cost < m_best_path_per_goal.at(n).first)
  {
    if (!isCollisionFree(path))
      return false;
    m_best_path_per_goal.at(n).first=cost;
    m_best_path_per_goal.at(n).second=path;

    if (cost<0.9999*m_best_cost)
    {
      //      ROS_INFO("this path improves the overall best path (cost %f) delta=%e",cost,m_best_cost-cost);
      m_best_cost=cost;
      m_best_path=path;
    }
    else
      ROS_INFO("this path improved the best path to goal (cost %f)",cost);
    return true;
  }
  return false;
}

Path Net::pruningPath(Path path)
{
  unsigned int ic=0;

  while (ic<(path.size()-1))
  {
    bool cutted=false;
    for (unsigned int inext=path.size()-1/*std::min(ic+4,(unsigned int)path.size()-1)*/;inext>=ic+1;inext--)
    {
      ConnectionPtr shortcut_connection;
      bool is_alreay_present=false;
      if (path.at(ic)->getParent()->checkIfConnectedWith(path.at(inext)->getChild(),shortcut_connection))
      {
        is_alreay_present=true;
        // if the connection is in collision, continue to next cycle
        if (shortcut_connection->isCollisionChecked())
          if (shortcut_connection->isInCollision(m_planning_scene))
            continue;

        // needs to flip a connection
        if (shortcut_connection->getParent()!=path.at(ic)->getParent())
          shortcut_connection=shortcut_connection->createFlippedConnection();
      }
      else
        shortcut_connection=std::make_shared<Connection>(path.at(ic)->getParent(),path.at(inext)->getChild(),m_conn_params);

      if (!shortcut_connection->isInCollision(m_planning_scene))
      {
        double old_length=0;
        for (unsigned int idx=ic;idx<=inext;idx++)
          old_length+=path.at(idx)->getLength();

        if (shortcut_connection->getLength()<old_length)
        {
          if (!is_alreay_present)
          {
            shortcut_connection->getParent()->addConnection(shortcut_connection);
            shortcut_connection->getChild()->addConnection(shortcut_connection);
          }

          path.erase(path.begin()+ic,path.begin()+inext+1);
          path.insert(path.begin()+ic,shortcut_connection);

          cutted=true;
          break;
        }
      }
    }
    if (!cutted)
      ic++;
  }

  return path;
}

double Net::getPathDistance(const Path &path, const NodePtr &node)
{
  double distance=0;
  bool found=false;

  if (path.front()->getParent()==node)
    return distance;

  for (const ConnectionPtr& conn: path)
  {
    distance+=std::sqrt(conn->getLength());
    if (conn->getChild()==node)
    {
      found=true;
      break;
    }
  }
  if (!found)
    throw std::invalid_argument("node is not part of the path");
  return distance;
}


void Net::generateNodesFromGrid(const int &points_per_dimension)
{
  if (points_per_dimension<=0)
    return;

  unsigned int num_nodes=m_nodes.size();


  unsigned int grid_global_nodes = std::pow(points_per_dimension,m_dof);

  for (const TreePtr &goal_tree: m_goal_trees)
  {
    NodePtr endnode=goal_tree->getRoot();

    Eigen::Map<const Eigen::VectorXd> p1(m_start_node->getJoints().data(),m_dof);
    Eigen::Map<const Eigen::VectorXd> p2(endnode->getJoints().data(),m_dof);
    Eigen::VectorXd center=(p1+p2)/2.0;

    Eigen::VectorXd semiaxes=computeEllipsoid(m_start_node->getJoints(),
                                              endnode->getJoints(),
                                              m_best_cost);
    Eigen::VectorXd p(m_dof);

    std::vector<std::vector<double>> qgrid;

    double max_delta=0;
    for (size_t idof=0;idof<m_dof;idof++)
    {
      double delta=2*semiaxes(idof)/(double)(points_per_dimension-1);
      max_delta=std::max(delta,max_delta);
      std::vector<double> idim(points_per_dimension);
      ROS_FATAL("delta=%f, point per dim=%u",delta, points_per_dimension);

      idim.at(0)=-semiaxes(idof);
      for (unsigned int idx=1;idx<points_per_dimension;idx++)
        idim.at(idx)=idim.at(idx-1)+delta;
      qgrid.push_back(idim);
    }


    std::vector<double> q(m_dof);
    std::vector<unsigned int> indices(m_dof,0);
    std::vector<NodePtr> grid_nodes(grid_global_nodes);

    bool connected_with_endnode=false;
    bool connected_with_startnode=false;

    for (unsigned int in=0; in<grid_global_nodes;in++)
    {

      for (size_t idof=0;idof<m_dof;idof++)
        p(idof)=qgrid.at(idof).at(indices.at(idof));

      p=m_rot_matrix.at(endnode)*p+center;

      bool out_of_bounds=false;
      for (size_t idof=0;idof<m_dof;idof++)
      {
        q.at(idof)=p(idof);
        if ((q.at(idof)>m_ub.at(idof)) || (q.at(idof)<m_lb.at(idof)))
          out_of_bounds=true;
      }

      double dist_from_start=std::sqrt(squareDistance(m_start_node->getJoints(),q));
      double dist_from_end=std::sqrt(squareDistance(endnode->getJoints(),q));

      if ( (!out_of_bounds) && ((dist_from_start+dist_from_end)<m_best_cost))
      {


        grid_nodes.at(in)=std::make_shared<Node>(q,m_node_params,m_conn_params);
        grid_nodes.at(in)->computeHeuristic(m_end_nodes);


        // create connection
        for (size_t idof=0;idof<m_dof;idof++)
        {
          if (indices.at(idof)==0)
            continue;

          unsigned int in2=0;
          unsigned int other_ids=0;
          for (size_t idof2=0;idof2<m_dof;idof2++)
          {
            if (idof!=idof2)
            {
              in2+=indices.at(idof2)*std::pow(points_per_dimension,idof2);
              other_ids+=idof2;
            }
            else
            {
              in2+=(indices.at(idof2)-1)*std::pow(points_per_dimension,idof2);
            }
          }

          if (grid_nodes.at(in2))
          {
            ConnectionPtr conn;
            if (other_ids%2==0)
            {
              conn=std::make_shared<Connection>(grid_nodes.at(in2),grid_nodes.at(in),m_conn_params);
            }
            else
            {
              conn=std::make_shared<Connection>(grid_nodes.at(in),grid_nodes.at(in2),m_conn_params);
            }

            if (conn->getChild()->getHeuristic()>conn->getParent()->getHeuristic())
              conn->updatePheromone(m_max_pheromone);
            else
              conn->updatePheromone(m_min_pheromone);

            grid_nodes.at(in2)->addConnection(conn);
            grid_nodes.at(in)->addConnection(conn);

          }
        }


        for (ConnectionPtr best_conn: m_best_path)
        {
          if (std::sqrt(squareDistance(best_conn->getParent()->getJoints(),grid_nodes.at(in)->getJoints()))<4*max_delta)
          {
            ConnectionPtr conn=std::make_shared<Connection>(best_conn->getParent(),grid_nodes.at(in),m_conn_params);
            if (connected_with_startnode || conn->isInCollision(m_planning_scene))
            {
              best_conn->getParent()->addConnection(conn);
              grid_nodes.at(in)->addConnection(conn);
              connected_with_startnode=true;
            }
          }
        }

        if (std::sqrt(squareDistance(endnode->getJoints(),grid_nodes.at(in)->getJoints()))<4*max_delta)
        {
          ConnectionPtr conn=std::make_shared<Connection>(grid_nodes.at(in),endnode,m_conn_params);
          if (connected_with_endnode || conn->isInCollision(m_planning_scene))
          {
            endnode->addConnection(conn);
            grid_nodes.at(in)->addConnection(conn);
            connected_with_endnode=true;
          }
        }
        m_nodes.push_back(grid_nodes.at(in));



      }



      for (size_t idof=0;idof<m_dof;idof++)
      {

        indices.at(idof)++;
        if (indices.at(idof)>=points_per_dimension) // if true, the next for iteration increment the following indices
          indices.at(idof)=0;
        else  // if false, do not increment next indices
          break;
      }


    }

    if (!connected_with_endnode || !connected_with_startnode)
      ROS_FATAL("djjdddddddddddddddddddddddddddddddddddddd");
  }

  ROS_FATAL("Created a net of %zu nodes",m_nodes.size()-num_nodes);

}

}

