#include <net_path_core/net_path_core.h>

namespace ha_planner {


Net::Net(const unsigned int& dof,
         const std::string& group_name,
         const planning_scene::PlanningSceneConstPtr& planning_scene,
         const std::vector<double> scaling,
         const std::vector<double> lb,
         const std::vector<double> ub):
  m_planning_scene(planning_scene),
  m_gen(time(0))
{

  m_ud=std::uniform_real_distribution<double>(0,1);

  m_dof=dof;
  m_lb.resize(m_dof,0);
  m_ub.resize(m_dof,0);

  m_min_length=0.06;
  m_max_length=.8; //2

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
  m_conn_params.weigth=50;

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
    m_conn_params.unscaling.at(idx)=1.0/scaling.at(idx);
    m_unscaling(idx)=1.0/scaling.at(idx);
    m_lb.at(idx)=lb.at(idx)*scaling.at(idx);
    m_ub.at(idx)=ub.at(idx)*scaling.at(idx);
  }

  m_best_cost = std::numeric_limits<double>::infinity();
  m_best_path.resize(0);

  ROS_PROTO("inizializzato!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
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

      ConnectionPtr tmp_conn;
      if (!node->checkIfConnectedWith(path.at(idx+1)->getChild(),tmp_conn))
      {
        ROS_PROTO("QUI");
      }

      unsigned int iter=0;
      while(iter<100 && (max_distance-min_distance)>m_min_length)
      {
        perturbation=-distance*prominence;

        Eigen::VectorXd p2new=p2+perturbation;

        if (!node->checkIfConnectedWith(path.at(idx+1)->getChild(),tmp_conn))
        {
          ROS_PROTO("QUI");
        }
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
  Path new_path=warpPath(number_of_trials,m_best_path);
  storeIfImproveCost(new_path);
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
  std::vector<std::vector<double>> solutions=getSamples(endnode,1);
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
        ball.setRandom();
        ball*=std::pow(m_ud(m_gen),1.0/(double)m_dof)/ball.norm();
        q=m_rot_matrix.at(endnode)*semiaxes.asDiagonal()*ball+center;

        out_of_bounds=false;
        for (unsigned int iax=0;iax<m_dof;iax++)
          if ((joints.at(iax)<m_lb.at(iax)) || joints.at(iax)>m_ub.at(iax))
            out_of_bounds=true;

      }
      samples.push_back(joints);

    }
  }
  else
  {
    for (unsigned int is=0;is<number_of_samples;is++)
    {
      for (unsigned int idof=0;idof<m_dof;idof++)
        joints.at(idof)=m_lb.at(idof)+(m_ub.at(idof)-m_lb.at(idof))*m_ud(m_gen);
      samples.push_back(joints);

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
  m_nodes.clear();
  m_nodes.push_back(m_start_node);

  m_end_nodes.resize(0);
  m_best_path_per_goal.clear();
  m_rot_matrix.clear();
  m_utopia_cost=std::numeric_limits<double>::infinity();
  m_best_cost = std::numeric_limits<double>::infinity();
  m_best_path.resize(0);
  NodePtr actual_node=m_start_node;

  m_start_tree =  std::make_shared<Tree>(m_start_node,
                                         m_node_params,
                                         m_conn_params,
                                         m_planning_scene,
                                         m_max_square_length,
                                         Direction::Forward,
                                         m_nodes,
                                         m_end_nodes);
  m_start_tree->setHumanFilter(m_human_filter);


  for (const std::vector<double>& end_point: end_points)
  {
    assert(m_dof==end_point.size());

    std::vector<double> scaled_end_point(m_dof);
    for (unsigned int idof=0;idof<m_dof;idof++)
      scaled_end_point.at(idof)=end_point.at(idof)*m_conn_params.scaling.at(idof);
    NodePtr end_node=std::make_shared<Node>(scaled_end_point,m_node_params,m_conn_params);
    if (end_node->isInCollision(m_planning_scene))
    {
      ROS_WARN("this end point is in collision, skipping it");
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
    double utopia=std::sqrt(squareDistance(end_node->getJoints(),m_start_node->getJoints()));
    m_utopia_cost=std::min(m_utopia_cost,utopia);
    m_utopia_costs.insert(std::pair<NodePtr,double>(end_node,utopia));
    m_estimated_cost=std::max(m_estimated_cost,1.5*sqrt(utopia));

    if (utopia<m_conn_params.checking_collision_distance)
    {
      ROS_DEBUG("start and end points are almost the same");
      ConnectionPtr conn=std::make_shared<Connection>(m_start_node,end_node,m_conn_params);
      conn->registerConnection();
      Path path;
      path.push_back(conn);
      storeIfImproveCost(path);
    }
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
    m_goal_trees.at(idx)->setHumanFilter(m_human_filter);
    NodePtr last_node_add_to_start_tree;
    if (m_utopia_costs.at(m_end_nodes.at(idx))<5*m_best_cost)
    {
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
        ROS_INFO("direct connection between start and end goal");
        Path path;
        if (!m_start_tree->getPathToNode(m_end_nodes.at(idx),path))
        {
          ROS_ERROR("not able to find a path, something wrong");
        }

        path=pruningPath(path);
        double cost=computePathCost(path);
        storeIfImproveCost(path);

        //      std::vector<Path> paths;
        //      for (const std::pair<NodePtr,std::pair<double,Path>> p: m_best_path_per_goal)
        //      {
        //        if (p.second.first<std::numeric_limits<double>::infinity())
        //          paths.push_back(p.second.second);
        //      }
        //      pruning(paths);


      }
    }
    else
      ROS_PROTO("utopia cost for this goal is 5times high that the best cost, skipping it for direct connection checking");
  }


  m_estimated_cost=10*m_best_cost;

}

void Net::computeNodeCost()
{
  if (m_human_filter)
  {
    ROS_PROTO("divide path");
    m_best_path=dividePath(m_best_path,m_min_length*2);
    for (const NodePtr& n: m_end_nodes)
    {
      m_best_path_per_goal.at(n).second=dividePath(m_best_path_per_goal.at(n).second,m_min_length*2);
    }

    ROS_PROTO("recomputing cost");
    for (NodePtr& node: m_nodes)
    {
      if (m_human_filter)
        computeOccupancy(node);
    }
    for (const NodePtr& n: m_end_nodes)
    {
      std::pair<double,Path> new_cost(computePathCost(m_best_path_per_goal.at(n).second),m_best_path_per_goal.at(n).second);
      m_best_path_per_goal.at(n)=new_cost;
    }
    m_best_cost=computePathCost(m_best_path);
  }
  else
    ROS_WARN("no human filter set");
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

  if (m_human_filter)
  {
    computeOccupancy(node);
  }

  for (ConnectionPtr& closest_node_conn: closest_node->m_parent_connections)
  {

    const NodePtr& parent=closest_node_conn->getParent();

    ConnectionPtr conn;
    conn=std::make_shared<Connection>(parent,node,m_conn_params);
    conn->updatePheromone(closest_node_conn->getPheromone());
    conn->registerConnection();
  }
  for (ConnectionPtr& closest_node_conn: closest_node->m_child_connections)
  {

    const NodePtr& child=closest_node_conn->getChild();

    ConnectionPtr conn;
    conn=std::make_shared<Connection>(node,child,m_conn_params);
    conn->updatePheromone(closest_node_conn->getPheromone());
    conn->registerConnection();
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
    computeOccupancy(nodes.at(in));
    m_nodes.push_back(nodes.at(in));
  }

  for (unsigned in=0;in<(q.size()-1);in++)
  {
    ConnectionPtr conn=std::make_shared<Connection>(nodes.at(in),nodes.at(in+1),m_conn_params);
    conn->updatePheromone(m_max_pheromone);

    conn->registerConnection();
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
      conn->registerConnection();
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
      conn->registerConnection();
    }
  }

  return subpath;
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
  {
    if (m_best_path_per_goal.at(m_end_nodes.at(ien)).first<1.01*m_utopia_costs.at(m_end_nodes.at(ien)))
      continue;
    sum_of_inv_costs+=1.0/std::min(std::min(m_best_cost,100.0)*5,m_best_path_per_goal.at(m_end_nodes.at(ien)).first);
  }

  std::random_device rd{};
  std::mt19937 gen{rd()};
  std::normal_distribution<double> nd(0.0,1.0);
  std::uniform_real_distribution<double> ud(0,1);
  for  (unsigned int ien=0;ien<m_end_nodes.size();ien++)
  {
    if (m_best_path_per_goal.at(m_end_nodes.at(ien)).first<1.01*m_utopia_costs.at(m_end_nodes.at(ien)))
      continue;
    unsigned int np=number_of_nodes*1.0/std::min(std::min(m_best_cost,100.0)*5,m_best_path_per_goal.at(m_end_nodes.at(ien)).first)/sum_of_inv_costs;

    std::vector<std::vector<double>> samples=getSamples(m_end_nodes.at(ien),np);
    for (const std::vector<double>& joint: samples)
      addNodeToTheGrid(joint);
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
  for (unsigned int idof=0;idof<m_dof;idof++)
    waypoints.back().at(idof)*=m_conn_params.unscaling.at(idof);
  return waypoints;
}

const Path& Net::getBestPathRef(){return m_best_path;}

std::vector<std::vector<double>> Net::getBestPath()
{
  return getPath(m_best_path);
}


bool Net::runRRTConnect()
{
  ROS_PROTO("Check if problem is already solved");
  if (isSolutionFound())
  {
    ROS_PROTO("solution is already found");
    return true;
  }
  ROS_PROTO("run RRT");

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
  start_tree->setHumanFilter(m_human_filter);

  for (unsigned int idx=0;idx<m_end_nodes.size();idx++)
  {
    ROS_PROTO("creating tree %u of %u",idx,m_end_nodes.size());

    goal_trees.at(idx)= std::make_shared<Tree>(m_end_nodes.at(idx),
                                               m_node_params,
                                               m_conn_params,
                                               m_planning_scene,
                                               m_max_square_length,
                                               Direction::Backward,
                                               m_nodes,
                                               m_end_nodes);
    goal_trees.at(idx)->setHumanFilter(m_human_filter);
  }

  unsigned int ien=0;

  while(m_best_path_per_goal.at(m_end_nodes.at(ien)).first<1.01*m_utopia_costs.at(m_end_nodes.at(ien)))
  {
    ROS_PROTO("check if end node %u (of %u) is direct connected",ien,m_end_nodes.size());
    if (++ien>=m_end_nodes.size())
      ien=0;
  }

  unsigned int goal_bias_counter=0;
  unsigned int goal_bias_max=2;
  while (!success && !fail)
  {

    if (m_best_cost<std::numeric_limits<double>::infinity())
    {
      if (iter++>50)
        fail=true;
    }
    else
    {
      if (iter++>1000)
        fail=true;
    }


    std::vector<double> joint=sample(m_end_nodes.at(ien));
    bool full_added_to_start_tree;
    bool find_connection_to_goal=false;
    NodePtr last_node_add_to_start_tree;
    ROS_PROTO("add a node to trees");

    if (++goal_bias_counter==goal_bias_max)
    {
      full_added_to_start_tree=start_tree->extend(m_end_nodes.at(ien),last_node_add_to_start_tree);
      if (full_added_to_start_tree)
        find_connection_to_goal=true;
      goal_bias_counter=0;
    }
    else
    {

      while(m_best_path_per_goal.at(m_end_nodes.at(ien)).first<1.01*m_utopia_costs.at(m_end_nodes.at(ien)))
      {
        if (++ien>=m_end_nodes.size())
          ien=0;
      }

      full_added_to_start_tree=start_tree->createAndExtend(joint,last_node_add_to_start_tree);
    }

    for (TreePtr& goal_tree: goal_trees)
    {
      NodePtr last_node_add_to_goal_tree;
      bool full_added_to_stop_tree;
      if (find_connection_to_goal)
      {
        full_added_to_stop_tree=true;
      }
      else
      {

        if (m_best_path_per_goal.at(goal_tree->getRoot()).first<1.01*m_utopia_costs.at(goal_tree->getRoot()))
          continue;

        full_added_to_stop_tree=goal_tree->extend(last_node_add_to_start_tree,last_node_add_to_goal_tree);

        if (!full_added_to_start_tree && !full_added_to_stop_tree)
        {
          if (start_tree->tryConnectWith(last_node_add_to_goal_tree))
          {
            last_node_add_to_start_tree=last_node_add_to_goal_tree;
            full_added_to_stop_tree=true;
          }
          else
          {
            goal_tree->createAndExtend(joint,last_node_add_to_goal_tree);

            if (goal_tree->tryConnectWith(last_node_add_to_start_tree))
            {
              full_added_to_stop_tree=true;
            }
            else if (start_tree->tryConnectWith(last_node_add_to_goal_tree))
            {
              last_node_add_to_start_tree=last_node_add_to_goal_tree;
              full_added_to_stop_tree=true;
            }
          }
        }
      }
      if (full_added_to_stop_tree)
      {

        Path path;

        //  ROS_DEBUG("insert in the path the branch of the start_tree");
        if (!start_tree->getPathToNode(last_node_add_to_start_tree,path))
        {
          ROS_ERROR("tree not complete");
        }

        // ROS_DEBUG("insert in the path the branch of the goal_tree");
        if (!goal_tree->getPathToNode(last_node_add_to_start_tree,path))
        {
          ROS_ERROR("tree not complete");
        }


        path=pruningPath(path);

        path=warpPath(1,path);


        //        path=splitPath(2,path);


        double cost=computePathCost(path);

        if (storeIfImproveCost(path,cost))
        {

          for (unsigned int igt=0;igt<m_goal_trees.size();igt++)
          {
            if (m_goal_trees.at(igt)->getRoot()==goal_tree->getRoot())
              m_goal_trees.at(igt)=goal_tree;
          }

          m_start_tree=start_tree;
        }

        paths.push_back(path);
        success=true;
      }
    }

  }


  for (const std::pair<NodePtr,std::pair<double,Path>> p: m_best_path_per_goal)
  {
    if (p.second.first<std::numeric_limits<double>::infinity())
      paths.push_back(p.second.second);
  }
  //  pruning(paths);

  if (!success)
    ROS_DEBUG("unable to find a solution");


  return success;

}

bool Net::isSolutionFound()
{
  ROS_PROTO("check if best cost is equal to the utopia cost");
  if (m_best_cost<1.01*m_utopia_cost)
    return true;

  ROS_PROTO("check goal by goal");
  for (const NodePtr end_node: m_end_nodes)
  {
    ROS_PROTO("check if best cost for each goal is equal to the correspoding utopia cost");
    try
    {
      if (m_best_path_per_goal.at(end_node).first>=std::numeric_limits<double>::infinity())
        return false;
    }
    catch(std::exception& e)
    {
      ROS_ERROR("what? %s, number of endnodes %zu, number of best path per goal  %zu",e.what(),m_end_nodes.size(),m_best_path_per_goal.size());

      throw e;
    }
  }
  return true;
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

    if (cost<0.99999*m_best_cost)
    {
      ROS_DEBUG("this path improves the overall best path (cost %f) delta=%e",cost,m_best_cost-cost);
      m_best_cost=cost;
      m_best_path=path;
    }
    else
      ROS_DEBUG("this path improved the best path to goal (cost %f)",cost);
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
        {
          old_length+=path.at(idx)->getLength();
        }
        if (shortcut_connection->getLength()<old_length)
        {
          if (!is_alreay_present)
          {
            shortcut_connection->registerConnection();
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

Path Net::localSearch(const unsigned int& number_of_trials, const Path& path)
{
  double old_cost=computePathCost(path);
  double best_cost=old_cost;
  Path best_path=path;
  for (unsigned int i_trial=0;i_trial<number_of_trials;i_trial++)
  {

//    ROS_PROTO("TRIAL = %u", i_trial);

    for (unsigned int idx=0;idx<(path.size()-1);idx++)
    {
      Path out_path= best_path;
      Path in_path=  best_path;
      Path new_path= best_path;

      NodePtr node1=best_path.at(idx)->getParent();
      NodePtr node2=best_path.at(idx)->getChild();
      NodePtr node3=best_path.at(idx+1)->getChild();

      ConnectionPtr tmp_conn;
      assert(node2->checkIfConnectedWith(node3,tmp_conn));

      Eigen::Map<const Eigen::VectorXd> p1(node1->getJoints().data(),m_dof);
      Eigen::Map<const Eigen::VectorXd> p3(node3->getJoints().data(),m_dof);
      Eigen::Map<const Eigen::VectorXd> p2(node2->getJoints().data(),m_dof);
      Eigen::VectorXd v13=(p3-p1).normalized();

      Eigen::VectorXd v12;
      Eigen::VectorXd v23;
      Eigen::VectorXd prominence;



      v12=best_path.at(idx)->versor();
      v23=best_path.at(idx+1)->versor();
      prominence=v12-(v12.dot(v13))*v13;
      prominence=prominence.normalized();
      if (i_trial>0)
      {
        Eigen::VectorXd pert(prominence.size());
        pert.setRandom();
        prominence+=pert.normalized();
        prominence=prominence.normalized();
      }
      else if (prominence.norm()<0.001)
      {
        prominence.setRandom();
        prominence=prominence.normalized();
      }


      double distance=std::abs((p2-p1).dot(prominence));
      double step=std::max(distance*0.05,m_min_length);


      Eigen::VectorXd p2in  = p2-step*prominence;
      Eigen::VectorXd p2out = p2+step*prominence;

      std::vector<double> p(m_dof,0);
      for (unsigned idof=0;idof<m_dof;idof++)
        p.at(idof)=p2in(idof);
      NodePtr in_node=addNodeToTheGrid(p,node2);
      if (m_human_filter)
        computeOccupancy(in_node);

      for (unsigned idof=0;idof<m_dof;idof++)
        p.at(idof)=p2out(idof);
      NodePtr out_node=addNodeToTheGrid(p,node2);
      if (m_human_filter)
        computeOccupancy(out_node);

      ConnectionPtr conn_12_in;
      if (!in_node->checkIfConnectedWith(node1,conn_12_in))
        ROS_ERROR("Thes nodes should be connected");
      ConnectionPtr conn_23_in;
      if (!in_node->checkIfConnectedWith(node3,conn_23_in))
        ROS_ERROR("Thes nodes should be connected");
      in_path.at(idx)=conn_12_in;
      in_path.at(idx+1)=conn_23_in;
      double in_cost=computePathCost(in_path);
      if (isCollisionFree(in_path))
        storeIfImproveCost(in_path,in_cost);
      else
        in_cost=std::numeric_limits<double>::infinity();

      ConnectionPtr conn_12_out;
      if (!out_node->checkIfConnectedWith(node1,conn_12_out))
        ROS_ERROR("Thes nodes should be connected");
      ConnectionPtr conn_23_out;
      if (!out_node->checkIfConnectedWith(node3,conn_23_out))
        ROS_ERROR("Thes nodes should be connected");
      out_path.at(idx)=conn_12_out;
      out_path.at(idx+1)=conn_23_out;
      double out_cost=computePathCost(out_path);
      if (isCollisionFree(out_path))
        storeIfImproveCost(out_path,out_cost);
      else
        out_cost=std::numeric_limits<double>::infinity();


      if ( (in_cost>best_cost) && (out_cost>best_cost) )
        continue;

      Eigen::VectorXd p2new;
      bool direction_flag=out_cost<in_cost;
      if (direction_flag)
      {
        best_cost=out_cost;
        best_path=out_path;
        p2new=p2out;
        node2=out_node;
        removeNodeWithConnections(in_node);
      }
      else
      {
        best_cost=in_cost;
        best_path=in_path;
        p2new=p2in;
        node2=in_node;
        removeNodeWithConnections(out_node);
      }

      unsigned int iter=0;
      while(iter<100)
      {
        p2new  = p2new+(direction_flag? step: -step)*prominence;
        for (unsigned idof=0;idof<m_dof;idof++)
          p.at(idof)=p2new(idof);
        NodePtr new_node=addNodeToTheGrid(p,node2);

        ConnectionPtr conn_12_new;
        if (!new_node->checkIfConnectedWith(node1,conn_12_new))
          ROS_ERROR("Thes nodes should be connected");
        ConnectionPtr conn_23_new;
        if (!new_node->checkIfConnectedWith(node3,conn_23_new))
          ROS_ERROR("Thes nodes should be connected");
        new_path.at(idx)=conn_12_new;
        new_path.at(idx+1)=conn_23_new;
        double new_cost=computePathCost(new_path);
        if (isCollisionFree(new_path))
          storeIfImproveCost(new_path,new_cost);
        else
          new_cost=std::numeric_limits<double>::infinity();


        if (new_cost<best_cost)
        {
          iter++;
          best_cost=new_cost;
          best_path=new_path;
          node2=new_node;
        }
        else
        {
          removeNodeWithConnections(new_node);
          break;
        }

      }
    }
    if (best_cost>=0.999*old_cost)
      break;
    else
      old_cost=best_cost;
  }

  return best_path;
}

bool Net::localSearch2(const unsigned int& number_of_trials)
{
  double old_cost=m_best_cost;
  localSearch(number_of_trials,m_best_path);
  return m_best_cost<old_cost;
}

Path Net::dividePath(const Path &path, const double& desired_distance)
{
  Path new_path;
  for (unsigned int ic=0;ic<path.size();ic++)
  {
    double connection_length=std::sqrt(path.at(ic)->getLength());
    unsigned int n_pnts=std::ceil(connection_length/desired_distance)+1;

    if (n_pnts<=2)
    {
      new_path.push_back(path.at(ic));
      continue;
    }

    NodePtr first_node=path.at(ic)->getParent();
    NodePtr second_node=path.at(ic)->getChild();
    NodePtr last_add_node=first_node;
    NodePtr new_node;
    for (unsigned int ipnt=1;ipnt<=n_pnts;ipnt++)
    {

      if (ipnt<n_pnts)
      {
        std::vector<double> qstep(m_dof);
        for (size_t i_dof=0;i_dof<m_dof;i_dof++)
        {
          qstep.at(i_dof)=first_node->getJoints().at(i_dof)+(second_node->getJoints().at(i_dof)-first_node->getJoints().at(i_dof))*((double)ipnt)/((double)n_pnts);
        }
        new_node=std::make_shared<Node>(qstep,m_node_params,m_conn_params);
        computeOccupancy(new_node);
      }
      else
      {
        new_node=second_node;
      }
      assert(new_node->isInCollision(m_planning_scene));

      ConnectionPtr new_conn=std::make_shared<Connection>(last_add_node,new_node,m_conn_params);
      new_conn->registerConnection();
      assert(new_conn->isInCollision(m_planning_scene));
      last_add_node=new_node;
      new_path.push_back(new_conn);
    }


  }

  printPath(new_path);
  ROS_PROTO("old path size = %zu, new = %zu",path.size(),new_path.size());
  return new_path;
}

void Net::computeOccupancy(NodePtr& node)
{
  if (m_human_filter)
  {
    Eigen::Map<const Eigen::VectorXd> q(node->getJoints().data(),m_dof);
    node->setCost(m_human_filter->occupancy(q*m_unscaling));;
  }
}
}

