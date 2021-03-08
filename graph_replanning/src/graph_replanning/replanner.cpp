#include "graph_replanning/replanner.h"

namespace pathplan
{
Replanner::Replanner(Eigen::VectorXd& current_configuration,
                     PathPtr& current_path,
                     std::vector<PathPtr>& other_paths,
                     const TreeSolverPtr &solver,
                     const MetricsPtr& metrics,
                     const CollisionCheckerPtr& checker,
                     const Eigen::VectorXd& lb,
                     const Eigen::VectorXd& ub)
{
  current_configuration_ = current_configuration;
  current_path_ = current_path;
  other_paths_ = other_paths;

  assert(solver);

  solver_ = solver;
  metrics_ = metrics;
  checker_ = checker;
  lb_ = lb;
  ub_ = ub;

  admissible_other_paths_ = other_paths_;
  success_ = 0;
  emergency_stop_ = false;
  available_time_ =  std::numeric_limits<double>::infinity();
  pathSwitch_cycle_time_mean_ = std::numeric_limits<double>::infinity();
  informedOnlineReplanning_cycle_time_mean_ = std::numeric_limits<double>::infinity();
  an_obstacle_ = 0;

  informedOnlineReplanning_disp_ = false;
  pathSwitch_disp_ = false;

  informedOnlineReplanning_verbose_ = false;
  pathSwitch_verbose_ = false;

}

bool Replanner::checkPathValidity(const CollisionCheckerPtr &this_checker)
{
  CollisionCheckerPtr checker = checker_;
  if(this_checker != NULL) checker = this_checker;

  bool validity = true;

  if(!current_path_->isValid(checker)) validity = false;

  for(const PathPtr& path: other_paths_)
  {
    if(!path->isValid(checker)) validity = false;
  }

  return validity;
}

bool Replanner::simplifyReplannedPath(const double& distance)
{
  for(unsigned int i=0;i<replanned_path_->getConnections().size();i++)
  {
    if(replanned_path_->getConnections().at(i)->norm()<distance)
    {
      //ROS_INFO_STREAM("conn number: "<<i<<" length: "<<replanned_path_->getConnections().at(i)->norm());
    }
  }
  int count = 0;

  bool simplify = false;
  bool simplified = false;
  do
  {
    simplify = replanned_path_->simplify(distance);
    if(simplify)
    {
      count ++;
      simplified = true;
    }
  }
  while(simplify);

  return simplified;
}

void Replanner::startReplannedPathFromNewCurrentConf(Eigen::VectorXd &configuration)
{

  std::vector<pathplan::ConnectionPtr> path_connections;
  std::vector<pathplan::ConnectionPtr> connections;

  pathplan::PathPtr path = std::make_shared<pathplan::Path>(replanned_path_->getConnections(),metrics_,checker_);
  pathplan::NodePtr current_node = std::make_shared<pathplan::Node>(configuration);
  pathplan::NodePtr path_start = path->getConnections().front()->getParent();

  for(const Eigen::VectorXd wp:path->getWaypoints())
  {
    if(wp == configuration)
    {
      if(wp == path->getWaypoints().back()) assert(0);
      replanned_path_ = path->getSubpathFromNode(current_node);
      return;
    }
  }

  int idx_current_conf, idx_path_start;
  double abscissa_current_conf = current_path_->curvilinearAbscissaOfPoint(configuration,idx_current_conf);
  double abscissa_path_start = current_path_->curvilinearAbscissaOfPoint(path_start->getConfiguration(),idx_path_start);

  if(abscissa_current_conf == abscissa_path_start) return;  //the start of the replanned path is the current configuration
  else if(abscissa_current_conf < abscissa_path_start)  //the replanned path starts from a position after the current one
  {
    if(idx_current_conf == idx_path_start)
    {
      //Directly connect the current configuration with the start of the replanned path
      ConnectionPtr conn = std::make_shared<Connection>(current_node, path_start);
      double cost_conn = metrics_->cost(configuration,path_start->getConfiguration());
      conn->setCost(cost_conn);
      conn->add();

      connections.push_back(conn);
    }
    if(idx_current_conf < idx_path_start)
    {
      NodePtr child = current_path_->getConnections().at(idx_current_conf)->getChild();
      if(child->getConfiguration() != configuration)
      {
        ConnectionPtr conn = std::make_shared<Connection>(current_node, child);
        double cost_conn = metrics_->cost(configuration,child->getConfiguration());
        conn->setCost(cost_conn);
        conn->add();

        connections.push_back(conn);
      }

      //Adding the connections between the two configurations
      for(unsigned int z = idx_current_conf+1; z<idx_path_start; z++) connections.push_back(current_path_->getConnections().at(z));

      if(path_start->getConfiguration() == current_path_->getConnections().at(idx_path_start)->getChild()->getConfiguration())
      {
        connections.push_back(current_path_->getConnections().at(idx_path_start));
      }
      else
      {
        NodePtr parent = current_path_->getConnections().at(idx_path_start)->getParent();
        if(parent->getConfiguration() != path_start->getConfiguration())
        {
          ConnectionPtr conn = std::make_shared<Connection>(parent,path_start);
          double cost_conn = metrics_->cost(parent->getConfiguration(),path_start->getConfiguration());
          conn->setCost(cost_conn);
          conn->add();

          connections.push_back(conn);
        }
      }
    }

    for(const ConnectionPtr& replanned_connection:path->getConnections()) connections.push_back(replanned_connection);
    path->setConnections(connections);
    replanned_path_ = path;

    return;
  }
  else //the replanned path starts from a position before the current configuration
  {
    pathplan::NodePtr node;

    int idx_conn;
    if(path->findConnection(configuration,idx_conn) != NULL)
    {
      node = path->getConnections().at(idx_conn)->getChild();

      if((path->getConnections().size()-1) > idx_conn)
      {
        for(unsigned int i=idx_conn+1; i<path->getConnections().size();i++) path_connections.push_back(path->getConnections().at(i));
      }
      for(unsigned int i=0; i<=idx_conn; i++) path->getConnections().at(i)->remove();

      if(current_node->getConfiguration() != node->getConfiguration())
      {
        pathplan::ConnectionPtr conn = std::make_shared<pathplan::Connection>(current_node,node);
        double cost = metrics_->cost(current_node,node);
        conn->setCost(cost);
        conn->add();

        connections.push_back(conn);
      }
      if((path->getConnections().size()-1) > idx_conn) connections.insert(connections.end(),path_connections.begin(),path_connections.end());
      path->setConnections(connections);
    }
    else
    {
      ConnectionPtr conn = current_path_->getConnections().at(idx_path_start);

      int idx = idx_path_start;
      if(path_start->getConfiguration() == current_path_->getConnections().at(idx_path_start)->getChild()->getConfiguration()) idx = idx_path_start + 1;

      int j = 0;
      int j_save = -2;
      for(unsigned int i=idx; i<current_path_->getConnections().size();i++)
      {
        if(path->getConnections().at(j)->getChild()->getConfiguration() == current_path_->getConnections().at(i)->getChild()->getConfiguration())
        {
          node = path->getConnections().at(j)->getChild();
          j_save = j;
        }
        else break;
        j+=1;
      }

      bool add_conn = false;
      if(j_save != -2)
      {
        for(unsigned int i=j_save+1; i<path->getConnections().size();i++) path_connections.push_back(path->getConnections().at(i));
        for(unsigned int i=0; i<=j_save; i++) path->getConnections().at(i)->remove();
      }
      else
      {
        if((conn->getParent()->getConfiguration() == path_start->getConfiguration()) || (conn->getChild()->getConfiguration() == path_start->getConfiguration()))
        {
          node = path_start;
          path_connections = path->getConnections();
        }
        else
        {
          if(idx_current_conf == idx_path_start) node = current_node;
          else node = current_path_->getConnections().at(idx_path_start)->getChild();
          path_connections = path->getConnections();
          add_conn = true;
        }
      }

      bool connected = false;
      if(add_conn && idx_path_start == idx_current_conf) connected = true;  //if the current conf is near after the pat start, you only have to connect these two confs

      int t = idx_current_conf;
      pathplan::NodePtr child;
      pathplan::NodePtr parent;

      while(!connected)
      {
        if(t == idx_current_conf)
        {
          child = current_node;
        }
        else
        {
          child = std::make_shared<pathplan::Node>(current_path_->getConnections().at(t)->getChild()->getConfiguration());
        }
        parent = std::make_shared<pathplan::Node>(current_path_->getConnections().at(t)->getParent()->getConfiguration());

        if(child->getConfiguration() != node->getConfiguration())
        {
          pathplan::ConnectionPtr conn = std::make_shared<pathplan::Connection>(child,parent); //you re moving backwards
          double cost = metrics_->cost(child,parent);
          conn->setCost(cost);
          conn->add();

          connections.push_back(conn);
        }
        else connected = true;

        t-=1;
      }

      if(add_conn)
      {
        pathplan::ConnectionPtr conn = std::make_shared<pathplan::Connection>(node,path_start); //you are moving backwards
        double cost = metrics_->cost(node,path_start);
        conn->setCost(cost);
        conn->add();

        connections.push_back(conn);
      }
      connections.insert(connections.end(),path_connections.begin(),path_connections.end());
      path->setConnections(connections);
    }

    replanned_path_ = path;
  }
}

bool Replanner::connect2goal(const PathPtr& current_path, const NodePtr& node, PathPtr &new_path)
{
  std::vector<double> marker_color = {1.0,1.0,0.0,1.0};
  std::vector<double> marker_scale(3,0.01);
  int id;

  ros::WallTime tic=ros::WallTime::now();
  ros::WallTime toc;
  double max_time;
  if(pathSwitch_disp_) max_time = std::numeric_limits<double>::infinity();
  else max_time = available_time_;
  double time = max_time;
  double initial_value = max_time;
  double percentage_variability = 0.8;
  std::vector<double> time_vector;

  if(pathSwitch_verbose_) ROS_INFO_STREAM("Connect2Goal cycle starting mean: "<<pathSwitch_cycle_time_mean_);

  if(an_obstacle_ || pathSwitch_cycle_time_mean_ == std::numeric_limits<double>::infinity())
  {
    pathSwitch_cycle_time_mean_ = initial_value;
    if(pathSwitch_verbose_) ROS_INFO_STREAM("new mean: "<<pathSwitch_cycle_time_mean_);
  }
  else
  {
    time_vector.push_back(pathSwitch_cycle_time_mean_);
  }

  if(time<=0.0 || time<percentage_variability*pathSwitch_cycle_time_mean_) return false;  //nb: ora time == max time quindi time<=percentage_variability*pathSwitch_cycle_time_mean_ con pathSwitch_cycle_time_mean == initial_value è sempre verificato

  bool success = 0;

  NodePtr goal = current_path->getConnections().back()->getChild();
  PathPtr subpath1 = current_path->getSubpathFromNode(node);
  double distance_path_node = (node->getConfiguration()-goal->getConfiguration()).norm();

  if (distance_path_node < subpath1->cost())
  {
    NodePtr node_fake = std::make_shared<Node>(node->getConfiguration());
    NodePtr goal_fake = std::make_shared<Node>(goal->getConfiguration());

    SamplerPtr sampler = std::make_shared<InformedSampler>(node_fake->getConfiguration(), goal_fake->getConfiguration(), lb_, ub_,subpath1->cost());

    solver_->setSampler(sampler);
    solver_->resetProblem();
    solver_->addStart(node_fake);

    toc = ros::WallTime::now();
    if(pathSwitch_cycle_time_mean_ == initial_value) time = initial_value-(toc-tic).toSec();  //when there is an obstacle or when the cycle time mean has not been defined yet
    else time = (2-percentage_variability)*pathSwitch_cycle_time_mean_-(toc-tic).toSec();
    ros::WallTime tic_setProblem = ros::WallTime::now();

    if(pathSwitch_verbose_) ROS_INFO("Searching for a direct connection...");
    solver_->addGoal(goal_fake,time);
    bool directly_connected = solver_->solved();

    if(pathSwitch_verbose_)
    {
      if(directly_connected) ROS_INFO("Solved->direct connection found");
      else ROS_INFO("Direct connection NOT found");

      ros::WallTime toc_setProblem = ros::WallTime::now();
      ROS_INFO_STREAM("Time to directly connect: "<<(toc_setProblem-tic_setProblem).toSec());
    }

    bool solver_has_solved;
    if(directly_connected)
    {
      new_path = solver_->getSolution();
      solver_has_solved = true;
    }

    ros::WallTime tic_solver = ros::WallTime::now();
    if(!directly_connected)
    {
      toc = ros::WallTime::now();
      if(pathSwitch_cycle_time_mean_ == initial_value) time = initial_value-(toc-tic).toSec();  //when there is an obstacle or when the cycle time mean has not been defined yet
      else time = (2-percentage_variability)*pathSwitch_cycle_time_mean_-(toc-tic).toSec();

      if(pathSwitch_verbose_) ROS_INFO_STREAM("solving...max time: "<<time);

      solver_has_solved = solver_->solve(new_path,1000,time);
    }

    if (solver_has_solved)
    {
      if(!directly_connected)
      {
        if(pathSwitch_verbose_)
        {
          ros::WallTime toc_solver = ros::WallTime::now();
          ROS_INFO_STREAM("max solver time: "<<time<<" used time: "<<(toc_solver-tic_solver).toSec());
        }

        PathLocalOptimizer path_solver(checker_, metrics_);
        path_solver.setPath(new_path);

        toc = ros::WallTime::now();
        if(pathSwitch_cycle_time_mean_ == initial_value) time = initial_value-(toc-tic).toSec();
        else time = (2-percentage_variability)*pathSwitch_cycle_time_mean_-(toc-tic).toSec();
        ros::WallTime tic_opt = ros::WallTime::now();

        path_solver.solve(new_path,10,time);

        if(pathSwitch_verbose_)
        {
          ros::WallTime toc_opt = ros::WallTime::now();
          ROS_INFO_STREAM("max opt time: "<<time<<" used time: "<<(toc_opt-tic_opt).toSec());
        }
      }

      std::vector<ConnectionPtr> new_path_conn = new_path->getConnections();
      std::vector<ConnectionPtr> new_connections;

      if(pathSwitch_verbose_) ROS_INFO_STREAM("solution cost: "<<new_path->cost());

      if(new_path->cost()<subpath1->cost())
      {
        if(new_path_conn.size()>1)
        {
          NodePtr node1 = new_path_conn.front()->getChild();

          double conn1_cost = metrics_->cost(node,node1);

          ConnectionPtr conn1 = std::make_shared<Connection>(node,node1);
          conn1->setCost(conn1_cost);
          conn1->add();

          new_connections.push_back(conn1);
          new_connections.insert(new_connections.end(),new_path_conn.begin()+1,new_path_conn.end());
          new_path_conn.front()->remove();
        }
        else
        {
          double conn1_cost =  metrics_->cost(node,goal_fake);

          ConnectionPtr conn1 = std::make_shared<Connection>(node,goal_fake);
          conn1->setCost(conn1_cost);
          conn1->add();

          new_path_conn.front()->remove();
          new_connections.push_back(conn1);
        }
        node_fake->disconnect();

        new_path = std::make_shared<Path>(new_connections, metrics_, checker_);
        success = 1;
        an_obstacle_ = false;
      }
      else if(pathSwitch_verbose_) ROS_INFO("It is not a better solution");

      toc = ros::WallTime::now();
      time_vector.push_back((toc-tic).toSec());
      if(pathSwitch_verbose_) ROS_INFO_STREAM("SOLVED->cycle time: "<<(toc-tic).toSec());
      pathSwitch_cycle_time_mean_ = std::accumulate(time_vector.begin(), time_vector.end(),0.0)/((double) time_vector.size());
    }
    else if(pathSwitch_verbose_) ROS_INFO("not solved");
  }

  if(pathSwitch_verbose_) ROS_INFO_STREAM("cycle time mean: "<<pathSwitch_cycle_time_mean_);

  if(pathSwitch_verbose_)
  {
    ROS_INFO_STREAM("Connect2Goal duration: "<<(ros::WallTime::now()-tic).toSec());
    if(success)
    {
      ROS_INFO_STREAM("Connect2Goal has found a solution with cost: " << new_path->cost());
    }
    else
    {
      ROS_INFO_STREAM("Connect2Goal has NOT found a solution");
    }
  }

  /*///////////////////////////////Visualization//////////////////////////////////*/
  if(pathSwitch_disp_)
  {
    disp_->clearMarker(id);
    disp_->changeConnectionSize(marker_scale);
    id = disp_->displayPath(new_path,"pathplan",marker_color);
    disp_->defaultConnectionSize();
    disp_->nextButton("Press \"next\" to execute the next step of Connect2Goal");
  }
  /*/////////////////////////////////////////////////////////////////////////////*/

  return success;
}

std::vector<PathPtr>  Replanner::addAdmissibleCurrentPath(const int &idx_current_conn,
                                                          PathPtr& admissible_current_path)
{
  std::vector<PathPtr> reset_other_paths;

  if(current_path_->cost() == std::numeric_limits<double>::infinity())  //if the path is obstructed by an obstacle, the connection obstructed cost is infinte
  {
    if(current_path_->getConnections().back()->getCost() == std::numeric_limits<double>::infinity())
    {
      admissible_current_path = NULL; //there are no connections between the obstacle and the goal
    }
    else
    {
      int z = current_path_->getConnections().size()-2;  //penultimate connection (last connection is at end-1)
      ConnectionPtr conn;

      while(z>=idx_current_conn) //to find the savable part of current_path, the subpath after the connection obstruced by the obstacle
      {
        conn = current_path_->getConnections().at(z);

        if(conn->getCost() == std::numeric_limits<double>::infinity())
        {
          admissible_current_path = current_path_->getSubpathFromNode(conn->getChild());
          z = -1;
        }
        else
        {
          z = z-1;
        }
      }
    }

    if(admissible_current_path != NULL)
    {
      // adding the savable subpath of the current_path to the set of available paths
      reset_other_paths.push_back(admissible_current_path);
      reset_other_paths.insert(reset_other_paths.end(),other_paths_.begin(),other_paths_.end());
    }
    else
    {
      reset_other_paths = other_paths_;
    }
  }
  else
  {
    reset_other_paths.push_back(current_path_);
    reset_other_paths.insert(reset_other_paths.end(),other_paths_.begin(),other_paths_.end());
  }

  return reset_other_paths;
}


bool Replanner::pathSwitch(const PathPtr &current_path,
                           const NodePtr &node,
                           const bool &succ_node,
                           PathPtr &new_path,
                           PathPtr &subpath_from_path2,
                           int &connected2path_number)
{
  ros::WallTime tic=ros::WallTime::now();
  ros::WallTime toc, tic_cycle, toc_cycle;
  double max_time;
  if(pathSwitch_disp_) max_time = std::numeric_limits<double>::infinity();
  else max_time = available_time_;
  double time = max_time;
  std::vector<double> time_vector;
  bool time_out = false;
  double initial_value = max_time;
  double percentage_variability = 0.8;

  if(pathSwitch_verbose_) ROS_INFO_STREAM("PathSwitch cycle starting mean: "<<pathSwitch_cycle_time_mean_);

  if(an_obstacle_ || pathSwitch_cycle_time_mean_ == std::numeric_limits<double>::infinity())
  {
    pathSwitch_cycle_time_mean_ = initial_value;
    if(pathSwitch_verbose_) ROS_INFO_STREAM("new mean: "<<pathSwitch_cycle_time_mean_);
  }
  else
  {
    time_vector.push_back(pathSwitch_cycle_time_mean_);
  }

  if(time<=0.0 || time<percentage_variability*pathSwitch_cycle_time_mean_) return false;  //nb: ora time == max time quindi time<=percentage_variability*pathSwitch_cycle_time_mean_ con pathSwitch_cycle_time_mean == initial_value è sempre verificato

  int new_path_id;
  int new_node_id;
  std::vector<double> marker_scale_sphere(3,0.03);
  std::vector<double> marker_color_sphere = {0.0,0.0,0.0,1.0};

  std::vector<double> marker_color = {1.0,1.0,0.0,1.0};
  std::vector<double> marker_scale(3,0.01);

  // Identifying the subpath of current_path starting from node
  NodePtr path1_node = node;
  PathPtr path1_node2goal;
  std::vector<NodePtr> path2_node_vector;

  path1_node2goal = current_path->getSubpathFromNode(path1_node);

  double subpath1_cost = path1_node2goal->cost();

  double path_cost = subpath1_cost;
  bool success = 0;

  std::map<double,PathPtr> path_map;
  for(unsigned int j = 0; j< admissible_other_paths_.size(); j++)
  {
    double distance;
    admissible_other_paths_.at(j)->findCloserNode(node,distance);
    path_map.insert(std::pair<double,PathPtr>(distance,admissible_other_paths_.at(j)));
  }

  //MULTIGOAL sui nodi + vicini

  for(const std::pair<double,PathPtr>& p: path_map)
  {
    PathPtr path2 = p.second;

    //Finding the closest node
    PathPtr path2_support;
    if(path2->getConnections().size()>1)
    {
      path2_support = path2->getSubpathToNode(path2->getConnections().back()->getParent());
    }
    else
    {
      path2_support = path2;
    }

    path2_node_vector.clear();
    if(path2_support->getConnections().size()>1)
    {
      path2_node_vector.push_back(path2_support->findCloserNode(path1_node->getConfiguration()));
    }
    else
    {
      path2_node_vector.push_back(path2_support->getConnections().at(0)->getParent());
    }

    if(succ_node)
    {
      std::vector<ConnectionPtr> path2_conn = path2->getConnections();
      for(unsigned int t = 0; t < path2_conn.size(); t++) //the GOAL is not considered as a node to which directly connecting the path
      {
        if((path2_conn.at(t)->getParent()->getConfiguration()-path2_node_vector.back()->getConfiguration()).norm() > 1e-03 && (path2_conn.at(t)->getParent()->getConfiguration() != path2_node_vector.front()->getConfiguration())) //when some nodes are too close to each other, only one of them is considered
        {
          path2_node_vector.push_back(path2_conn.at(t)->getParent());
        }
      }
    }

    for(const NodePtr& conf_node : path2_node_vector)  //DA ELIMINARE
    {
      if(conf_node->getConfiguration() == path2->getConnections().back()->getChild()->getConfiguration()) assert(0);
    }

    for(const NodePtr& path2_node : path2_node_vector)
    {
      if(!emergency_stop_)
      {
        tic_cycle = ros::WallTime::now();

        PathPtr path2_node2goal ;
        path2_node2goal = path2->getSubpathFromNode(path2_node);

        std::vector<ConnectionPtr> subpath2 = path2_node2goal->getConnections();
        double subpath2_cost = path2_node2goal->cost();

        double diff_subpath_cost = path_cost - subpath2_cost; //it is the maximum cost to make the connecting_path convenient
        double distance_path_node = (path1_node->getConfiguration()-path2_node->getConfiguration()).norm(); //the Euclidean distance is the minimum cost that the connecting_path can have

        if(pathSwitch_disp_)
        {
          int node_n;
          for(unsigned int r=0; r<path2->getWaypoints().size();r++)
          {
            if(path2_node->getConfiguration() == path2->getWaypoints().at(r))
            {
              node_n = r;
            }
          }
          ROS_INFO_STREAM("node n: " <<node_n<< " diff_subpath_cost: "<< diff_subpath_cost<<" distance: " << distance_path_node);
        }

        if (diff_subpath_cost > 0 && distance_path_node < diff_subpath_cost) //if the Euclidean distance between the two nodes is bigger than the maximum cost for the connecting_path to be convenient, it is useless to calculate a connecting_path because it surely will not be convenient
        {
          NodePtr path1_node_fake = std::make_shared<Node>(path1_node->getConfiguration());
          NodePtr path2_node_fake = std::make_shared<Node>(path2_node->getConfiguration());

          SamplerPtr sampler = std::make_shared<InformedSampler>(path1_node_fake->getConfiguration(), path2_node_fake->getConfiguration(), lb_, ub_,diff_subpath_cost); //the ellipsoide determined by diff_subpath_cost is used. Outside from this elipsoid, the nodes create a connecting_path not convenient
          //sampler->setCost(diff_subpath_cost);

          solver_->setSampler(sampler);
          solver_->resetProblem();
          solver_->addStart(path1_node_fake);

          double solver_time;
          toc_cycle = ros::WallTime::now();
          toc = ros::WallTime::now();
          if(pathSwitch_cycle_time_mean_ == initial_value) solver_time = initial_value-(toc-tic).toSec();  //when there is an obstacle or when the cycle time mean has not been defined yet
          else solver_time = (2-percentage_variability)*pathSwitch_cycle_time_mean_-(toc_cycle-tic_cycle).toSec();
          ros::WallTime tic_setProblem = ros::WallTime::now();

          if(pathSwitch_verbose_) ROS_INFO("Searching for a direct connection...");
          solver_->addGoal(path2_node_fake,solver_time);
          double directly_connected = solver_->solved();

          if(pathSwitch_verbose_)
          {
            if(directly_connected) ROS_INFO("Solved->direct connection found");
            else ROS_INFO("Direct connection NOT found");

            ros::WallTime toc_setProblem = ros::WallTime::now();
            ROS_INFO_STREAM("Max time: "<<solver_time<<". Time to directly connect: "<<(toc_setProblem-tic_setProblem).toSec());
          }

          PathPtr connecting_path;
          bool solver_has_solved;
          if(directly_connected)
          {
            connecting_path = solver_->getSolution();
            solver_has_solved = true;
          }

          ros::WallTime tic_solver = ros::WallTime::now();
          if(!directly_connected)
          {
            toc_cycle = ros::WallTime::now();
            toc = ros::WallTime::now();
            if(pathSwitch_cycle_time_mean_ == initial_value) solver_time = initial_value-(toc-tic).toSec();  //when there is an obstacle or when the cycle time mean has not been defined yet
            else solver_time = (2-percentage_variability)*pathSwitch_cycle_time_mean_-(toc_cycle-tic_cycle).toSec();

            if(pathSwitch_verbose_) ROS_INFO_STREAM("solving...max time: "<<solver_time);

            solver_has_solved = solver_->solve(connecting_path,1000,solver_time);
          }
          if(solver_has_solved)
          {
            if(!directly_connected)
            {
              if(pathSwitch_verbose_)
              {
                ros::WallTime toc_solver = ros::WallTime::now();
                ROS_INFO_STREAM("max solver time: "<<solver_time<<" used time: "<<(toc_solver-tic_solver).toSec());
              }

              PathLocalOptimizer path_solver(checker_, metrics_);
              path_solver.setPath(connecting_path);

              double opt_time;
              toc_cycle = ros::WallTime::now();
              toc = ros::WallTime::now();
              if(pathSwitch_cycle_time_mean_ == initial_value) opt_time = initial_value-(toc-tic).toSec();
              else opt_time = (2-percentage_variability)*pathSwitch_cycle_time_mean_-(toc_cycle-tic_cycle).toSec();
              ros::WallTime tic_opt = ros::WallTime::now();

              path_solver.solve(connecting_path,10,opt_time);

              if(pathSwitch_verbose_)
              {
                ros::WallTime toc_opt = ros::WallTime::now();
                ROS_INFO_STREAM("max opt time: "<<opt_time<<" used time: "<<(toc_opt-tic_opt).toSec());
              }
            }

            std::vector<ConnectionPtr> connecting_path_conn = connecting_path->getConnections();
            std::vector<ConnectionPtr> new_connecting_path_conn;

            double conn_cost = subpath2_cost + connecting_path->cost();

            if(pathSwitch_verbose_) ROS_INFO_STREAM("solution cost: "<<conn_cost);

            if(conn_cost<path_cost && conn_cost<subpath1_cost && !subpath2.empty()) //maybe "if(conn_cost<path_cost)" is enough (the GOAL is always excluded and at the beginning subpath1_cost is path_cost)
            {
              if(connecting_path_conn.size()>1)
              {
                NodePtr node1 = connecting_path_conn.front()->getChild();
                NodePtr node2 = connecting_path_conn.back()->getParent();

                double conn1_cost = metrics_->cost(path1_node,node1);
                double conn2_cost = metrics_->cost(node2,path2_node);

                ConnectionPtr conn1 = std::make_shared<Connection>(path1_node,node1);
                conn1->setCost(conn1_cost);
                conn1->add();
                ConnectionPtr conn2 = std::make_shared<Connection>(node2,path2_node);
                conn2->setCost(conn2_cost);
                conn2->add();

                new_connecting_path_conn.push_back(conn1);
                if(connecting_path_conn.size()>2) new_connecting_path_conn.insert(new_connecting_path_conn.end(), connecting_path_conn.begin()+1, connecting_path_conn.end()-1);
                new_connecting_path_conn.push_back(conn2);

                connecting_path_conn.front()->remove();
                connecting_path_conn.back()->remove();
              }
              else
              {
                double conn1_cost =  metrics_->cost(path1_node,path2_node);

                ConnectionPtr conn1 = std::make_shared<Connection>(path1_node,path2_node);
                conn1->setCost(conn1_cost);
                conn1->add();

                connecting_path_conn.front()->remove();
                new_connecting_path_conn.push_back(conn1);
              }

              path1_node_fake->disconnect();
              path2_node_fake->disconnect();

              new_connecting_path_conn.insert(new_connecting_path_conn.end(),subpath2.begin(),subpath2.end());
              new_path = std::make_shared<Path>(new_connecting_path_conn, metrics_, checker_);
              path_cost = new_path->cost();

              success = 1;
              an_obstacle_ = false;
              subpath_from_path2 = path2_node2goal;
              for(unsigned int z=0; z<admissible_other_paths_.size();z++)
              {
                if(path2 == admissible_other_paths_.at(z)) connected2path_number = z;
              }

              /*//////////Visualization//////////////////*/
              if(pathSwitch_disp_)
              {
                disp_->clearMarker(new_path_id);
                disp_->changeConnectionSize(marker_scale);
                new_path_id = disp_->displayPath(new_path,"pathplan",marker_color);
                disp_->defaultConnectionSize();

                disp_->nextButton("Press \"next\" to execute the next PathSwitch step");
              }
              /*//////////////////////////////////////*/
            }
            else
            {
              if(pathSwitch_disp_)
              {
                /*//////////Visualization//////////////////*/
                if(pathSwitch_disp_)
                {
                  ROS_INFO_STREAM("It is not a better solution");

                  disp_->clearMarker(new_node_id);
                  disp_->changeNodeSize(marker_scale_sphere);
                  disp_->displayNode(path2_node,"pathplan",marker_color_sphere);
                  disp_->defaultNodeSize();

                  disp_->nextButton("Press \"next\" to execute the next PathSwitch step");
                }
                /*//////////////////////////////////////*/
              }
            }

            toc_cycle = ros::WallTime::now();
            time_vector.push_back((toc_cycle-tic_cycle).toSec());
            if(pathSwitch_verbose_) ROS_INFO_STREAM("SOLVED->cycle time: "<<(toc_cycle-tic_cycle).toSec());
            pathSwitch_cycle_time_mean_ = std::accumulate(time_vector.begin(), time_vector.end(),0.0)/((double) time_vector.size());
          }
          else
          {
            if(pathSwitch_verbose_) ROS_INFO("not solved");

            /*//////////Visualization//////////////////*/
            if(pathSwitch_disp_)
            {
              ROS_INFO("not solved");

              disp_->clearMarker(new_node_id);
              disp_->changeNodeSize(marker_scale_sphere);
              disp_->displayNode(path2_node,"pathplan",marker_color_sphere);
              disp_->defaultNodeSize();

              disp_->nextButton("Press \"next\" to execute the next PathSwitch step");
            }
            /*//////////////////////////////////////*/
          }
        }
        else
        {
          /*//////////Visualization//////////////////*/
          if(pathSwitch_disp_)
          {
            ROS_INFO_STREAM("It would not be a better solution");

            disp_->clearMarker(new_node_id);
            disp_->changeNodeSize(marker_scale_sphere);
            disp_->displayNode(path2_node,"pathplan",marker_color_sphere);
            disp_->defaultNodeSize();

            disp_->nextButton("Press \"next\" to execute the next PathSwitch step");
          }
          /*//////////////////////////////////////*/
        }

        toc=ros::WallTime::now();
        time = max_time - (toc-tic).toSec();
        if(pathSwitch_verbose_) ROS_INFO_STREAM("cycle time mean: "<<pathSwitch_cycle_time_mean_<<" -> available time: "<< time);

        toc=ros::WallTime::now();
        time = max_time - (toc-tic).toSec();
        if((!an_obstacle_ && time<percentage_variability*pathSwitch_cycle_time_mean_)|| time<=0.0)  //if there is an obstacle, you should use the entire available time to find a feasible solution
        {
          time_out = true;
          break;
        }

      }
      else break;
    }
    if(time_out)
    {
      if(pathSwitch_verbose_) ROS_INFO_STREAM("TIME OUT! max time: "<<max_time<<", time_available: "<<time<<", time needed for a new cycle: "<<percentage_variability*pathSwitch_cycle_time_mean_);
      break;
    }
  }

  if(pathSwitch_verbose_ || pathSwitch_disp_)
  {
    if(pathSwitch_verbose_) ROS_INFO_STREAM("PathSwitch duration: "<<(ros::WallTime::now()-tic).toSec());
    if(success)
    {
      ROS_INFO_STREAM("PathSwitch has found a solution with cost: " << new_path->cost());
    }
    else
    {
      ROS_INFO_STREAM("PathSwitch has NOT found a solution");
    }
  }

  return success;
}

bool Replanner::informedOnlineReplanning(const int& informed,
                                         const bool& succ_node,
                                         const double &max_time)
{
  ros::WallTime tic=ros::WallTime::now();
  ros::WallTime toc, tic_cycle, toc_cycle;
  double MAX_TIME;
  if(informedOnlineReplanning_disp_) MAX_TIME= std::numeric_limits<double>::infinity();
  else MAX_TIME = max_time;
  available_time_ = MAX_TIME;
  std::vector<double> time_vector;
  const double TIME_LIMIT = 0.85*MAX_TIME; //seconds
  const int CONT_LIMIT = 5;
  double percentage_variability = 0.8;

  if(informedOnlineReplanning_cycle_time_mean_ != std::numeric_limits<double>::infinity()) time_vector.push_back(informedOnlineReplanning_cycle_time_mean_);
  if(available_time_<=0.0) return false;

  std::vector<double> marker_scale_sphere(3,0.03);
  std::vector<double> marker_color_sphere = {0.0,0.0,0.0,1.0};

  std::vector<double> marker_color = {1.0,1.0,0.0,1.0};
  std::vector<double> marker_scale(3,0.01);
  int replanned_path_id;

  PathPtr new_path;
  PathPtr replanned_path;
  std::vector<PathPtr> replanned_path_vector;
  std::vector<PathPtr> reset_other_paths;
  std::vector<NodePtr> path1_node_vector;
  double replanned_path_cost = std::numeric_limits<double>::infinity();
  double previous_cost = current_path_->cost();
  bool success = 0;
  bool solved = 0;
  bool first_sol = 1; //flag to calculate the first solution time
  unsigned int cont = 0;  //to count the number of replanning without significant improvement in the final solution
  int index = -1;
  bool available_nodes;
  PathPtr subpath_from_path2;
  PathPtr confirmed_subpath_from_path2;
  int confirmed_connected2path_number;
  int connected2path_number;
  PathPtr admissible_current_path = NULL;
  NodePtr starting_node;
  bool no_available_paths = 1;
  PathPtr subpath1;
  std::vector<ConnectionPtr> subpath1_conn;

  examined_nodes_.clear();
  an_obstacle_ = false;  //it will be checked later

  int idx; //to save the index of the connection on which the current configuration is
  ConnectionPtr current_conn = current_path_->findConnection(current_configuration_,idx);

  if(idx<0) // ////DA ELIMINARE/////////////////////////////////////////////////
  {
    double dist1 = (current_configuration_ - current_path_->getWaypoints().at(1)).norm();
    double dist2 = (current_path_->getWaypoints().at(0) - current_path_->getWaypoints().at(1)).norm();
    ROS_INFO_STREAM("current conf: "<< current_configuration_.transpose()<< " dist1: "<<dist1);
    ROS_INFO_STREAM("start conf: "<< current_path_->getWaypoints().at(0).transpose() << " lenght_conn: "<<dist2);
    ROS_INFO_STREAM("DIFF: "<<std::abs(dist1-dist2));
    ROS_INFO_STREAM("idx: "<<idx);

    for(Eigen::VectorXd wp:current_path_->getWaypoints()) ROS_INFO_STREAM("WP: "<<wp.transpose());
    throw std::invalid_argument("idx < 0");
  } // //////////////////////////////////////////////////////////////////////////

  reset_other_paths = addAdmissibleCurrentPath(idx, admissible_current_path);
  admissible_other_paths_ = reset_other_paths;

  for(const PathPtr& path: admissible_other_paths_)
  {
    if(path->getConnections().back()->getCost() != std::numeric_limits<double>::infinity()) no_available_paths = 0;  //if there is a path with the last connection free it means that there is almost an available path to connect to
  }

  NodePtr child = current_conn->getChild();

  NodePtr actual_node = std::make_shared<Node>(current_configuration_);
  ConnectionPtr actual_node_conn = NULL;
  double actual_node_conn_cost = 0;
  std::vector<ConnectionPtr> conn;

  if(idx<current_path_->getConnections().size()-1)
  {
    subpath1 = current_path_->getSubpathFromNode(child);
    subpath1_conn =  subpath1->getConnections();
    if(subpath1->cost() == std::numeric_limits<double>::infinity()) an_obstacle_ = true;
  }

  if(current_configuration_ != child->getConfiguration())
  {
    actual_node_conn = std::make_shared<Connection>(actual_node,child);

    if(current_conn->getCost() == std::numeric_limits<double>::infinity())
    {
      if(!checker_->checkPath(actual_node->getConfiguration(), child->getConfiguration()))
      {
        actual_node_conn_cost = std::numeric_limits<double>::infinity();
        an_obstacle_ = true;
      }
      else actual_node_conn_cost = metrics_->cost(actual_node,child);
    }
    else actual_node_conn_cost = metrics_->cost(actual_node,child);

    actual_node_conn->setCost(actual_node_conn_cost);
    actual_node_conn->add();

    if(idx<current_path_->getConnections().size()-1)
    {
      conn.push_back(actual_node_conn);
      conn.insert(conn.end(),subpath1_conn.begin(),subpath1_conn.end());
    }
    else conn.push_back(actual_node_conn);
  }
  else conn = subpath1_conn;

  replanned_path = std::make_shared<Path>(conn,metrics_,checker_); // at the start, the replanned path is initialized with the subpath of the current path from the current config to the goal
  replanned_path_cost = replanned_path->cost();

  if(actual_node_conn_cost == std::numeric_limits<double>::infinity() || idx == current_path_->getConnections().size()-1) //if the obstacle is obstructing the current connection, the replanning must start from the current configuration
  {
    if(informed == 0)
    {
      bool rewire = 1;
      actual_node = current_path_->addNodeAtCurrentConfig(current_configuration_,current_conn,rewire); //with informed == 0 the node must be added to current_path_ because current_path is given to PathSwitch and not the replanned path
    }

    starting_node = actual_node;
    available_nodes = 0;
    path1_node_vector.push_back(actual_node);
  }
  else      //if the current connection is free, all the nodes between the current child to the parent of the connection obstructed are considered as starting points for the replanning
  {
    for(unsigned int i=0; i< subpath1_conn.size(); i++)
    {
      path1_node_vector.push_back(subpath1_conn.at(i)->getParent());
      if(subpath1_conn.at(i)->getCost() ==  std::numeric_limits<double>::infinity())
      {
        index = i;
        break;
      }
    }
    if(index == -1) path1_node_vector.pop_back();  // if the path is free, you can consider all the nodes but it is useless to consider the last one before the goal (it is already connected to the goal with a straight line)

    starting_node = child;
    available_nodes = 1;
  }

  int j = path1_node_vector.size()-1;
  if(index != -1 && j != index) assert(0);   // DA ELIMINARE, e anche l'uso di index

  bool flag_other_paths = 0;

  while(j>=0 && !emergency_stop_)
  {
    if(informedOnlineReplanning_disp_) ROS_INFO_STREAM("j: "<<j);

    tic_cycle = ros::WallTime::now();

    if(informed>0)
    {
      if(flag_other_paths == 1 && !no_available_paths) // if this flag is 1, a solution has been found, so the set of available path has to be updated: if the algorithm is considering a node on the subpath of path2, only the subpath after this node and not the whole path2 is considered. If there aren't available free paths this calculation are unnecessary
      {
        std::vector<Eigen::VectorXd> node_vector = confirmed_subpath_from_path2->getWaypoints();
        node_vector.pop_back();  // removing the goal from the vector

        int pos = -1;
        for (unsigned int k=0; k<node_vector.size(); k++)
        {
          if(path1_node_vector.at(j)->getConfiguration() == node_vector.at(k))
          {
            pos = k;
            break;
          }
        }

        if(pos>=0)
        {
          if(pos == 0)  flag_other_paths = 0;

          if(confirmed_connected2path_number<admissible_other_paths_.size()-1)
          {
            admissible_other_paths_.clear();

            admissible_other_paths_.insert(admissible_other_paths_.begin(),reset_other_paths.begin(),reset_other_paths.begin()+confirmed_connected2path_number);  //l'ultimo elemento è escluso
            admissible_other_paths_.push_back(confirmed_subpath_from_path2->getSubpathFromNode(node_vector.at(pos)));
            admissible_other_paths_.insert(admissible_other_paths_.end(),reset_other_paths.begin()+confirmed_connected2path_number+1,reset_other_paths.end());
          }
          else
          {
            admissible_other_paths_.clear();

            admissible_other_paths_.insert(admissible_other_paths_.begin(),reset_other_paths.begin(),reset_other_paths.begin()+confirmed_connected2path_number);
            admissible_other_paths_.push_back(confirmed_subpath_from_path2->getSubpathFromNode(node_vector.at(pos)));
          }
        }
        else
        {
          admissible_other_paths_ = reset_other_paths;
        }
      }

      toc = ros::WallTime::now();
      available_time_ = MAX_TIME - (toc-tic).toSec();
      double min_time_pathSwitch;
      if(an_obstacle_ || pathSwitch_cycle_time_mean_ == std::numeric_limits<double>::infinity()) min_time_pathSwitch = 0.0;
      else min_time_pathSwitch = percentage_variability*pathSwitch_cycle_time_mean_;

      if(informedOnlineReplanning_verbose_) ROS_INFO_STREAM("available time: "<<available_time_<<", min time for PathSwitch: "<<min_time_pathSwitch);

      if(available_time_>=min_time_pathSwitch && !emergency_stop_)
      {
        if(no_available_paths)
        {
          if(informedOnlineReplanning_disp_) ROS_INFO_STREAM("Launching Connect2Goal...");
          solved = connect2goal(replanned_path,path1_node_vector.at(j),new_path);
        }
        else
        {
          if(informedOnlineReplanning_disp_) ROS_INFO_STREAM("Launching PathSwitch...");
          solved = pathSwitch(replanned_path, path1_node_vector.at(j), succ_node, new_path, subpath_from_path2, connected2path_number);
        }
      }
      else solved = 0;
    }
    else
    {
      toc = ros::WallTime::now();
      available_time_ = MAX_TIME - (toc-tic).toSec();
      double min_time_pathSwitch;
      if(an_obstacle_ || pathSwitch_cycle_time_mean_ == std::numeric_limits<double>::infinity()) min_time_pathSwitch = 0.0;
      else min_time_pathSwitch = percentage_variability*pathSwitch_cycle_time_mean_;

      if(informedOnlineReplanning_verbose_) ROS_INFO_STREAM("available time: "<<available_time_<<", min time for PathSwitch: "<<min_time_pathSwitch);

      if(available_time_>=min_time_pathSwitch && !emergency_stop_)
      {
        if(no_available_paths)
        {
          if(informedOnlineReplanning_disp_) ROS_INFO_STREAM("Launching Connect2Goal...");
          solved = connect2goal(current_path_,path1_node_vector.at(j),new_path);
        }
        else
        {
          if(informedOnlineReplanning_disp_) ROS_INFO_STREAM("Launching PathSwitch...");
          solved = pathSwitch(current_path_, path1_node_vector.at(j), succ_node, new_path, subpath_from_path2, connected2path_number);
        }
      }
      else solved = 0;
    }

    path1_node_vector.at(j)->setAnalyzed(1);  //to set as ANALYZED the node just analyzed. In this way, it will not be analyzed again in this replanning procedure
    examined_nodes_.push_back(path1_node_vector.at(j));  //to save the analyzed nodes

    if(informedOnlineReplanning_disp_)
    {
      /*////////////////////////Visualization of analyzed nodes //////////////////////////////////////*/
      disp_->changeNodeSize(marker_scale_sphere);
      disp_->displayNode(examined_nodes_.back(),"pathplan",marker_color_sphere);
      disp_->defaultNodeSize();
      /*/////////////////////////////////////////////////////////////////////////////////////////////*/

      ROS_INFO_STREAM("Solved: "<<solved);
    }

    if(solved)
    {
      PathPtr path;
      PathPtr subpath;
      std::vector<ConnectionPtr> path_conn;

      if(available_nodes) //calculating the cost of the replanned path found
      {
        if(path1_node_vector.at(j)->getConfiguration() != child->getConfiguration())
        {
          if(actual_node_conn != NULL) path_conn.push_back(actual_node_conn);  //connection between the current config and the child of the current conn

          subpath =  subpath1->getSubpathToNode(path1_node_vector.at(j));  //path between the current connection child and the node analyzed now

          std::vector<ConnectionPtr> conn_sup = subpath->getConnections();
          path_conn.insert(path_conn.end(),conn_sup.begin(),conn_sup.end());

          conn_sup.clear();
          conn_sup = new_path->getConnections();
          path_conn.insert(path_conn.end(),conn_sup.begin(),conn_sup.end());

          path = std::make_shared<Path>(path_conn,metrics_,checker_);
        }
        else    //the node analyzed is the child of the current connection
        {
          if(actual_node_conn != NULL) path_conn.push_back(actual_node_conn);

          std::vector<ConnectionPtr> conn_sup =  new_path->getConnections();
          path_conn.insert(path_conn.end(),conn_sup.begin(),conn_sup.end());

          path = std::make_shared<Path>(path_conn,metrics_,checker_);
        }
      }
      else
      {
        path = new_path;
      }

      if(path->cost()<replanned_path_cost) //if the cost of the new solution found is better than the cost of the best solution found so far
      {
        if(informedOnlineReplanning_disp_) ROS_INFO_STREAM("new path found, cost: " << path->cost() <<" previous cost: " << replanned_path_cost);

        if(first_sol)
        {
          toc = ros::WallTime::now();
          time_first_sol_ = (toc - tic).toSec();
          time_replanning_ = time_first_sol_;
          first_sol = 0;  //0 when the time of the first solution found has been already saved
        }

        previous_cost = replanned_path_cost;
        replanned_path = path;
        replanned_path_cost = path->cost();

        if(!no_available_paths)  //PathSwitch called and not Connect2Goal
        {
          confirmed_connected2path_number = connected2path_number;  //to remember the vector index of the path to which the algoithm has created a connection
          confirmed_subpath_from_path2 = subpath_from_path2;        //to save the subpath of the path to which the algoritm has created a connection
        }

        success = 1;
        flag_other_paths = 1;
        an_obstacle_ = false;
        if(replanned_path->cost() == std::numeric_limits<double>::infinity()) assert(0);  //se ho trovato una soluzione è perchè il costo è minore, quindi non può essere infinito

        if(replanned_path_vector.size()<10) //the algorithm gives as output the vector of the best 10 solutions found
        {
          replanned_path_vector.push_back(replanned_path);
        }
        else
        {
          std::vector<PathPtr> support_vector;
          support_vector.insert(support_vector.begin(),replanned_path_vector.begin()+1,replanned_path_vector.end());
          support_vector.push_back(replanned_path);

          replanned_path_vector = support_vector;
        }

        if(informed == 2 && available_nodes == 0 && starting_node == actual_node)  // when actual conn is obstructed, a path has been found and informed is 2 -> PathSwitch will be called fron the nodes of the new path found
        {
          actual_node_conn = replanned_path->getConnections().at(0);
          child = actual_node_conn->getChild();
          available_nodes = 1;
        }

        if(informedOnlineReplanning_disp_)
        {
          /*//////////////////////////Visualization////////////////////////////////////*/
          disp_->clearMarker(replanned_path_id);
          disp_->changeConnectionSize(marker_scale);
          replanned_path_id = disp_->displayPath(replanned_path,"pathplan",marker_color);
          disp_->defaultConnectionSize();
          /*/////////////////////////////////////////////////////////////////////////*/
        }

        toc = ros::WallTime::now();

        if((toc-tic).toSec()>TIME_LIMIT && cont >= CONT_LIMIT)
        {
          j = -1;
          break;
        }
        else
        {
          if((previous_cost-replanned_path_cost)<0.05*previous_cost) cont = cont+1;
          else cont = 0;
        }
      }
      else
      {
        if(informedOnlineReplanning_disp_) ROS_INFO_STREAM("NO better path found, cost: " << path->cost() <<" previous cost: " << replanned_path_cost);
      }

      toc_cycle = ros::WallTime::now();
      time_vector.push_back((toc_cycle-tic_cycle).toSec());
      informedOnlineReplanning_cycle_time_mean_ = std::accumulate(time_vector.begin(), time_vector.end(),0.0)/((double) time_vector.size());
      if(informedOnlineReplanning_verbose_) ROS_INFO_STREAM("Solution with cost "<<replanned_path_cost<<" found!->Informed cycle duration: "<<(toc_cycle-tic_cycle).toSec());
    }

    if(informed == 2 && success && j == 0)
    {
      if(child->getConfiguration() != current_path_->getWaypoints().back())
      {
        subpath1 = replanned_path->getSubpathFromNode(child);
        path1_node_vector.clear();
        for(unsigned int r=0; r<subpath1->getConnections().size()-1; r++)
        {
          if(subpath1->getConnections().at(r)->getParent()->getAnalyzed() == 0 && subpath1->getConnections().at(r)->getParent()->getNonOptimal() == 0) //Analyzed to check if they have been already analyzed (if 0 not not analyzed), nonOptimal to check if they are useful to improve the replanning solution (if 0, maybe they can improve the solution)
          {
            // the nodes of the new solution found are added to the set of the nodes to be analyzed
            path1_node_vector.push_back(subpath1->getConnections().at(r)->getParent());
          }
        }
        j = path1_node_vector.size();  //then, j=j-1

        if(informedOnlineReplanning_disp_) ROS_INFO_STREAM("NEW J: "<<j-1);
      }
    }

    if(informedOnlineReplanning_verbose_) ROS_INFO_STREAM("informed cycle time mean: "<< informedOnlineReplanning_cycle_time_mean_);

    toc = ros::WallTime::now();
    available_time_ = MAX_TIME-(toc-tic).toSec();

    if((!an_obstacle_ && available_time_<percentage_variability*informedOnlineReplanning_cycle_time_mean_) || available_time_<=0.0)
    {
      if(informedOnlineReplanning_verbose_) ROS_INFO_STREAM("TIME OUT! available time: "<<available_time_<<", time needed for a new cycle: "<<percentage_variability*informedOnlineReplanning_cycle_time_mean_);
      j = -1;
      break;
    }

    j -= 1;

    if(emergency_stop_)
    {
      ROS_WARN("EMERGENCY STOP HANDLED");   //ELIMINA
      emergency_stop_ = false;
      break;
    }

    if(informedOnlineReplanning_verbose_) ROS_INFO("------------------------------------------");
    if(informedOnlineReplanning_disp_) disp_->nextButton("Press \"next\" to execute the next InformedOnlineReplanning step");
  }

  for(unsigned int x=0; x<examined_nodes_.size();x++) examined_nodes_.at(x)->setAnalyzed(0);

  if(success)
  {
    replanned_path_ = replanned_path;
    std::reverse(replanned_path_vector.begin(),replanned_path_vector.end());  //ordered with growing cost
    replanned_paths_vector_ = replanned_path_vector;
    success_ = 1;

    toc = ros::WallTime::now();
    time_replanning_ = (toc - tic).toSec();

    if(informedOnlineReplanning_verbose_ || informedOnlineReplanning_disp_) ROS_INFO_STREAM("InformedOnlineReplanning has found a solution with cost: " <<replanned_path_->cost() << " in "<< time_replanning_ << "seconds. Number of sol: " << replanned_path_vector.size());
  }
  else
  {
    success_ = 0;
    if(informedOnlineReplanning_verbose_ || informedOnlineReplanning_disp_) ROS_INFO_STREAM("InformedOnlineReplanning has NOT found a solution");
  }

  toc = ros::WallTime::now();
  available_time_ = MAX_TIME-(toc-tic).toSec();

  return success;
}

}
