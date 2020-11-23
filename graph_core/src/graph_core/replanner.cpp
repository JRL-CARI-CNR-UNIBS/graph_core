#include "graph_core/replanner.h"

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
}

std::vector<PathPtr>  Replanner::addAdmissibleCurrentPath(const int idx_current_conn,
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

  //ROS_INFO_STREAM("other path size: "<<reset_other_paths.size());
  //ROS_INFO_STREAM("path size: "<<reset_other_paths.at(0)->getConnections().size());

  return reset_other_paths;
}


bool Replanner::pathSwitch(const PathPtr &current_path,
                           const NodePtr &node,
                           const bool &succ_node,
                           PathPtr &new_path,
                           PathPtr &subpath_from_path2,
                           int &connected2path_number)
{
  // Identifying the subpath of current_path starting from node
  NodePtr path1_node = node;
  PathPtr path1_node2goal = current_path->getSubpathFromNode(path1_node);
  double subpath1_cost = path1_node2goal->cost();

  double path_cost = subpath1_cost;
  double success = 0;

  for(unsigned int j = 0; j< admissible_other_paths_.size(); j++)
  {
    PathPtr path2 = admissible_other_paths_.at(j);

    //Finding the closest node
    std::vector<NodePtr> path2_node_vector;
    path2_node_vector.push_back(path2->findCloserNode(path1_node->getConfiguration()));

    if((path2_node_vector.back()->getConfiguration() - path2->getConnections().back()->getChild()->getConfiguration()).norm()<1e-06 && path2->getConnections().size()>1) //if the cloest node is the GOAL, the second closest node is considered
    {
      PathPtr path_support = path2->getSubpathToNode(path2->getConnections().back()->getParent());
      path2_node_vector.clear();
      path2_node_vector.push_back(path_support->findCloserNode(path1_node->getConfiguration()));
    }

    if(succ_node)
    {
      std::vector<ConnectionPtr> path2_conn = path2->getConnections();
      for(unsigned int t = 0; t < path2_conn.size(); t++) //the GOAL is not considered as a node to which directly connecting the path
      {
        if((path2_conn.at(t)->getParent()->getConfiguration()-path2_node_vector.back()->getConfiguration()).norm() > 1e-03 && (path2_conn.at(t)->getParent()->getConfiguration() != path2_node_vector.front()->getConfiguration())) //when some nodes are too close to each other, only one of them is considered
        {
          path2_node_vector.push_back(path2_conn[t]->getParent());
        }
      }
    }

    for(const NodePtr& path2_node : path2_node_vector)
    {
      PathPtr path2_node2goal = path2->getSubpathFromNode(path2_node);
      std::vector<ConnectionPtr> subpath2 = path2_node2goal->getConnections();
      double subpath2_cost = path2_node2goal->cost();

      double diff_subpath_cost = path_cost - subpath2_cost; //it is the maximum cost for connecting_path to be convenient
      double distance_path_node = (path1_node->getConfiguration()-path2_node->getConfiguration()).norm(); //the Euclidean distance is the minimum cost that the connecting_path can have

      if (diff_subpath_cost > 0 && distance_path_node < diff_subpath_cost) //if the Euclidean distance between the two nodes is bigger than the maximum cost for the connecting_path to be convenient, it isuseless to calculate a connecting_path because it surely will not be convenient
      {
        NodePtr path1_node_fake = std::make_shared<Node>(path1_node->getConfiguration());
        NodePtr path2_node_fake = std::make_shared<Node>(path2_node->getConfiguration());

        SamplerPtr sampler = std::make_shared<InformedSampler>(path1_node_fake->getConfiguration(), path2_node_fake->getConfiguration(), lb_, ub_,diff_subpath_cost); //the ellipsoide determined by diff_subpath_cost is used. Outside from this elipsoid, the nodes create a connecting_path not convenient
        //sampler->setCost(diff_subpath_cost);

        solver_->setSampler(sampler);

        solver_->resetProblem();
        bool s = solver_->addStart(path1_node_fake);
        solver_->addGoal(path2_node_fake);

        PathPtr connecting_path;

        if (solver_->solve(connecting_path, 1000))
        {
          PathLocalOptimizer path_solver(checker_, metrics_);

          path_solver.setPath(connecting_path);
          path_solver.solve(connecting_path);

          std::vector<ConnectionPtr> connecting_path_conn = connecting_path->getConnections();

          double conn_cost = subpath2_cost + connecting_path->cost();

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

              connecting_path_conn.front() = conn1;
              connecting_path_conn.back() = conn2;
            }

            else
            {
              double conn1_cost =  metrics_->cost(path1_node,path2_node);

              ConnectionPtr conn1 = std::make_shared<Connection>(path1_node,path2_node);
              conn1->setCost(conn1_cost);
              conn1->add();

              connecting_path_conn.clear();    //va tenuto perchè sopra ho dichiarato e l'ho riempito, in questo caso dovrà essere una singola connessione tra i due nodi
              connecting_path_conn.push_back(conn1);  //connecting_path_conn = conn1;
            }

            path1_node_fake->disconnect();
            path2_node_fake->disconnect();

            std::vector<ConnectionPtr> new_path_conn;

            new_path_conn.insert(new_path_conn.begin(),connecting_path_conn.begin(),connecting_path_conn.end());
            new_path_conn.insert(new_path_conn.end(),subpath2.begin(),subpath2.end());

            new_path = std::make_shared<Path>(new_path_conn, metrics_, checker_);
            path_cost = new_path->cost();

            success = 1;
            connected2path_number = j;
            subpath_from_path2 = path2_node2goal;
          }
        }
      }
    }
  }

  if(success)
  {
    //ROS_INFO_STREAM("PathSwitch has found a solution with cost: " << new_path->cost());
  }
  else
  {
    //ROS_INFO_STREAM("PathSwitch has NOT found a solution");
  }
  return success;
}
//Per il test
bool Replanner::pathSwitch(const PathPtr &current_path,
                           const NodePtr &node,
                           const bool &succ_node,
                           PathPtr &new_path,
                           PathPtr &subpath_from_path2,
                           int &connected2path_number, pathplan::TestUtil& ut)
{
  // Identifying the subpath of current_path starting from node
  NodePtr path1_node = node;
  PathPtr path1_node2goal = current_path->getSubpathFromNode(path1_node);
  double subpath1_cost = path1_node2goal->cost();

  double path_cost = subpath1_cost;
  double success = 0;

  for(unsigned int j = 0; j< admissible_other_paths_.size(); j++)
  {
    PathPtr path2 = admissible_other_paths_.at(j);

    //Finding the closest node
    std::vector<NodePtr> path2_node_vector;
    path2_node_vector.push_back(path2->findCloserNode(path1_node->getConfiguration()));

    if((path2_node_vector.back()->getConfiguration() - path2->getConnections().back()->getChild()->getConfiguration()).norm()<1e-06 && path2->getConnections().size()>1) //if the cloest node is the GOAL, the second closest node is considered
    {
      PathPtr path_support = path2->getSubpathToNode(path2->getConnections().back()->getParent());
      path2_node_vector.clear();
      path2_node_vector.push_back(path_support->findCloserNode(path1_node->getConfiguration()));
    }

    if(succ_node)
    {
      std::vector<ConnectionPtr> path2_conn = path2->getConnections();
      for(unsigned int t = 0; t < path2_conn.size(); t++) //the GOAL is not considered as a node to which directly connecting the path
      {
        if((path2_conn.at(t)->getParent()->getConfiguration()-path2_node_vector.back()->getConfiguration()).norm() > 1e-03 && (path2_conn.at(t)->getParent()->getConfiguration() != path2_node_vector.front()->getConfiguration())) //when some nodes are too close to each other, only one of them is considered
        {
          path2_node_vector.push_back(path2_conn[t]->getParent());
        }
      }
    }

    for(const NodePtr& path2_node : path2_node_vector)
    {
      int node_n;
      for(unsigned int r=0; r<path2->getWaypoints().size();r++)
      {
        if(path2_node->getConfiguration() == path2->getWaypoints().at(r))
        {
          node_n = r;
        }
      }
      PathPtr path2_node2goal = path2->getSubpathFromNode(path2_node);
      std::vector<ConnectionPtr> subpath2 = path2_node2goal->getConnections();
      double subpath2_cost = path2_node2goal->cost();

      double diff_subpath_cost = path_cost - subpath2_cost; //it is the maximum cost for connecting_path to be convenient
      double distance_path_node = (path1_node->getConfiguration()-path2_node->getConfiguration()).norm(); //the Euclidean distance is the minimum cost that the connecting_path can have

      ROS_INFO_STREAM("node n: " <<node_n<< " diff_subpath_cost: "<< diff_subpath_cost<<" distance: " << distance_path_node);

      if (diff_subpath_cost > 0 && distance_path_node < diff_subpath_cost) //if the Euclidean distance between the two nodes is bigger than the maximum cost for the connecting_path to be convenient, it isuseless to calculate a connecting_path because it surely will not be convenient
      {
        NodePtr path1_node_fake = std::make_shared<Node>(path1_node->getConfiguration());
        NodePtr path2_node_fake = std::make_shared<Node>(path2_node->getConfiguration());

        SamplerPtr sampler = std::make_shared<InformedSampler>(path1_node_fake->getConfiguration(), path2_node_fake->getConfiguration(), lb_, ub_,diff_subpath_cost); //the ellipsoide determined by diff_subpath_cost is used. Outside from this elipsoid, the nodes create a connecting_path not convenient
        //sampler->setCost(diff_subpath_cost);

        solver_->setSampler(sampler);

        bool s = solver_->addStart(path1_node_fake);
        solver_->addGoal(path2_node_fake);

        PathPtr connecting_path;

        if (solver_->solve(connecting_path, 1000))
        {
          PathLocalOptimizer path_solver(checker_, metrics_);

          path_solver.setPath(connecting_path);
          path_solver.solve(connecting_path);

          std::vector<ConnectionPtr> connecting_path_conn = connecting_path->getConnections();

          double conn_cost = subpath2_cost + connecting_path->cost();

          ROS_INFO_STREAM("conn_cost: "<<conn_cost<<" path cost: "<<path_cost);

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

              connecting_path_conn.front() = conn1;
              connecting_path_conn.back() = conn2;
            }

            else
            {
              double conn1_cost =  metrics_->cost(path1_node,path2_node);

              ConnectionPtr conn1 = std::make_shared<Connection>(path1_node,path2_node);
              conn1->setCost(conn1_cost);
              conn1->add();

              connecting_path_conn.clear();    //va tenuto perchè sopra ho dichiarato e l'ho riempito, in questo caso dovrà essere una singola connessione tra i due nodi
              connecting_path_conn.push_back(conn1);  //connecting_path_conn = conn1;
            }

            path1_node_fake->disconnect();
            path2_node_fake->disconnect();

            std::vector<ConnectionPtr> new_path_conn;

            new_path_conn.insert(new_path_conn.begin(),connecting_path_conn.begin(),connecting_path_conn.end());
            new_path_conn.insert(new_path_conn.end(),subpath2.begin(),subpath2.end());

            new_path = std::make_shared<Path>(new_path_conn, metrics_, checker_);
            path_cost = new_path->cost();

            success = 1;
            connected2path_number = j;
            subpath_from_path2 = path2_node2goal;

            //Visualization

            std::vector<double> t_vector(new_path->getWaypoints().size(),0.0);  //plotto solo il path, no time parametrization
            std::vector<int> marker_id; marker_id.push_back(-101);
            std::vector<double> marker_scale(3,0.01);
            //std::vector<double> marker_scale_sphere(3,0.03);
            std::vector<double> marker_color;
            marker_color = {1.0,1.0,0.0,1.0};

            std::vector<moveit::core::RobotState> wp_state_vector = ut.fromWaypoints2State(new_path->getWaypoints());
            //ut.displayTrajectoryOnMoveitRviz(new_path, t_vector, rviz_visual_tools::YELLOW);
            ut.displayPathNodesRviz(wp_state_vector, visualization_msgs::Marker::LINE_STRIP, marker_id, marker_scale, marker_color); //line strip
            ut.nextButton("Press \"next\" to execute the next PathSwitch step");
          }
        }
        else
        {
          ROS_INFO_STREAM("conn_path not solved");
        }
      }
    }
  }

  if(success)
  {
    ROS_INFO_STREAM("PathSwitch has found a solution with cost: " << new_path->cost());
    std::vector<double> t_vector(new_path->getWaypoints().size(),0.0);
    ut.displayTrajectoryOnMoveitRviz(new_path, t_vector, rviz_visual_tools::YELLOW);
  }
  else
  {
    ROS_INFO_STREAM("PathSwitch has NOT found a solution");
  }
  return success;
}

bool Replanner::informedOnlineReplanning(const int& informed, const bool& succ_node)
{
  ros::WallTime tic=ros::WallTime::now();
  ros::WallTime toc;
  double time_span;

  const double  MAX_TIME = 30.0; //seconds
  const double TIME_LIMIT = 25.0; //seconds
  const int CONT_LIMIT = 5;

  PathPtr new_path;
  PathPtr replanned_path;
  std::vector<PathPtr> replanned_path_vector;
  std::vector<PathPtr> reset_other_paths;
  std::vector<NodePtr> path1_node_vector;
  double replanned_path_cost = std::numeric_limits<double>::infinity();
  double previous_cost = current_path_->cost();
  bool success = 0;
  bool solved = 0;
  bool flag_other_paths = 0;
  bool first_sol = 1; //flag to calculate the first solution time
  unsigned int cont = 0;  //to count the number of replanning without significant improvement in the final solution
  std::vector<unsigned int> index;
  bool available_nodes;
  int limit;
  PathPtr subpath_from_path2;
  PathPtr confirmed_subpath_from_path2;
  int confirmed_connected2path_number;
  int connected2path_number;
  PathPtr admissible_current_path = NULL;
  NodePtr starting_node;

  examined_nodes_.clear();

  int idx; //to save the index of the connection on which the current configuration is
  ConnectionPtr current_conn = current_path_->findConnection(current_configuration_,idx);

  reset_other_paths = addAdmissibleCurrentPath(idx, admissible_current_path);
  admissible_other_paths_ = reset_other_paths;

  //NodePtr parent = current_conn->getParent();
  NodePtr child = current_conn->getChild();

  NodePtr actual_node = std::make_shared<Node>(current_configuration_);
  ConnectionPtr actual_node_conn = std::make_shared<Connection>(actual_node,child);
  double actual_node_conn_cost;
  if(current_conn->getCost() == std::numeric_limits<double>::infinity())
  {
    actual_node_conn_cost = std::numeric_limits<double>::infinity();        //DA ELIMINARE NELL'IMPLEMENTAZIONE REALE
  }
  else
  {
    actual_node_conn_cost = metrics_->cost(actual_node,child);
  }
  actual_node_conn->setCost(actual_node_conn_cost);
  actual_node_conn->add();

  PathPtr subpath1 = current_path_->getSubpathFromNode(child);
  std::vector<ConnectionPtr> subpath1_conn = subpath1->getConnections();

  std::vector<ConnectionPtr> conn;
  conn.push_back(actual_node_conn);
  conn.insert(conn.end(),subpath1_conn.begin(),subpath1_conn.end());

  replanned_path = std::make_shared<Path>(conn,metrics_,checker_); // at the start, the replanned path is initialized with the subpath of the current path from the current config to the goal
  replanned_path_cost = replanned_path->cost();

  if(current_conn->getCost() == std::numeric_limits<double>::infinity()) //if the obstacle is obstructing the current connection, the replanning must start from the current configuration
  {
    if(informed == 0)
    {
      bool rewire = 1;
      actual_node = current_path_->addNodeAtCurrentConfig(current_configuration_,current_conn,rewire); //with informed == 0 the node must be added to current_path_ because current_path is given to PathSwitch and not the replanned path
    }

    starting_node = actual_node;
    available_nodes = 0;
    limit = 0;
    path1_node_vector.push_back(actual_node);
  }
  else      //if the current connection is free, all the nodes between the current child to the parent of the connection obstructed are considered as starting points for the replanning
  {
    for(unsigned int i=0; i< subpath1_conn.size(); i++)   //escludo il nodo prima di goal
    {

      path1_node_vector.push_back(subpath1_conn.at(i)->getParent());
      if(subpath1_conn.at(i)->getCost() ==  std::numeric_limits<double>::infinity())
      {
        index.push_back(i);    //How to skip?
      }
    }

    if(index.empty())
    {
      limit = path1_node_vector.size()-2;  //you can consider the whole path1_node_vector without the last node before the goal, it is directly connected to the goal
    }
    else
    {
      limit = index.at(0); //to descard the subpath from the connection with infinite cost to the goal
    }

    starting_node = child;
    available_nodes = 1;
  }

  int n;
  int change_j = 0;
  int j = limit;

  while(j>=0)
  {
    if(informed>0)
    {
      if(flag_other_paths == 1) // if this flag is 1, a solution has been found, so the set of available path is updated: if the algorithm is considering a node on the subpath of path2, only the subpath after this node and not the whole path2 is considered
      {
        n = confirmed_subpath_from_path2->getConnections().size()-1;
        if(n == -1)
        {
          flag_other_paths = 0;
        }

        while(n>=0)
        {
          if(n==0)
          {
            flag_other_paths = 0;
          }

          if(path1_node_vector.at(j)->getConfiguration() == confirmed_subpath_from_path2->getConnections().at(n)->getParent()->getConfiguration()) // if the node analyzed is on the subpath2..(it happens only if informed == 2)
          {
            if(confirmed_connected2path_number<admissible_other_paths_.size()-1)
            {
              admissible_other_paths_.clear();

              admissible_other_paths_.insert(admissible_other_paths_.begin(),reset_other_paths.begin(),reset_other_paths.begin()+confirmed_connected2path_number);  //l'ultimo elemento è escluso
              admissible_other_paths_.push_back(confirmed_subpath_from_path2->getSubpathFromNode(confirmed_subpath_from_path2->getConnections().at(n)->getParent()));
              admissible_other_paths_.insert(admissible_other_paths_.end(),reset_other_paths.begin()+confirmed_connected2path_number+1,reset_other_paths.end());
            }
            else
            {
              admissible_other_paths_.clear();

              admissible_other_paths_.insert(admissible_other_paths_.begin(),reset_other_paths.begin(),reset_other_paths.begin()+confirmed_connected2path_number);
              admissible_other_paths_.push_back(confirmed_subpath_from_path2->getSubpathFromNode(confirmed_subpath_from_path2->getConnections().at(n)->getParent()));
            }

            n = -1;
          }
          else
          {
            admissible_other_paths_ = reset_other_paths;
            n = n-1;
          }
        }

      }

      solved = pathSwitch(replanned_path, path1_node_vector.at(j), succ_node, new_path, subpath_from_path2, connected2path_number);
    }
    else
    {
      solved = pathSwitch(current_path_, path1_node_vector.at(j), succ_node, new_path, subpath_from_path2, connected2path_number);
    }

    path1_node_vector.at(j)->setAnalyzed(1);  //to set as ANALYZED the node just analyzed. In this way, it will not be analyzed again in this replanning procedure
    examined_nodes_.push_back(path1_node_vector.at(j));  //to save the analyzed nodes

    if(solved)
    {
      PathPtr path;
      PathPtr subpath;
      std::vector<ConnectionPtr> path_conn;

      if(available_nodes) //calculating the cost of the replanned path found
      {
        if(j>0)
        {
          subpath =  subpath1->getSubpathToNode(path1_node_vector.at(j));  //path between the current connection child and the node analyzed now

          path_conn.push_back(actual_node_conn);  //connection between the current config e the child of the current conn

          std::vector<ConnectionPtr> conn_sup = subpath->getConnections();
          path_conn.insert(path_conn.end(),conn_sup.begin(),conn_sup.end());

          conn_sup = new_path->getConnections();
          path_conn.insert(path_conn.end(),conn_sup.begin(),conn_sup.end());

          path = std::make_shared<Path>(path_conn,metrics_,checker_);
        }
        else                                    //the node analyzed is the child of the current connection
        {
          path_conn.push_back(actual_node_conn);
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
        if(first_sol)
        {
          toc = ros::WallTime::now();
          time_first_sol_ = (toc - tic).toSec();
          first_sol = 0;  //0 when the time of the first solution found has been already saved
        }

        previous_cost = replanned_path_cost;
        replanned_path = path;
        replanned_path_cost = path->cost();
        confirmed_connected2path_number = connected2path_number;  //to remember the vector index of the path to which the algoithm has created a connection
        confirmed_subpath_from_path2 = subpath_from_path2;        //to save the subpath of the path to which the algoritm has created a connection
        success = 1;
        flag_other_paths = 1;  // a first solution has been found

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

        if(informed == 2)
        {
          std::vector<NodePtr> support;

          if(available_nodes == 1)
          {
            support.insert(support.begin(),path1_node_vector.begin(),path1_node_vector.begin()+j); //the nodes before the "j-th" surely have not yet been analyzed because they are analyzed in order starting from the one (available in the set) closest to the goal
          }

          for(unsigned int r=0; r<new_path->getConnections().size()-1; r++) //to exclude also the last node before the goal, surely a better path from it can't be found  ///////// //CHIEDI, in caso valuta anche dal current path se escludere il nodo prima di goal
          {
            if(new_path->getConnections().at(r)->getParent()->getAnalyzed() == 0 && new_path->getConnections().at(r)->getParent()->getNonOptimal() == 0) //Analyzed to check if they have been already analyzed (if 0 not not analyzed), nonOptimal to check if they are useful to improve the replanning solution (if 0, maybe they can improve the solution)
            {
              // the nodes of the new solution found are added to the set of the nodes to be analyzed
              support.push_back(new_path->getConnections().at(r)->getParent());
              change_j = change_j+1;
            }
          }

          path1_node_vector = support;

          if(available_nodes == 0 && starting_node == actual_node)
          {
            child = replanned_path->getConnections().at(0)->getChild();
            actual_node_conn = replanned_path->getConnections().at(0);
            available_nodes = 1;
          }

          subpath1 = replanned_path->getSubpathFromNode(child);
        }
      }

      toc = ros::WallTime::now();
      time_span = (toc-tic).toSec();

      if(time_span>MAX_TIME)
      {
        j = -1;
      }
      else
      {
        if(time_span>TIME_LIMIT && cont >= CONT_LIMIT)
        {
          j = -1;
        }
        else
        {
          if(previous_cost-replanned_path_cost< 0.05*previous_cost) //note that replanned_path_cost is always lower than previous_cost
          {
            cont = cont+1;
          }
          else
          {
            cont = 0;
          }
        }
      }
    }

    j = j-1;
    j = j+change_j;
    change_j = 0;
  }

  for(unsigned int x=0; x<examined_nodes_.size();x++)
  {
    examined_nodes_.at(x)->setAnalyzed(0);
  }

  if(success)
  {
    replanned_path_ = replanned_path;
    std::reverse(replanned_path_vector.begin(),replanned_path_vector.end());  //ordered with growing cost
    replanned_paths_vector_ = replanned_path_vector;

    ROS_INFO_STREAM("InformedOnlineReplanning has found a solution with cost: " <<replanned_path_->cost());
  }
  else
  {
    ROS_INFO_STREAM("InformedOnlineReplanning has NOT found a solution");
  }

  toc = ros::WallTime::now();
  time_replanning_ = (toc - tic).toSec();

  return success;
}


bool Replanner::informedOnlineReplanning(const int& informed, const bool& succ_node, pathplan::TestUtil& ut)
{
  ROS_INFO_STREAM("costo: "<<current_path_->cost());
  ros::WallTime tic=ros::WallTime::now();
  ros::WallTime toc;
  double time_span;

  const double  MAX_TIME = 3000000000.0; //seconds
  const double TIME_LIMIT = 2500000000.0; //seconds
  const int CONT_LIMIT = 5;

  std::vector<double> marker_scale_sphere(3,0.03);
  std::vector<double> marker_color_sphere = {0.0,0.0,0.0,1.0};
  std::vector<int> marker_id_sphere = {15678};

  PathPtr new_path;
  PathPtr replanned_path;
  std::vector<PathPtr> replanned_path_vector;
  std::vector<PathPtr> reset_other_paths;
  std::vector<NodePtr> path1_node_vector;
  double replanned_path_cost = std::numeric_limits<double>::infinity();
  double previous_cost = current_path_->cost();
  bool success = 0;
  bool solved = 0;
  bool flag_other_paths = 0;
  bool first_sol = 1; //flag to calculate the first solution time
  unsigned int cont = 0;  //to count the number of replanning without significant improvement in the final solution
  std::vector<unsigned int> index;
  bool available_nodes;
  int limit;
  PathPtr subpath_from_path2;
  PathPtr confirmed_subpath_from_path2;
  int confirmed_connected2path_number;
  int connected2path_number;
  PathPtr admissible_current_path = NULL;
  NodePtr starting_node;

  examined_nodes_.clear();

  int idx; //to save the index of the connection on which the current configuration is
  ConnectionPtr current_conn = current_path_->findConnection(current_configuration_,idx);

  reset_other_paths = addAdmissibleCurrentPath(idx, admissible_current_path);
  admissible_other_paths_ = reset_other_paths;

  //NodePtr parent = current_conn->getParent();
  NodePtr child = current_conn->getChild();

  NodePtr actual_node = std::make_shared<Node>(current_configuration_);
  ConnectionPtr actual_node_conn = std::make_shared<Connection>(actual_node,child);
  double actual_node_conn_cost;
  if(current_conn->getCost() == std::numeric_limits<double>::infinity())
  {
    actual_node_conn_cost = std::numeric_limits<double>::infinity();        //DA ELIMINARE NELL'IMPLEMENTAZIONE REALE
  }
  else
  {
    actual_node_conn_cost = metrics_->cost(actual_node,child);
  }
  actual_node_conn->setCost(actual_node_conn_cost);
  actual_node_conn->add();

  PathPtr subpath1 = current_path_->getSubpathFromNode(child);
  std::vector<ConnectionPtr> subpath1_conn = subpath1->getConnections();

  std::vector<ConnectionPtr> conn;
  conn.push_back(actual_node_conn);
  conn.insert(conn.end(),subpath1_conn.begin(),subpath1_conn.end());

  replanned_path = std::make_shared<Path>(conn,metrics_,checker_); // at the start, the replanned path is initialized with the subpath of the current path from the current config to the goal
  replanned_path_cost = replanned_path->cost();

  if(current_conn->getCost() == std::numeric_limits<double>::infinity()) //if the obstacle is obstructing the current connection, the replanning must start from the current configuration
  {
    if(informed == 0)
    {
      bool rewire = 1;
      actual_node = current_path_->addNodeAtCurrentConfig(current_configuration_,current_conn,rewire); //with informed == 0 the node must be added to current_path_ because current_path is given to PathSwitch and not the replanned path
    }

    starting_node = actual_node;
    available_nodes = 0;
    limit = 0;
    path1_node_vector.push_back(actual_node);
  }
  else      //if the current connection is free, all the nodes between the current child to the parent of the connection obstructed are considered as starting points for the replanning
  {
    for(unsigned int i=0; i< subpath1_conn.size(); i++)
    {
      path1_node_vector.push_back(subpath1_conn.at(i)->getParent());
      if(subpath1_conn.at(i)->getCost() ==  std::numeric_limits<double>::infinity())
      {
        index.push_back(i);    //How to skip?
      }
    }

    if(index.empty())
    {
      limit = path1_node_vector.size()-2;  //you can consider the whole path1_node_vector
    }
    else
    {
      limit = index.at(0); //to descard the subpath from the connection with infinite cost to the goal
    }

    starting_node = child;
    available_nodes = 1;
  }

  int n;
  int change_j = 0;
  int j = limit;

  while(j>=0)
  {
    ROS_INFO_STREAM("j: "<<j);
    ROS_INFO_STREAM("path1_node_vector size:"<<path1_node_vector.size());
    ROS_INFO_STREAM("admissible_other_path size:"<<admissible_other_paths_.size());

    if(informed>0)
    {
      if(flag_other_paths == 1) // if this flag is 1, a solution has been found, so the set of available path is updated: if the algorithm is considering a node on the subpath of path2, only the subpath after this node and not the whole path2 is considered
      {
        n = confirmed_subpath_from_path2->getConnections().size()-1;
        if(n == -1)
        {
          flag_other_paths = 0;
        }

        while(n>=0)
        {
          if(n==0)
          {
            flag_other_paths = 0;
          }

          if(path1_node_vector.at(j)->getConfiguration() == confirmed_subpath_from_path2->getConnections().at(n)->getParent()->getConfiguration()) // if the node analyzed is on the subpath2..(it happens only if informed == 2)
          {
            if(confirmed_connected2path_number<admissible_other_paths_.size()-1)
            {
              admissible_other_paths_.clear();

              admissible_other_paths_.insert(admissible_other_paths_.begin(),reset_other_paths.begin(),reset_other_paths.begin()+confirmed_connected2path_number);  //l'ultimo elemento è escluso
              admissible_other_paths_.push_back(confirmed_subpath_from_path2->getSubpathFromNode(confirmed_subpath_from_path2->getConnections().at(n)->getParent()));
              admissible_other_paths_.insert(admissible_other_paths_.end(),reset_other_paths.begin()+confirmed_connected2path_number+1,reset_other_paths.end());

              ROS_INFO_STREAM("dimensione admissible: "<<admissible_other_paths_.size());
            }
            else
            {
              admissible_other_paths_.clear();

              admissible_other_paths_.insert(admissible_other_paths_.begin(),reset_other_paths.begin(),reset_other_paths.begin()+confirmed_connected2path_number);
              admissible_other_paths_.push_back(confirmed_subpath_from_path2->getSubpathFromNode(confirmed_subpath_from_path2->getConnections().at(n)->getParent()));

              ROS_INFO_STREAM("dimensione admissible: "<<admissible_other_paths_.size());
            }

            n = -1;
          }
          else
          {
            admissible_other_paths_ = reset_other_paths;
            n = n-1;
          }
        }

      }

      solved = pathSwitch(replanned_path, path1_node_vector.at(j), succ_node, new_path, subpath_from_path2, connected2path_number);
    }
    else
    {
      solved = pathSwitch(current_path_, path1_node_vector.at(j), succ_node, new_path, subpath_from_path2, connected2path_number);
    }

    path1_node_vector.at(j)->setAnalyzed(1);  //to set as ANALYZED the node just analyzed. In this way, it will not be analyzed again in this replanning procedure
    examined_nodes_.push_back(path1_node_vector.at(j));  //to save the analyzed nodes

    /*////////////////////////Visualization of analyzed nodes //////////////////////////////////////*/
    moveit::core::RobotState black_marker = ut.fromWaypoints2State(examined_nodes_.back()->getConfiguration());
    marker_id_sphere.back() = marker_id_sphere.back()+1;

    std::vector<moveit::core::RobotState> black_marker_v = {black_marker};
    ut.displayPathNodesRviz(black_marker_v, visualization_msgs::Marker::SPHERE, marker_id_sphere, marker_scale_sphere, marker_color_sphere);
    /*/////////////////////////////////////////////////////////////////////////////////////////////*/

    if(solved)
    {
      PathPtr path;
      PathPtr subpath;
      std::vector<ConnectionPtr> path_conn;

      ROS_INFO_STREAM("available nodes: "<<available_nodes);

      if(available_nodes) //calculating the cost of the replanned path found
      {
        if(j>0)
        {
          subpath =  subpath1->getSubpathToNode(path1_node_vector.at(j));  //path between the current connection child and the node analyzed now

          path_conn.push_back(actual_node_conn);  //connection between the current config e the child of the current conn

          std::vector<ConnectionPtr> conn_sup = subpath->getConnections();
          path_conn.insert(path_conn.end(),conn_sup.begin(),conn_sup.end());

          conn_sup = new_path->getConnections();
          path_conn.insert(path_conn.end(),conn_sup.begin(),conn_sup.end());

          path = std::make_shared<Path>(path_conn,metrics_,checker_);
        }
        else                                    //the node analyzed is the child of the current connection
        {
          path_conn.push_back(actual_node_conn);
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
        ROS_INFO_STREAM("new path found, cost: " << path->cost() <<" previous cost: " << replanned_path_cost);

        if(first_sol)
        {
          toc = ros::WallTime::now();
          time_first_sol_ = (toc - tic).toSec();
          first_sol = 0;  //0 when the time of the first solution found has been already saved
        }

        previous_cost = replanned_path_cost;
        replanned_path = path;
        replanned_path_cost = path->cost();
        confirmed_connected2path_number = connected2path_number;  //to remember the vector index of the path to which the algoithm has created a connection
        confirmed_subpath_from_path2 = subpath_from_path2;        //to save the subpath of the path to which the algoritm has created a connection
        success = 1;
        flag_other_paths = 1;  // a first solution has been found

        if(replanned_path_vector.size()<10) //the algorithm gives as output the vector of the best 10 solutions found
        {
          replanned_path_vector.push_back(replanned_path);
        }
        else
        {
          ROS_INFO_STREAM("else riempimento vettore replanned");
          std::vector<PathPtr> support_vector;
          support_vector.insert(support_vector.begin(),replanned_path_vector.begin()+1,replanned_path_vector.end());
          support_vector.push_back(replanned_path);

          replanned_path_vector = support_vector;
        }

        if(informed == 2)
        {
          std::vector<NodePtr> support;

          if(available_nodes == 1)
          {
            support.insert(support.begin(),path1_node_vector.begin(),path1_node_vector.begin()+j); //the nodes before the "j-th" surely have not yet been analyzed because they are analyzed in order starting from the one (available in the set) closest to the goal
          }

          for(unsigned int r=0; r<new_path->getConnections().size()-1; r++)
          {
            if(new_path->getConnections().at(r)->getParent()->getAnalyzed() == 0 && new_path->getConnections().at(r)->getParent()->getNonOptimal() == 0) //Analyzed to check if they have been already analyzed (if 0 not not analyzed), nonOptimal to check if they are useful to improve the replanning solution (if 0, maybe they can improve the solution)
            {
              // the nodes of the new solution found are added to the set of the nodes to be analyzed
              support.push_back(new_path->getConnections().at(r)->getParent());
              change_j = change_j+1;
              ROS_INFO_STREAM("new node added, change_j: = "<<change_j);
            }
          }

          path1_node_vector = support;

          if(available_nodes == 0 && starting_node == actual_node)
          {
            child = replanned_path->getConnections().at(0)->getChild();
            actual_node_conn = replanned_path->getConnections().at(0);
            available_nodes = 1;
          }

          subpath1 = replanned_path->getSubpathFromNode(child);
        }

        /*//////////////////////////Visualization////////////////////////////////////*/

        std::vector<double> t_vector(replanned_path->getWaypoints().size(),0.0);  //plotto solo il path, no time parametrization
        std::vector<int> marker_id; marker_id.push_back(-101);
        std::vector<double> marker_scale(3,0.01);
        //std::vector<double> marker_scale_sphere(3,0.03);
        std::vector<double> marker_color;
        marker_color = {1.0,1.0,0.0,1.0};

        std::vector<moveit::core::RobotState> wp_state_vector = ut.fromWaypoints2State(replanned_path->getWaypoints());
        //ut.displayTrajectoryOnMoveitRviz(replanned_path, t_vector, rviz_visual_tools::YELLOW);
        ut.displayPathNodesRviz(wp_state_vector, visualization_msgs::Marker::LINE_STRIP, marker_id, marker_scale, marker_color); //line strip
        /*/////////////////////////////////////////////////////////////////////////*/

      }
      else
      {
        ROS_INFO_STREAM("NO better path found, cost: " << path->cost() <<" previous cost: " << replanned_path_cost);
      }

      toc = ros::WallTime::now();
      time_span = (toc-tic).toSec();

      if(time_span>MAX_TIME)
      {
        j = -1;
      }
      else
      {
        if(time_span>TIME_LIMIT && cont >= CONT_LIMIT)
        {
          j = -1;
        }
        else
        {
          if(previous_cost-replanned_path_cost< 0.05*previous_cost) //note that replanned_path_cost is always lower than previous_cost
          {
            cont = cont+1;
          }
          else
          {
            cont = 0;
          }
        }
      }
    }

    j = j-1;
    j = j+change_j;
    change_j = 0;

    ut.nextButton("Press \"next\" to execute the next InformedOnlineReplanning step");

  }

  for(unsigned int x=0; x<examined_nodes_.size();x++)
  {
    examined_nodes_.at(x)->setAnalyzed(0);
  }

  if(success)
  {
    replanned_path_ = replanned_path;
    std::reverse(replanned_path_vector.begin(),replanned_path_vector.end());  //ordered with growing cost
    replanned_paths_vector_ = replanned_path_vector;

    ROS_INFO_STREAM("InformedOnlineReplanning has found a solution with cost: " <<replanned_path_->cost());
  }
  else
  {
    ROS_INFO_STREAM("InformedOnlineReplanning has NOT found a solution");
  }

  toc = ros::WallTime::now();
  time_replanning_ = (toc - tic).toSec();

  return success;
}

}
