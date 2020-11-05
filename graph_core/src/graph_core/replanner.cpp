#include "graph_core/replanner.h"

namespace pathplan
{
    Replanner::Replanner(Eigen::VectorXd& current_configuration,
                         PathPtr& current_path,
                         std::vector<PathPtr>& other_paths,
                         const TreeSolverPtr& solver,
                         const MetricsPtr& metrics,
                         const CollisionCheckerPtr& checker,
                         const Eigen::VectorXd& lb,
                         const Eigen::VectorXd& ub)
    {
        current_configuration_ = current_configuration;
        current_path_ = current_path;
        other_paths_ = other_paths;
        solver_ = solver;
        metrics_ = metrics;
        checker_ = checker;
        lb_ = lb;
        ub_ = ub;
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

            if(path2_node_vector.back()->getConfiguration() == path2->getConnections().back()->getChild()->getConfiguration()) //if the cloest node is the GOAL, the second closest node is considered
            {
                PathPtr path_support = path2->getSubpathToNode(path2->getConnections().back()->getParent());
                path2_node_vector.clear();
                path2_node_vector.push_back(path_support->findCloserNode(path1_node->getConfiguration()));
            }

            if(succ_node == 1)
            {
                std::vector<ConnectionPtr> path2_conn = path2->getConnections();
                for(unsigned int t = 0; t < path2_conn.size(); t++) //the GOAL is not considered as a node to which directly connecting the path
                {
                    if((path2_conn.at(t)->getParent()->getConfiguration()-path2_node_vector.back()->getConfiguration()).norm() > 0.001 && (path2_conn.at(t)->getParent()->getConfiguration() != path2_node_vector.front()->getConfiguration())) //when some nodes are too close to each other, only one of them is considered
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

                if (distance_path_node < diff_subpath_cost) //if the Euclidean distance between the two nodes is bigger than the maximum cost for the connecting_path to be convenient, it isuseless to calculate a connecting_path because it surely will not be convenient
                {
                    NodePtr path1_node_fake = std::make_shared<Node>(path1_node->getConfiguration());
                    NodePtr path2_node_fake = std::make_shared<Node>(path2_node->getConfiguration());

                    SamplerPtr sampler = std::make_shared<InformedSampler>(path1_node_fake->getConfiguration(), path2_node_fake->getConfiguration(), lb_, ub_); //the ellipsoide determined by diff_subpath_cost is used. Outside from this elipsoid, the nodes create a connecting_path not convenient
                    sampler->setCost(diff_subpath_cost);

                    solver_->setSampler(sampler);
                    solver_->addStart(path1_node_fake);
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

                            new_path = std::make_shared<Path>(new_path_conn, metrics_, checker_);  //chiedi
                            path_cost = new_path->cost();

                            success = 1;
                            connected2path_number = j;
                            subpath_from_path2 = path2_node2goal;
                        }
                    }
                }
            }
        }

        return success;
    }

    bool Replanner::informedOnlineReplanning(const int& informed, const bool& succ_node)
    {
        clock_t tic;
        clock_t toc;
        tic = clock();

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
        unsigned int cont = 0;
        std::vector<unsigned int> index;
        bool available_nodes;
        int limit;
        PathPtr subpath_from_path2;
        PathPtr confirmed_subpath_from_path2;
        int confirmed_connected2path_number;
        int connected2path_number;
        PathPtr subpath1;
        std::vector<ConnectionPtr> subpath1_conn;
        std::vector<NodePtr> support;

        int idx;
        NodePtr actual_node = current_path_->actualNode(current_configuration_,idx);

        if(current_path_->cost() == std::numeric_limits<double>::infinity())  //if the path is obstructed by an obstacle, the connection obstructed cost is infinte
        {
            int z = current_path_->getConnections().size()-1;
            PathPtr admissible_current_path = NULL;

            while(z>idx) //to find the savable part of current_path, the subpath after the connection obstruced by the obstacle
            {
                if(current_path_->getConnections().at(z)->getCost() == std::numeric_limits<double>::infinity())
                {
                    if(z == current_path_->getConnections().size()-1)
                    {
                        admissible_current_path = NULL; //there are no connections between the obstacle and the goal
                    }
                    else
                    {
                        admissible_current_path = current_path_->getSubpathFromNode(current_path_->getConnections().at(z)->getChild());
                    }

                    z = -1;
                }
                else
                {
                    z = z-1;
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

        NodePtr node;
        NodePtr parent = current_path_->getConnections().at(idx)->getParent();
        NodePtr child = current_path_->getConnections().at(idx)->getChild();
        double actual_node_conn_cost;
        ConnectionPtr actual_node_conn;

        if(current_path_->getConnections().at(idx)->getCost() == std::numeric_limits<double>::infinity() || idx == current_path_->getConnections().size()-1) //if the obstacle is obstructing the current connection or the current connection is the last one, the replanning must start from the current configuration, so a node corresponding to the config is added
        {
            node = actual_node;

            double cost_parent = metrics_->cost(parent->getConfiguration(), node->getConfiguration());
            ConnectionPtr conn_parent = std::make_shared<Connection>(parent, node);
            conn_parent->setCost(cost_parent);
            conn_parent->add();  //devo disconnettere la connessione precedente?

            double cost_child = std::numeric_limits<double>::infinity();
            ConnectionPtr conn_child = std::make_shared<Connection>(node,child);
            conn_child->setCost(cost_child);
            conn_child->add();

            PathPtr subpath_parent = current_path_->getSubpathToNode(parent);
            PathPtr subpath_child = current_path_->getSubpathFromNode(child);

            std::vector<ConnectionPtr> conn;
            conn.insert(conn.begin(),subpath_parent->getConnections().begin(),subpath_parent->getConnections().end());
            conn.push_back(conn_parent);
            conn.push_back(conn_child);
            conn.insert(conn.end(),subpath_child->getConnections().begin(),subpath_child->getConnections().end());

            current_path_->setConnections(conn); // chiedi metodo setConnections
            current_path_->computeCost();

            replanned_path = current_path_->getSubpathFromNode(node); //at the start, the replanned path is initialized with the subpath of the current path
            replanned_path_cost =  replanned_path->cost();

            available_nodes = 0;
            limit = 0;
            path1_node_vector.push_back(node);
        }
        else            //if the current connection is free, all the nodes between the current child to the parent of the connection obstructed are considered as starting points for the replanning
        {
            node = child;
            actual_node_conn_cost = metrics_->cost(actual_node,node);
            actual_node_conn = std::make_shared<Connection>(actual_node,node);
            actual_node_conn->setCost(actual_node_conn_cost);
            actual_node_conn->add();

            subpath1 = current_path_->getSubpathFromNode(node);
            subpath1_conn = subpath1->getConnections();

            std::vector<ConnectionPtr> conn;
            conn.push_back(actual_node_conn);
            conn.insert(conn.end(),subpath1_conn.begin(),subpath1_conn.end());

            replanned_path = current_path_->getSubpathFromNode(node); // at the start, the replanned path is initialized with the subpath of the current path from the current config to the goal
            replanned_path->setConnections(conn);  //chiedi
            replanned_path->computeCost();
            replanned_path_cost = replanned_path->cost();

            for(unsigned int i=0; i< subpath1_conn.size(); i++)
            {
                path1_node_vector.push_back(subpath1_conn.at(i)->getParent());
                if(subpath1_conn.at(i)->getCost() ==  std::numeric_limits<double>::infinity())
                {
                    index.push_back(i);
                }
            }

            if(index.empty())
            {
                limit = path1_node_vector.size()-1;
            }
            else
            {
                limit = index.front(); //to descard the subpath from the connection with infinite cost to the goal
            }

            available_nodes = 1;
        }

        int change_j = 0;
        int j = limit;
        int n;

        while(j>=0)
        {
            if(informed>0)
            {
                if(flag_other_paths == 1) //if almost one replanned path has been found, the number of nodes of the subpath of the path2 to which the replaned path is connected are calculated
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

                      //admissible_other_paths_.clear();

                      if(path1_node_vector.at(j)->getConfiguration() == confirmed_subpath_from_path2->getConnections().at(n)->getParent()->getConfiguration()) // if the node analyzed is on the subpath2..(it happens only if informed == 2)
                      {
                          if(confirmed_connected2path_number<other_paths_.size()-1)
                          {
                              admissible_other_paths_.insert(admissible_other_paths_.begin(),reset_other_paths.begin(),reset_other_paths.begin()+confirmed_connected2path_number-1);
                              admissible_other_paths_.push_back(confirmed_subpath_from_path2->getSubpathFromNode(confirmed_subpath_from_path2->getConnections().at(n)->getParent()));
                              admissible_other_paths_.insert(admissible_other_paths_.end(),reset_other_paths.begin()+confirmed_connected2path_number+1,reset_other_paths.end());
                          }
                          else
                          {
                              admissible_other_paths_.insert(admissible_other_paths_.begin(),reset_other_paths.begin(),reset_other_paths.begin()+confirmed_connected2path_number-1);
                              admissible_other_paths_.push_back(confirmed_subpath_from_path2->getSubpathFromNode(confirmed_subpath_from_path2->getConnections().at(n)->getParent()));
                          }

                          n = -1;
                      }
                      else
                      {
                          admissible_other_paths_ = reset_other_paths;  //if informed == 1 there are "n" while iterations unnecessary..you should correct it
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
            examined_nodes_.push_back(path1_node_vector.at(j));

            if(solved == 1)
            {
                PathPtr path;
                PathPtr subpath;
                std::vector<ConnectionPtr> path_conn;

                if(available_nodes == 1) //calculating the cost of the replanned path found
                {
                    if(j>0)
                    {
                       subpath =  subpath1->getSubpathToNode(path1_node_vector.at(j));
                       path_conn.push_back(actual_node_conn);
                       path_conn.insert(path_conn.end(),subpath->getConnections().begin(),subpath->getConnections().end());
                       path_conn.insert(path_conn.end(),new_path->getConnections().begin(),new_path->getConnections().end());

                       path = std::make_shared<Path>(path_conn,metrics_,checker_);
                    }
                    else
                    {
                        path_conn.push_back(actual_node_conn);
                        path_conn.insert(path_conn.end(),new_path->getConnections().begin(),new_path->getConnections().end());

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
                        time_first_sol_ = (clock()-tic)/CLOCKS_PER_SEC;
                        first_sol = 0;  //0 when the time of the first solution found has been already saved
                    }

                    previous_cost = replanned_path_cost;
                    replanned_path = path;
                    replanned_path_cost = path->cost();
                    confirmed_connected2path_number = connected2path_number;
                    confirmed_subpath_from_path2 = subpath_from_path2;
                    success = 1;
                    flag_other_paths = 1;

                    if(replanned_path_vector.size()<10) //the algorithm gives as output the vector of the best 10 solutions found, oredered by their cost
                    {
                        replanned_path_vector.push_back(replanned_path);
                    }
                    else
                    {
                        replanned_path_vector.insert(replanned_path_vector.begin(),replanned_path_vector.begin()+1,replanned_path_vector.end());
                        replanned_path_vector.back() = replanned_path;   //controlla
                    }

                    if(informed == 2 && available_nodes == 1)
                    {
                        support.clear();
                        support.insert(support.begin(),path1_node_vector.begin(),path1_node_vector.begin()+j); //the nodes before the "j+1 th" surely have not yet been analyzed

                        for(unsigned int r=0; r<new_path->getConnections().size(); r++)
                        {
                            if(new_path->getConnections().at(r)->getParent()->getAnalyzed() == 0 && new_path->getConnections().at(r)->getParent()->getNonOptimal() == 0) //Analyzed to check if they have been already analyzed (if 0 not not analyzed), NonOptimal to check if they are useful to improve the replanning solution (if 0, maybe they can improve the solution)
                            {
                                // the nodes of the new solution found are added to the set of the nodes to be analyzed
                                support.push_back(new_path->getConnections().at(r)->getParent());
                                change_j = change_j+1;
                            }
                        }

                        path1_node_vector = support;
                        subpath1 = replanned_path->getSubpathFromNode(node);
                    }

                }
                toc = (clock()-tic)/CLOCKS_PER_SEC;
                if(toc>30)
                {
                    j = -1;
                }
                else
                {
                    if(toc>25 && cont >= 5)
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

                j = j-1;
                j = j+change_j;
                change_j = 0;
            }


        }

        for(unsigned int x=0; x<examined_nodes_.size();x++)
        {
            examined_nodes_.at(x)->setAnalyzed(0);
        }

        if(success)
        {
            replanned_path_ = replanned_path;
            std::reverse(replanned_path_vector.begin(),replanned_path_vector.end());
            replanned_paths_vector_ = replanned_path_vector;
        }

        time_replanning_ = (clock()-tic)/CLOCKS_PER_SEC;
        return success;

    }

}
