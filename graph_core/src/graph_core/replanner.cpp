#include "graph_core/replanner.h"

namespace pathplan{
    Replanner::Replanner()
    {
        //ctor
    }

    replanner::~Replanner()
    {
        //dtor
    }

    bool pathSwitch(const NodePtr& node, const bool& succ_node, PathPtr& new_path, PathPtr& subpath_from_path2, int& connected2path_number)
    {
        // Identifying the subpath of current_path starting from node
        NodePtr path1_node = node;
        PathPtr path1_node2goal = current_path_->getSubpathFromNode(path1_node);
        double subpath1_cost = path1_node2goal->cost();

        double path_cost = subpath_cost;

        for(unsigned int j = 0; j< admissible_other_paths_.size(); j++)//for(const PathPtr& path2 : admissible_other_paths_)
        {
            PathPtr& path2 = admissible_other_paths_[j];

            //Finding the closest node
            std::vector<NodePtr> path2_node_vector;
            path2_node_vector.push_back(path2->findCloserNode(path1_node->getConfiguration()));

            if(path2_node_vector.back() == (path2->getConnections()).back()->getChild()) //if the cloest node is the GOAL, the second closest node is considered
            {
                PathPtr path_support = path2->getSubpathToNode((path2->getConnections()).back()->getParent());
                path2_node_vector.back() = path_support->findCloserNode(path1_node->getConfiguration());
            }

            if(succ_node == 1)
            {
                std::vector<ConnectionPtr> path2_conn = path2->getConnections();
                path2_node_vector.push_back(path2_conn.front()->getParent());

                for(unsigned int t = 1; t < path2_conn.size(); t++) //the GOAL is not considered as a node to which directly connecting the path
                {
                    if((path2_conn[t]->getParent()->getConfiguration()-path2_node_vector.back()->getConfiguration()).norm() > 0.001 && (path2_conn[t]->getParent() != path2_node_vector.front())) //when some nodes are too close to each other, only one of them is considered
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
                    NodePtr path1_node_fake = std::make_shared<pathplan::Node>(path1_node->getConfiguration());
                    NodePtr path2_node_fake = std::make_shared<pathplan::Node>(path2_node->getConfiguration());

                    pathplan::SamplerPtr sampler = std::make_shared<pathplan::InformedSampler>(path1_node_fake->getConfiguration(), path2_node_fake->getConfiguration(), lb, ub, diff_subpath_cost); //the ellipsoide determined by diff_subpath_cost is used. Outside from this elipsoid, the nodes create a connecting_path not convenient

                    pathplan::BiRRT solver(metrics, checker, sampler);
                    //solver.config(nh);  //CHIEDI e chiedi max distance se va usata e opt_type nell'ottimizzazione
                    solver.addStart(path1_node_fake);
                    solver.addGoal(path2_node_fake);

                    PathPtr connecting_path;

                    if (solver.solve(connecting_path, 1000))
                    {
                        PathLocalOptimizer path_solver(checker, metrics); //chiedi
                        //path_solver.config(nh);

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

                                double conn1_cost = metrics->cost(path1_node,node1);
                                double conn2_cost = metrics->cost(node2,path2_node);

                                ConnectionPtr conn1 = std::make_shared<Connection>(path1_node,node1);
                                conn1->setCost(conn1_cost);
                                ConnectionPtr conn2 = std::make_shared<Connection>(node2,path2_node);
                                conn2->setCost(conn2_cost);

                                connecting_path_conn.front() = conn1;
                                connecting_path_conn.back() = conn2;

                            }
                            else
                            {
                               double conn1_cost =  metrics->cost(path1_node,path2_node);

                               ConnectionPtr conn1 = std::make_shared<Connection>(path1_node,path2_node);
                               conn1->setCost(conn1_cost);

                               connecting_path_conn.clear();
                               connecting_path_conn.push_back(conn1);  //connecting_path_conn = conn1;

                            }

                            path1_node_fake.disconnect();
                            path2_node_fake.disconnect();

                            std::vector<ConnectionPtr> new_path_conn;

                            for(unsigned int z = 1; z < connecting_path_conn.size(); z++)  // /////
                            {
                                new_path_conn.push_back(connecting_path_conn[z]);
                            }

                            for(unsigned int z = 1; z < subpath2.size(); z++)  // /////
                            {
                                new_path_conn.push_back(subpath2[z]);
                            }

                            PathPtr new_path = std::make_shared<Path>(new_path_conn, metrics, checker);
                            path_cost = new_path->cost();

                            double success = 1;
                            connected2path_number = j;
                            subpath_from_path2 = path2_node2goal;

                        }
                    }

                }
            }

        }

    }

}
