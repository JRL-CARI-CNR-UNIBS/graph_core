#include <graph_core/problem.h>

namespace ha_planner
{

Problem::Problem(const GraphPtr& graph)
{
  m_graph=graph;
}

void Problem::generateNodesFromStartAndEndPoints(const std::vector<std::vector<double> > &stop_points,
                                                 const std::vector<std::vector<double> > &end_points)
{
  for (const std::vector<double>& start_point: stop_points)
  {
    NodePtr node=m_graph->addNode(start_point);
    if (m_graph->checkNodeCollision(node))
    {
      m_start_nodes.push_back(node);
      m_start_trees.push_back(std::make_shared<Tree>(node,Forward));
    }
    else
    {
      m_graph->removeNodeWithConnections(node);
    }
  }

  for (const std::vector<double>& end_point: end_points)
  {
    NodePtr node=m_graph->addNode(end_point);
    if (m_graph->checkNodeCollision(node))
    {
      m_end_nodes.push_back(node);
      m_goal_trees.push_back(std::make_shared<Tree>(node,Backward));
      for (const NodePtr& start_node: m_start_nodes)
      {
        m_combinations.push_back(Combination(start_node,node));
      }
    }
    else
    {
      m_graph->removeNodeWithConnections(node);
    }
  }

  m_best_path=std::make_shared<Path>();
  for (const Combination& comb: m_combinations)
  {
    PathPtr path=std::make_shared<Path>();
    double utopia=distance(comb.first->getScaledJoints(),comb.second->getScaledJoints());
    if (utopia<m_tolerance)
    {
      ConnectionVct sol;
      sol.push_back(m_graph->addConnection(comb.first,comb.second));
      path=std::make_shared<Path>(sol);
      m_best_path=path;
    }
    m_best_path_per_combination.insert(  std::pair<Combination,PathPtr>(comb,path));
    m_utopia_cost_per_combination.insert(std::pair<Combination,double>(comb,utopia));
    m_rot_matrix.insert(std::pair<Combination,Eigen::MatrixXd>(comb,computeRotationMatrix(comb.first->getScaledJoints(),
                                                                                          comb.second->getScaledJoints())));
  }

}

bool Problem::solved()
{
  return (m_best_path->getCost()<std::numeric_limits<double>::infinity());
}

bool Problem::storeIfImproveCost(const PathPtr &path)
{
  Combination comb;
  if (!IsASolution(path,comb))
  {
    ROS_ERROR("Path is not a solution of the problem");
    return false;
  }
  if (!path->isCollisionFree())
  {
    GRAPH_TEST("path is in collision");
    return false;
  }

  if (m_best_path_per_combination.at(comb)->getCost()>path->getCost())
  {
    m_best_path_per_combination.at(comb)=path;
    if (m_best_path->getCost()>path->getCost())
    {
      m_best_path=path;
    }
  }
  return true;
}



}
