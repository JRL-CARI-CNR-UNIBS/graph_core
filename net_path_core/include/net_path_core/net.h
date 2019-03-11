#ifndef net_path_net_201901040910
#define net_path_net_201901040910

#include <net_path_core/net_path_core.h>

namespace ha_planner
{

class Net
{
protected:
  std::vector<NodePtr> m_nodes;
  unsigned int m_dof;
  unsigned int m_grid_global_nodes;
  unsigned int m_points_per_dimension;

  std::vector<double> m_lb;
  std::vector<double> m_ub;

  double m_max_square_length;
  double m_min_square_length;

  double m_sqrt_evaporation_ratio;
  double m_max_pheromone;
  double m_min_pheromone;

  NodePtr m_start_node;
  std::vector<NodePtr> m_end_nodes;
  std::map<NodePtr,std::pair<double,Path>> m_best_path_per_goal;
  std::vector<Eigen::MatrixXd> m_rot_matrix;

  double m_best_cost;
  Path m_best_path;


  NodeParams m_node_params;
  ConnectionParam m_conn_params;


  const planning_scene::PlanningSceneConstPtr m_planning_scene;

  NodePtr addNodeWithConnections(const std::vector<double>& q, const bool& force_add=false);

  bool addNodeToTheClosestNodeOfTheTree(const std::vector<double>& q, std::vector<NodePtr>& tree);

  void removeNodeWithConnections(NodePtr& node);
public:
  Net(const unsigned int& dof,
      const planning_scene::PlanningSceneConstPtr &planning_scene);

  virtual void generateNodesFromStartAndEndPoints(const std::vector<double>& start_point, const std::vector<std::vector<double>>& end_points);
  virtual void generateNodesFromGrid(const int& number_of_nodes, const std::vector<double>& lower_bound, const std::vector<double>& upper_bound);
  virtual void generateRandomNodesFromGrid(const int& number_of_nodes, const std::vector<double>& lower_bound, const std::vector<double>& upper_bound);
  virtual void generateNodesFromEllipsoid(const int& number_of_nodes);

  virtual bool rrtConnectSingleGoal(const NodePtr& node);
  virtual bool rrtConnect();

  unsigned int removeUnconnectedNodes();
  unsigned int removeLowPheromoneConnections(const unsigned int& number_of_nodes_to_check);

  void updateNodeHeuristic();
  void evaporatePheromone();
  void distributePheromone(const double& gain);

  bool sendAnt(Path& path);
  double computePathCost(const Path& path);
  bool checkCollision(const Path& path);

  bool runAntCycle(const unsigned int& n_ants);

  void printBestPath();
  void printPath(const Path& path);

  void print();

  unsigned int getNodeNumber(){return m_nodes.size();}
  const Path& getBestPathRef();
  std::vector<std::vector<double>> getBestPath();
  std::vector<std::vector<double>> getPath(const Path& path);
  bool isSolutionFound();

  Path simplifyIntermidiatePoint(const Path& path);

//  bool runLocalSearch();
//  bool eliminationFilter();



};


}


#endif
