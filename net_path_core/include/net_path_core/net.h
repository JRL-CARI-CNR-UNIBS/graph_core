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

  std::vector<double> m_lb;
  std::vector<double> m_ub;

  double m_max_square_length;
  double m_min_square_length;
  double m_max_length;
  double m_min_length;

  double m_sqrt_evaporation_ratio;
  double m_max_pheromone;
  double m_min_pheromone;

  NodePtr m_start_node;
  std::vector<NodePtr> m_end_nodes;
  std::map<NodePtr,std::pair<double,Path>> m_best_path_per_goal;
  std::map<NodePtr,Eigen::MatrixXd> m_rot_matrix;
  TreePtr m_start_tree;
  std::vector<TreePtr> m_goal_trees;
  double m_best_cost;
  double m_estimated_cost;
  double m_utopia_cost;
  std::map<NodePtr,double> m_utopia_costs;
  Path m_best_path;

  std::random_device m_rd;
  std::mt19937 m_gen;
  std::uniform_real_distribution<double> m_ud;

  NodeParams m_node_params;
  ConnectionParam m_conn_params;


  planning_scene::PlanningSceneConstPtr m_planning_scene;

  NodePtr searchClosestNode(const std::vector<double>& q);
  NodePtr addNodeToTheGrid(const std::vector<double>& q);
  NodePtr addNodeToTheGrid(const std::vector<double>& q, const NodePtr& closest_node);
  Path addSubGridToTheGrid(const std::vector<std::vector<double>>& q, const NodePtr& closest_node);

  double getPathDistance(const Path& path, const NodePtr& node);

  void removeNodeWithConnections(NodePtr& node);

  void pruning(const std::vector<Path>& paths);

  std::vector<double> sample(const NodePtr& endnode);
  std::vector<std::vector<double> > getSamples(const NodePtr& endnode, const unsigned int& number_of_samples=1);



public:
  Net(const unsigned int& dof,
      const std::string& group_name,
      const planning_scene::PlanningSceneConstPtr &planning_scene,
      const std::vector<double> scaling,
      const std::vector<double> lb,
      const std::vector<double> ub);

  // sampling algorithms
  bool runRRTConnect();
  //net methods
  unsigned int getNodeNumber(){return m_nodes.size();}
  bool isSolutionFound();
  double getBestCost(){return m_best_cost;}

  void setPlanningScene ( const planning_scene::PlanningSceneConstPtr& planning_scene )
  {
    m_planning_scene=planning_scene;
  }
  // nodes generations
  virtual void generateNodesFromStartAndEndPoints(const std::vector<double>& start_point, const std::vector<std::vector<double>>& end_points);
  virtual void generateNodesFromEllipsoid(const int& number_of_nodes);

  // path
  virtual Path warpPath(const unsigned int& number_of_trials, const Path& path);
  virtual bool warpPath2(const unsigned int& number_of_trials);
  virtual Path splitPath(const unsigned int& number_of_trials, const Path& path);
  virtual bool splitPath2(const unsigned int& number_of_trials);
  // node removal functions
  unsigned int removeUnconnectedNodes();
  unsigned int removeLowPheromoneConnections(const unsigned int& number_of_nodes_to_check);

  // Ant colony methods
  void updateNodeHeuristic();
  void evaporatePheromone();
  void distributePheromone(const double& gain);
  bool sendAnt(Path& path);
  bool runAntCycle(const unsigned int& n_ants);

  // path methods
  bool storeIfImproveCost(const Path& path);
  bool storeIfImproveCost(const Path& path, const double& cost);
  bool checkIfImproveCost(const Path& path);
  bool checkIfImproveCost(const Path& path, const double& cost);

  double computePathCost(const Path& path);
  bool isCollisionFree(const Path& path);
  const Path& getBestPathRef();
  std::vector<std::vector<double>> getBestPath();
  std::vector<std::vector<double>> getPath(const Path& path);
  void printBestPath();
  void printPath(const Path& path);

  Path pruningPath(Path path);



};


}


#endif
