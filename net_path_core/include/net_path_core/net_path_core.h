#ifndef net_path_core_201901040910
#define net_path_core_201901040910

#include <ros/ros.h>
#include <memory>
#include <moveit/planning_scene/planning_scene.h>

namespace ha_planner
{

double squareDistance(const std::vector<double>& q1, const std::vector<double>& q2);

struct NodeParams
{
  std::string group_name;
};
struct ConnectionParam
{
  std::string group_name;
  double checking_collision_distance;
};

class Connection;

class Node: public std::enable_shared_from_this<Node>
{

protected:
  double m_heuristic;
  std::vector<double> m_q;
  double m_square_length;
  bool m_is_collision_checked;
  bool m_is_in_collision;
  virtual void checkCollision(const planning_scene::PlanningSceneConstPtr& planning_scene);
  const NodeParams& m_params;
  //reference member is not stored in memory, but you need to use it carefully: life cycle is not automatically managed. In this library there are no issues because Nodes and Connections are members of a Net.
  const ConnectionParam& m_conn_params;

public:
  std::vector<std::shared_ptr<Connection>> m_connections;

  Node(const std::vector<double>& q,
       const NodeParams& node_parameters,
       const ConnectionParam& connection_parameters);
  std::shared_ptr<ha_planner::Node> pointer(){return shared_from_this();}

  virtual void computeHeuristic(const std::vector<std::shared_ptr<Node>>& end_points);
  const unsigned int getConnectionsNumber(){return m_connections.size();}
  const double& getHeuristic(){return m_heuristic;}
  const std::vector<double>& getJoints() const {return m_q;}
  bool isInCollision(const planning_scene::PlanningSceneConstPtr &planning_scene);
  virtual void createConnections(const double& max_distance,
                                 const double& min_distance,
                                 const unsigned int& max_connections,
                                 const std::vector<std::shared_ptr<Node>>& actual_nodes,
                                 unsigned int& connections,
                                 bool force_add=false);

  int rouletteWheel();

  void addConnection(std::shared_ptr<Connection>& connection, unsigned int max_connections);
  bool removeConnection(std::shared_ptr<Connection>& connection);

  void print();
};

class Connection: public std::enable_shared_from_this<ha_planner::Connection>
{
protected:
  std::shared_ptr<Node> m_node1;
  std::shared_ptr<Node> m_node2;
  double m_pheromone;
  double m_heuristic;
  double m_square_length;

  bool m_is_collision_checked;
  bool m_is_in_collision;
  virtual void checkCollision(const planning_scene::PlanningSceneConstPtr& planning_scene);
  virtual void computeLength();
  virtual void computeHeuristic();
  const ConnectionParam& m_params;
public:
  Connection(const std::shared_ptr<Node>& node1,
             const std::shared_ptr<Node>& node2,
             const ConnectionParam& connection_parameters); // excepetion if not valid
  std::shared_ptr<ha_planner::Connection> pointer(){return shared_from_this();}

  bool isInCollision(const planning_scene::PlanningSceneConstPtr &planning_scene);
  const std::shared_ptr<Node>& getOtherNode(const std::shared_ptr<Node>& node);
  const double& getLength(){return m_square_length;}
  const double& getHeuristic(){return m_heuristic;}
  void updatePheromone(const double& new_pheromone);
  const double& getPheromone() const {return m_pheromone;}

};


class Net
{
protected:
//  std::vector<std::shared_ptr<Connection>> m_connections;
  std::vector<std::shared_ptr<Node>> m_nodes;
  unsigned int m_dof;
  unsigned int m_grid_global_nodes;
  unsigned int m_points_per_dimension;

  double m_max_square_length;
  double m_min_square_length;
  double m_max_connections_per_node;

  double m_sqrt_evaporation_ratio;
  double m_max_pheromone;
  double m_min_pheromone;

  std::shared_ptr<Node> m_start_node;
  std::vector<std::shared_ptr<Node>> m_end_nodes;

  double m_best_cost;
  std::vector<unsigned int> m_best_path;

  NodeParams m_node_params;
  ConnectionParam m_conn_params;


  const planning_scene::PlanningSceneConstPtr m_planning_scene;

  std::shared_ptr<Node> addNodeWithConnections(const std::vector<double>& q, const bool& force_add=false);

public:
  Net(const unsigned int& dof,
      const planning_scene::PlanningSceneConstPtr &planning_scene);

  virtual void generateNodesFromStartAndEndPoints(const std::vector<double>& start_point, const std::vector<std::vector<double>>& end_points);
  virtual void generateNodesFromGrid(const unsigned int& number_of_nodes, const std::vector<double>& lower_bound, const std::vector<double>& upper_bound);
  //  virtual void generateNodesFromConnection(const std::shared_ptr<Connection>& connection, const unsigned int& number_of_nodes, const std::vector<double>& parameters);
  //  virtual void generateNodesFromNode(const std::shared_ptr<Node>& node, const unsigned int& number_of_nodes, const std::vector<double>& parameters);

  void updateNodeHeuristic();
  void evaporatePheromone();
  void distributePheromone(const double& gain);

  bool sendAnt(std::vector<unsigned int>& path);
  double computePathCost(const std::vector<unsigned int>& path);
  bool checkCollision(const std::vector<unsigned int>& path);

  bool runAntCycle(const unsigned int& n_ants);

  void printBestPath();
//  bool runLocalSearch();
//  bool eliminationFilter();


};
}


#endif
