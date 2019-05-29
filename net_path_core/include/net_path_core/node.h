#ifndef net_path_node_201901040910
#define net_path_node_201901040910

#include <net_path_core/net_path_core.h>

namespace ha_planner
{

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
  std::vector<ConnectionPtr> m_connections;
  std::vector<ConnectionPtr> m_parent_connections;
  std::vector<ConnectionPtr> m_child_connections;

  Node(const std::vector<double>& q,
       const NodeParams& node_parameters,
       const ConnectionParam& connection_parameters);
  std::shared_ptr<ha_planner::Node> pointer(){return shared_from_this();}

  virtual void computeHeuristic(const std::vector<NodePtr>& end_points);
  const unsigned int getConnectionsNumber(){return m_connections.size();}
  const double& getHeuristic(){return m_heuristic;}
  const std::vector<double>& getJoints() const {return m_q;}
  const bool& isCollisionChecked(){return m_is_collision_checked;}
  bool isInCollision(const planning_scene::PlanningSceneConstPtr &planning_scene);
  virtual NodePtr createConnections(const double& max_distance,
                                 const double& min_distance,
                                 const std::vector<NodePtr>& actual_nodes,
                                 unsigned int& connections);

  int rouletteWheel(const Direction direction=Any);

  bool isUnconnected(const planning_scene::PlanningSceneConstPtr &planning_scene);
  void addConnection(const ConnectionPtr& connection);
  bool removeConnection(ConnectionPtr& connection);

  const NodePtr& findParent();
  ConnectionPtr findConnectionToParent();

  std::vector<NodePtr> getAncestors(const unsigned int& level=1);
  std::vector<NodePtr> getDescendants(const unsigned int& level=1);
  bool checkIfConnectedWith(const NodePtr& node, ConnectionPtr& connection);

  void print();
};

}


#endif
