#ifndef tree_path_net_20190504
#define tree_path_net_20190504

#include <net_path_core/net_path_core.h>

namespace ha_planner
{

class Tree
{
protected:
  std::vector<NodePtr>& m_nodes;
  const NodeParams& m_node_parameters;
  const ConnectionParam& m_connection_parameters;

  std::vector<NodePtr> m_tree_nodes;
  std::vector<ConnectionPtr> m_tree_connections;
  unsigned int m_dof;
  NodePtr m_root_node;

  planning_scene::PlanningSceneConstPtr m_planning_scene;
  double m_max_square_length;
  double m_max_length;
  Direction m_direction;

  double m_frontier_threshold=3;
  double m_frontier_ratio=0.1;
  unsigned int m_non_frontier_count=1;
  unsigned int m_frontier_count=1;
  bool m_connect_mode=true;
  double m_temperature=10;
  double m_temperature_rate=1.05;
  double m_max_cost=0;
  double m_min_cost=1e6;

  std::vector<NodePtr>& m_end_nodes;
public:
  Tree(const NodePtr& root_node,
        const NodeParams& node_parameters,
        const ConnectionParam& connection_parameters,
        const planning_scene::PlanningSceneConstPtr& planning_scene,
        const double& max_square_length,
        const Direction& direction,
        std::vector<NodePtr>& nodes, std::vector<NodePtr>& endnodes);

//  ~Tree();

  bool createAndExtend(const std::vector<double>& q, NodePtr& last_add_node);
  bool extend(const NodePtr& n, NodePtr& last_add_node);

  bool createAndExtendFromNode(const std::vector<double>& q, const NodePtr starting_node, NodePtr& last_add_node);
  bool extendFromNode(const NodePtr& n, const NodePtr starting_node, NodePtr& last_add_node);

  void changeTreeDirection(const Direction& direction);
  bool changeRoot(const NodePtr& node);
  bool addSubTree(const TreePtr& subtree);

  bool tryConnectWith(const NodePtr& n);
  bool getPathToNode(const NodePtr& n, Path& path) const;
  NodePtr getRoot() const {return m_root_node;}
  std::vector<ConnectionPtr> getConnections() const {return m_tree_connections;}
  Direction getDirection() const {return m_direction;}
  const std::vector<NodePtr>& getNodes() const {return m_tree_nodes;}
  unsigned int size()const {return m_tree_nodes.size();}

  bool minExpansionControl(const double& dist);
  bool transitionTest(const ConnectionPtr& conn);
};


}


#endif
