#ifndef net_path_connection_201901040910
#define net_path_connection_201901040910

#include <net_path_core/net_path_core.h>

namespace ha_planner
{

class Connection: public std::enable_shared_from_this<ha_planner::Connection>
{
protected:
  NodePtr m_node1;
  NodePtr m_node2;
  double m_pheromone;
  double m_heuristic;
  double m_square_length;

  bool m_is_collision_checked;
  bool m_is_in_collision;
  double m_collision_probability;
  virtual void checkCollision(const planning_scene::PlanningSceneConstPtr& planning_scene);
  virtual void computeLength();
  virtual void computeHeuristic();
  const ConnectionParam& m_params;
public:
  Connection(const NodePtr& node1,
             const NodePtr& node2,
             const ConnectionParam& connection_parameters); // excepetion if not valid
  std::shared_ptr<ha_planner::Connection> pointer(){return shared_from_this();}

  const bool& isCollisionChecked(){return m_is_collision_checked;}
  bool isInCollision(const planning_scene::PlanningSceneConstPtr &planning_scene);
  void forceNotCollision(){m_is_collision_checked=true;m_is_in_collision=true;}

  const NodePtr& getOtherNode(const NodePtr& node);
  const double& getLength(){return m_square_length;}
  double getHeuristic();
  void updatePheromone(const double& new_pheromone);
  const double& getPheromone() const {return m_pheromone;}
  void print();
  bool isMember(const NodePtr& node){return (m_node1==node || m_node2==node);}
  void getNodes(NodePtr& node1, NodePtr& node2);
  const NodePtr& getParent(){return m_node1;}
  const NodePtr& getChild(){return m_node2;}

  Eigen::VectorXd versor();
  double dotProduct(const ConnectionPtr& conn);
  void flipDirection(){m_node1.swap(m_node2);}


};


}


#endif
