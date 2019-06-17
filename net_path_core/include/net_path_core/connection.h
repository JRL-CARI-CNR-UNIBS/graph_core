#ifndef net_path_connection_201901040910
#define net_path_connection_201901040910

#include <net_path_core/net_path_core.h>

namespace ha_planner
{

class Connection: public std::enable_shared_from_this<ha_planner::Connection>
{
protected:
  NodePtr m_parent;
  NodePtr m_child;
  double m_pheromone;
  double m_heuristic;
  double m_square_length;

  bool m_is_collision_checked;
  bool m_is_in_collision;
  double m_collision_probability;
  virtual void checkCollisionNew(const planning_scene::PlanningSceneConstPtr& planning_scene);
  virtual bool checkCollisionIteration(const planning_scene::PlanningSceneConstPtr& planning_scene,
                                       const std::vector<double>& q1,
                                       const std::vector<double>& q2,
                                       robot_state::RobotState& state);
  virtual void computeLength();
  virtual void computeHeuristic();
  const ConnectionParam& m_params;
public:
  Connection(const NodePtr& parent,
             const NodePtr& child,
             const ConnectionParam& connection_parameters); // excepetion if not valid


  std::shared_ptr<ha_planner::Connection> pointer(){return shared_from_this();}

  const bool& isCollisionChecked(){return m_is_collision_checked;}
  bool isInCollision(const planning_scene::PlanningSceneConstPtr &planning_scene);
  void forceNotCollision(){m_is_collision_checked=true;m_is_in_collision=false;}
  void forceCollision(){m_is_collision_checked=true;m_is_in_collision=true;}

  const NodePtr& getOtherNode(const NodePtr& node);
  const double& getLength(){return m_square_length;}
  double getHeuristic();
  void updatePheromone(const double& new_pheromone);
  const double& getPheromone() const {return m_pheromone;}
  void print();
  bool isMember(const NodePtr& node){return (m_parent==node || m_child==node);}
  void getNodes(NodePtr& parent, NodePtr& child);
  const NodePtr& getParent(){return m_parent;}
  const NodePtr& getChild(){return m_child;}


  Eigen::VectorXd versor();
  double dotProduct(const ConnectionPtr& conn);
  void flipDirection();
  ConnectionPtr createFlippedConnection();
  void registerConnection();

};


}


#endif
