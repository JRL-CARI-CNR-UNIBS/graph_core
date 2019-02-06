#ifndef net_path_core_201901040910
#define net_path_core_201901040910

#include <ros/ros.h>
namespace cnr
{
namespace net
{
  class Connection;
  
  class Node 
  {
  protected:
    double m_heuristic;
    std::vector<double> m_q;
    std::vector<std::shared_ptr<Connection>> m_connections;
    
    virtual bool eraseFromConnections();
  public:
    Node();
    Node(const std::shared_ptr<Node>& node); // construct by copy
    
    void setJointValues(const std::vector<double>& q);
    void setHeuristic(const double& heuristic);
    
    double getHeuristic(){return m_heuristic;};
    
    virtual void generateFromNode(const std::shared_ptr<Node>& node, const std::vector<double>& parameters);
    virtual void generateRandom(const std::vector<double>& lower_bound, const std::vector<double>& upper_bound);
    
  };
  
  class Connection
  {
  protected:
    std::shared_ptr<Node> m_node1;
    std::shared_ptr<Node> m_node2;
    double m_pheromone;
    double m_heuristic;
    double m_length;
    bool checkCollision();
    void computeLength();
    
  public:
    Connection(const std::shared_ptr<Node>& node1, const std::shared_ptr<Node>& node2); // excepetion if not valid
    
    
    double getLength(){return m_length;};
    double getHeuristic(){return m_heuristic;};
  };
}
}


#endif