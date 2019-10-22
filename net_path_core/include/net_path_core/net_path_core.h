#ifndef net_path_core_201901040910
#define net_path_core_201901040910

#include <ros/ros.h>
#include <memory>
#include <moveit/planning_scene/planning_scene.h>
#include <random>

#define ROS_PROTO(...) ROS_LOG(::ros::console::levels::Fatal, ROSCONSOLE_DEFAULT_NAME, __VA_ARGS__)

namespace ha_planner
{

Eigen::MatrixXd computeRotationMatrix(const std::vector<double>& p1, const std::vector<double>&  p2);

Eigen::VectorXd computeEllipsoid(const std::vector<double>& p1, const std::vector<double>&  p2, const double& cost);

double squareDistance(const std::vector<double>& q1, const std::vector<double>& q2);

std::vector<std::vector<double>> intermediatePoints(const std::vector<double> &q1, const std::vector<double> &q2, const std::vector<double>& unscaling, const double& distance_step);

struct NodeParams
{
  std::string group_name;
};
struct ConnectionParam
{
  std::string group_name;
  double checking_collision_distance;
  std::vector<double> scaling;
  std::vector<double> unscaling;
  double weigth=1;
};
struct ParticleParam
{
  double c_l=1; //weigth of local best
  double c_g=1; //weigth of global best
  double w=1;   //particle mass
  unsigned int dof; // number of degree of freedom
};

class Node;
class Connection;
class Particle;
class Tree;
typedef std::shared_ptr<Node> NodePtr;
typedef std::shared_ptr<Connection> ConnectionPtr;
typedef std::shared_ptr<Tree> TreePtr;
typedef std::shared_ptr<Particle> ParticlePtr;
typedef std::vector<std::shared_ptr<Connection>> Path;
enum Direction {Forward, Backward};

}

#include<net_path_core/node.h>
#include <net_path_core/connection.h>
#include <net_path_core/tree.h>
#include<net_path_core/particle.h>
#include <net_path_core/net.h>
#endif
