#ifndef net_path_core_201901040910
#define net_path_core_201901040910

#include <ros/ros.h>
#include <memory>
#include <moveit/planning_scene/planning_scene.h>
#include <random>


namespace ha_planner
{

Eigen::MatrixXd computeRotationMatrix(const std::vector<double>& p1, const std::vector<double>&  p2);

Eigen::VectorXd computeEllipsoid(const std::vector<double>& p1, const std::vector<double>&  p2, const double& cost);

double squareDistance(const std::vector<double>& q1, const std::vector<double>& q2);

std::vector<std::vector<double>> intermediatePoints(const std::vector<double> &q1, const std::vector<double> &q2, const double& distance_step);

struct NodeParams
{
  std::string group_name;
};
struct ConnectionParam
{
  std::string group_name;
  double checking_collision_distance;
};

class Node;
class Connection;
typedef std::shared_ptr<Node> NodePtr;
typedef std::shared_ptr<Connection> ConnectionPtr;
typedef std::vector<std::shared_ptr<Connection>> Path;
enum Direction {Any, Forward, Backward};

}

#include<net_path_core/node.h>
#include <net_path_core/connection.h>
#include <net_path_core/net.h>
#endif
