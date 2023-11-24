#include <graph_core/graph/connection.h>
#include <graph_core/graph/node.h>
#include <ros/ros.h>
#include <eigen3/Eigen/Core>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "prova_graph_core");
  ros::NodeHandle nh;

  Eigen::VectorXd q1(6); q1.setRandom();
  Eigen::VectorXd q2(6); q2.setRandom();

  ROS_INFO_STREAM("q1 "<<q1.transpose());
  ROS_INFO_STREAM("q2 "<<q2.transpose());

  pathplan::NodePtr parent = std::make_shared<pathplan::Node>(q1);
  pathplan::NodePtr child  = std::make_shared<pathplan::Node>(q2);

  ROS_INFO_STREAM("parent "<<*parent);
  ROS_INFO_STREAM("child "<<*child);

  pathplan::ConnectionPtr conn = std::make_shared<pathplan::Connection>(parent,child);
  conn->add();
  ROS_INFO_STREAM("conn "<<*conn);

//  ROS_INFO("QUA0");
//  parent->disconnect();
//  ROS_INFO("QUA1");
//  child->disconnect();
//  ROS_INFO("QUA2");
//  conn->remove();
//  ROS_INFO("QUA3");

  return 0;
}
