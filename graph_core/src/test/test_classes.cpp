#include <ros/ros.h>
#include <graph_core/metrics.h>
#include <graph_core/graph/node.h>
#include <graph_core/graph/path.h>
#include <graph_core/graph/tree.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "node");
  ros::NodeHandle nh;

  Eigen::VectorXd conf1(3);
  conf1.setConstant(0);
  conf1(0)=1.1;
  conf1(1)=0;
  conf1(2)=0;

  pathplan::NodePtr n1=std::make_shared<pathplan::Node>(conf1);
  Eigen::VectorXd conf2(3);
  conf2.setConstant(0);
  conf2(0)=1.2;
  conf2(1)=0;
  conf2(2)=2;
  pathplan::NodePtr n2=std::make_shared<pathplan::Node>(conf2);

  ROS_INFO("Test metrics.");
  pathplan::MetricsPtr metrics=std::make_shared<pathplan::Metrics>();
  ROS_INFO_STREAM("distance between configurations = " << metrics->cost(n1->getConfiguration(),n2->getConfiguration()));
  ROS_INFO_STREAM("distance between nodes = " << metrics->cost(n1,n2));

  ROS_INFO("Test Collision");
  pathplan::CollisionCheckerPtr checker=std::make_shared<pathplan::Cube3dCollisionChecker>();
  checker->check(conf1);
  checker->checkPath(conf1,conf2);

  ROS_INFO("Test Connection");
  pathplan::ConnectionPtr conn12=std::make_shared<pathplan::Connection>(n1,n2);
  conn12->setCost(metrics->cost(n1,n2));
  conn12->add();

  ROS_INFO("Test Path");

  Eigen::VectorXd conf3(3);
  conf3.setConstant(0);
  conf3(0)=0;
  conf3(1)=1.2;
  conf3(2)=3;
  pathplan::NodePtr n3=std::make_shared<pathplan::Node>(conf3);

  pathplan::ConnectionPtr conn23=std::make_shared<pathplan::Connection>(n2,n3);
  conn23->setCost(metrics->cost(n2,n3));
  conn23->add();

  Eigen::VectorXd conf4(3);
  conf4.setConstant(0);
  conf4(0)=-0.6;
  conf4(1)=1.1;
  conf4(2)=0;
  pathplan::NodePtr n4=std::make_shared<pathplan::Node>(conf4);

  pathplan::ConnectionPtr conn34=std::make_shared<pathplan::Connection>(n3,n4);
  conn34->setCost(metrics->cost(n3,n4));
  conn34->add();

  std::vector<pathplan::ConnectionPtr> connections;
  connections.push_back(conn12);
  connections.push_back(conn23);
  connections.push_back(conn34);
  pathplan::Path path(connections,metrics,checker);
  ROS_INFO_STREAM("path\n" << path);

  ROS_INFO("Test warp");
  path.warp();
  ROS_INFO_STREAM("path\n" << path);

  ROS_INFO("Test sampler");
  Eigen::VectorXd lb(3);
  Eigen::VectorXd ub(3);
  lb.setConstant(-M_PI);
  ub.setConstant(M_PI);

  pathplan::SamplerPtr sampler=std::make_shared<pathplan::InformedSampler>(conf1,conf2,lb,ub);
  for (unsigned int idx=0;idx<10;idx++)
  {
    ROS_INFO_STREAM("sample:\n"<<sampler->sample().transpose());
  }

  ROS_INFO("Test tree");
  pathplan::Tree tree(n1,pathplan::Forward,0.4,checker,metrics);

  pathplan::NodePtr new_node;
  tree.extend(sampler->sample(),new_node);

  ROS_INFO("Complete, deleting stuff");
  conn12->remove();
  conn23->remove();
  conn34->remove();
  return 0;
}
