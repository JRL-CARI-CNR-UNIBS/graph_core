#include <ros/ros.h>
#include <graph_core/graph/subtree.h>
#include <graph_core/narrow_pass_checker.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_subtree");
  ros::NodeHandle nh;

  pathplan::CollisionCheckerPtr checker=std::make_shared<pathplan::NarrowPassChecker>(0.1,1.0,1);
  pathplan::MetricsPtr metrics=std::make_shared<pathplan::Metrics>();

  int ndof=3;
  Eigen::VectorXd lb(ndof);
  Eigen::VectorXd ub(ndof);
  lb.setConstant(-10);
  ub.setConstant(10);
  pathplan::SamplerPtr sampler=std::make_shared<pathplan::InformedSampler>(lb,ub,lb,ub);

  sampler->sample();
  Eigen::VectorXd q=sampler->sample();
  pathplan::NodePtr root = std::make_shared<pathplan::Node>(sampler->sample());

  pathplan::TreePtr tree=std::make_shared<pathplan::Tree>(root,pathplan::Direction::Forward,1,checker,metrics);

  pathplan::NodePtr subtree_root;
  if (!tree->extend(sampler->sample(),subtree_root))
  {
    return 0;
  }
  ROS_INFO_STREAM("tree = " << *tree);


  ROS_INFO("growing parent tree");
  pathplan::NodePtr new_node;
  for (int idx=0;idx<100;idx++)
  {
    tree->connect(sampler->sample(),new_node);
  }

  pathplan::SubtreePtr subtree=pathplan::Subtree::createSubtree(tree,subtree_root);
  ROS_INFO_STREAM("tree = " << *tree);
  ROS_INFO_STREAM("subtree = " << *subtree);


  ROS_INFO("growing child tree");
  for (int idx=0;idx<100;idx++)
  {
    subtree->connect(sampler->sample(),new_node);
  }

  ROS_INFO_STREAM("tree = " << *tree);
  ROS_INFO_STREAM("subtree = " << *subtree);


  return 0;
}
