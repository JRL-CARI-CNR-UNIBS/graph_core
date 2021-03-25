#include <ros/ros.h>

#include <graph_core/moveit_collision_checker.h>
#include <graph_core/parallel_moveit_collision_checker.h>
#include <graph_core/sampler.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <graph_core/graph/graph_display.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "node");
  ros::NodeHandle nh;

  ros::AsyncSpinner aspin(4);

  std::string group_name = "manipulator";
  moveit::planning_interface::MoveGroupInterface move_group(group_name);
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr       kinematic_model = robot_model_loader.getModel();
  planning_scene::PlanningScenePtr planning_scene = std::make_shared<planning_scene::PlanningScene>(kinematic_model);


  int num_threads =nh.param("number_of_threads",5);
  ROS_INFO("Compare MoveitCollisionChecker and ParallelMoveitCollisionChecker with %d threads",num_threads);
  double steps=nh.param("steps",0.01);

  robot_state::RobotState state = planning_scene->getCurrentState();

  pathplan::CollisionCheckerPtr checker1 = std::make_shared<pathplan::MoveitCollisionChecker>(planning_scene, group_name,steps);
  pathplan::CollisionCheckerPtr checker2 = std::make_shared<pathplan::ParallelMoveitCollisionChecker>(planning_scene, group_name,num_threads,steps);

  std::vector<std::string> joint_names = kinematic_model->getJointModelGroup(group_name)->getActiveJointModelNames();

  unsigned int dof = joint_names.size();
  Eigen::VectorXd lb(dof);
  Eigen::VectorXd ub(dof);

  for (unsigned int idx = 0; idx < dof; idx++)
  {
    const robot_model::VariableBounds& bounds = kinematic_model->getVariableBounds(joint_names.at(idx));
    if (bounds.position_bounded_)
    {
      lb(idx) = bounds.min_position_;
      ub(idx) = bounds.max_position_;
    }
  }
  pathplan::SamplerPtr sampler = std::make_shared<pathplan::InformedSampler>(lb, ub, lb, ub);

  pathplan::Display display(planning_scene,group_name);

  ROS_INFO("Testing connections with feasible start and stop node and length 2");
  int iters=nh.param("iterations",100000);
  int attempts=0;
  double time_single=0;
  double time_par=0;
  int errors=0;
  for (int idx=0;idx<iters;idx++)
  {
    if (!ros::ok())
      break;
    ROS_INFO_THROTTLE(10,"%d iter of %d",idx,iters);
    Eigen::VectorXd q1=sampler->sample();

    if (!checker1->check(q1))
      continue;

    Eigen::VectorXd q2=sampler->sample();
    q2=q1+(q2-q1)*2.0/(q2-q1).norm();

    if (!checker1->check(q2))
      continue;

    attempts++;
    ros::WallTime t0=ros::WallTime::now();
    bool fl1=checker1->checkPath(q1,q2);
    ros::WallTime t1=ros::WallTime::now();
    bool fl2=checker2->checkPath(q1,q2);
    ros::WallTime t2=ros::WallTime::now();
    time_single+=(t1-t0).toSec();
    time_par+=(t2-t1).toSec();
    if (fl1!=fl2)
    {
      errors++;
    }
  }
  ROS_INFO("MoveitCollisionChecker: Average time = %f ms on %d attempts",1e3*time_single/(double)attempts,attempts);
  ROS_INFO("ParallelMoveitCollisionChecker: Average time = %f ms on %d attempts",1e3*time_par/(double)attempts,attempts);
  ROS_INFO("Erros = %d over %d attempts",errors,attempts);


  ROS_INFO("Testing random connections with length 2");
  attempts=0;
  time_single=0;
  time_par=0;
  errors=0;
  for (int idx=0;idx<iters;idx++)
  {
    if (!ros::ok())
      break;
    ROS_INFO_THROTTLE(10,"%d iter of %d",idx,iters);
    Eigen::VectorXd q1=sampler->sample();


    Eigen::VectorXd q2=sampler->sample();
    q2=q1+(q2-q1)*2.0/(q2-q1).norm();


    attempts++;
    ros::WallTime t0=ros::WallTime::now();
    bool fl1=checker1->checkPath(q1,q2);
    ros::WallTime t1=ros::WallTime::now();
    bool fl2=checker2->checkPath(q1,q2);
    ros::WallTime t2=ros::WallTime::now();
    time_single+=(t1-t0).toSec();
    time_par+=(t2-t1).toSec();
    if (fl1!=fl2)
    {
      pathplan::NodePtr n1=std::make_shared<pathplan::Node>(q1);
      pathplan::NodePtr n2=std::make_shared<pathplan::Node>(q2);

      pathplan::ConnectionPtr conn=std::make_shared<pathplan::Connection>(n1,n2);
      display.displayConnection(conn);
      errors++;
    }
  }
  ROS_INFO("MoveitCollisionChecker: Average time = %f ms on %d attempts",1e3*time_single/(double)attempts,attempts);
  ROS_INFO("ParallelMoveitCollisionChecker: Average time = %f ms on %d attempts",1e3*time_par/(double)attempts,attempts);
  ROS_INFO("Erros = %d over %d attempts",errors,iters);


  return 0;
}
