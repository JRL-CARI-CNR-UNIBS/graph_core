#include <ros/ros.h>
#include <graph_core/metrics.h>
#include <graph_core/graph/node.h>
#include <graph_core/graph/path.h>
#include <graph_core/graph/tree.h>
#include <graph_core/solvers/rrt_connect.h>
#include <graph_core/solvers/birrt.h>
#include <graph_core/solvers/rrt_star.h>
#include <graph_core/solvers/path_solver.h>
#include <graph_core/moveit_collision_checker.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "node");
  ros::NodeHandle nh;


  std::string group_name="vs060a3";
  moveit::planning_interface::MoveGroupInterface move_group(group_name);
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr       kinematic_model = robot_model_loader.getModel();
  planning_scene::PlanningScenePtr planning_scene=std::make_shared<planning_scene::PlanningScene>(kinematic_model);
  robot_state::RobotState state = planning_scene->getCurrentState();


  pathplan::MetricsPtr metrics=std::make_shared<pathplan::Metrics>();
  pathplan::CollisionCheckerPtr checker=std::make_shared<pathplan::MoveitCollisionChecker>(planning_scene,group_name);


  Eigen::VectorXd lb(6);
  lb.setConstant(-M_PI);
  Eigen::VectorXd ub(6);
  ub.setConstant(M_PI);

  Eigen::VectorXd start_conf(6);
  start_conf.setConstant(0);

  Eigen::VectorXd goal_conf(6);
//  goal_conf << -0.4883333241387038, 1.345835743383781, 1.6106694164020787, 3.1415750729640006,  -0.1850936022493632, -3.6299086982917856;
  goal_conf << -0.48833332341687274, 1.3458357251491204, 1.6106694320322434, -3.1416126113214666,  -0.1850932260114333, 2.6532789862227277;
//  goal_conf << -0.4883333241398625, 1.345835745411008, 1.6106694148309002, -1.7592098138836945e-05,  0.18509360138518802, -0.48831603322903483;
//  goal_conf << -0.4883333241307036, 1.3458357460587942, 1.610669414326582, -1.762597371424766e-05,   0.1850936018901623, 5.794869307832635;
//  goal_conf << -0.48833332558079007, 1.3458355912122064, 1.6106695348037567, 3.141580330146525,   -0.18509357070276614, 2.6532713506808197;
//  goal_conf << -0.4883333241161222, 1.3458357446041884, 1.610669415455745, -3.141610337022636,  -0.18509360178504436, -3.629908595466747;

  pathplan::NodePtr start_node=std::make_shared<pathplan::Node>(start_conf);
  pathplan::NodePtr goal_node=std::make_shared<pathplan::Node>(goal_conf);

  pathplan::SamplerPtr sampler=std::make_shared<pathplan::InformedSampler>(start_conf,goal_conf,lb,ub);


  ROS_INFO("Test solver");
//  pathplan::RRTConnect solver(metrics,checker,sampler);
  pathplan::BiRRT solver(metrics,checker,sampler);
  solver.config(nh);
  solver.addGoal(goal_node);
  solver.addStart(start_node);

  pathplan::PathPtr solution;
  if (!solver.solve(solution,1000))
  {
    ROS_INFO("No solutions found");
    return 0;
  }

  double rrt_cost=solution->cost();

  pathplan::PathLocalOptimizer path_solver(checker,metrics);
  path_solver.config(nh);

  ROS_INFO_STREAM("found a solution with cost = "<<solution->cost());
  path_solver.setPath(solution);
  path_solver.solve(solution);
  ROS_INFO_STREAM("improve the solution to cost = "<<solution->cost());


  sampler->setCost(solution->cost());
  solver.getStartTree()->addBranch(solution->getConnections());
  ROS_INFO("Improving solution with RRT*");
  pathplan::RRTStar opt_solver(metrics,checker,sampler);
  opt_solver.addStartTree(solver.getStartTree());
  opt_solver.addGoal(goal_node);
  opt_solver.config(nh);

  std::vector<pathplan::NodePtr> white_list;
  white_list.push_back(goal_node);

  ros::Duration max_time(5);
  ros::Time t0=ros::Time::now();

  for (unsigned int idx=0;idx<5000;idx++)
  {
    if (ros::Time::now()-t0>max_time)
      break;

    if (opt_solver.update(solution))
    {
      ROS_INFO_STREAM("Iteration = "<<idx<<" , rrt* reduce cost to = "<<solution->cost());
      path_solver.setPath(solution);
      path_solver.solve(solution);
      ROS_INFO_STREAM("local solver improve the solution to cost = "<<solution->cost());

      sampler->setCost(solution->cost());
      opt_solver.getStartTree()->purgeNodes( sampler , white_list,true);
    }
    else
      opt_solver.getStartTree()->purgeNodes( sampler , white_list,false);

    if (idx%100==0)
      ROS_INFO("iter=%u,tree with %u nodes",idx,opt_solver.getStartTree()->getNumberOfNodes());
  }

  double opt_cost=solution->cost();
  ROS_INFO_STREAM("solution cost = "<<solution->cost());
  ROS_INFO("tree with %u nodes",opt_solver.getStartTree()->getNumberOfNodes());

  ROS_INFO_STREAM("solution\n"<<*solution);

  return 0;
}
