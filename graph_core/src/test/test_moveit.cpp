#include <ros/ros.h>
#include <graph_core/metrics.h>
#include <graph_core/occupancy_metrics.h>
#include <graph_core/graph/node.h>
#include <graph_core/graph/path.h>
#include <graph_core/graph/tree.h>
#include <graph_core/solvers/rrt_connect.h>
#include <graph_core/solvers/birrt.h>
#include <graph_core/solvers/rrt_star.h>
#include <graph_core/solvers/path_solver.h>
#include <graph_core/moveit_collision_checker.h>
#include <graph_core/local_informed_sampler.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "node");
  ros::NodeHandle nh;

  ros::AsyncSpinner aspin(4);

  std::string group_name = "ur5_on_guide";
  moveit::planning_interface::MoveGroupInterface move_group(group_name);
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr       kinematic_model = robot_model_loader.getModel();
  planning_scene::PlanningScenePtr planning_scene = std::make_shared<planning_scene::PlanningScene>(kinematic_model);
  robot_state::RobotState state = planning_scene->getCurrentState();


//  pathplan::MetricsPtr metrics=std::make_shared<pathplan::Metrics>();
  pathplan::MetricsPtr metrics = std::make_shared<pathplan::OccupancyMetrics>(nh);
  pathplan::CollisionCheckerPtr checker = std::make_shared<pathplan::MoveitCollisionChecker>(planning_scene, group_name);

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
      ROS_FATAL("joint name =%s, bound = [%f, %f]", joint_names.at(idx).c_str(), lb(idx), ub(idx));
    }
  }

  Eigen::VectorXd start_conf(7);
  start_conf << 1.32, 1.3542318049188016, -1.2795944053731603, 1.537418563788997, -1.8577960102702673, -1.7793031710049774, 1.1336506606321133;

  Eigen::VectorXd goal_conf(7);
  goal_conf << 0, 2, -1.2795944053731603, 1.537418563788997, -1.8577960102702673, -1.7793031710049774, 1.1336506606321133;

  pathplan::NodePtr start_node = std::make_shared<pathplan::Node>(start_conf);
  pathplan::NodePtr goal_node = std::make_shared<pathplan::Node>(goal_conf);

  pathplan::SamplerPtr sampler = std::make_shared<pathplan::InformedSampler>(start_conf, goal_conf, lb, ub);


  ROS_INFO("Test solver");
//  pathplan::RRTConnect solver(metrics,checker,sampler);
  pathplan::BiRRT solver(metrics, checker, sampler);
  solver.config(nh);
  solver.addGoal(goal_node);
  solver.addStart(start_node);

  pathplan::PathPtr solution;
  if (!solver.solve(solution, 1000))
  {
    ROS_INFO("No solutions found");
    return 0;
  }

  double rrt_cost = solution->cost();
  ROS_INFO_STREAM("found a solution with cost = " << solution->cost());

  pathplan::PathLocalOptimizer path_solver(checker, metrics);
  path_solver.config(nh);

  solution->setTree(solver.getStartTree());
  path_solver.setPath(solution);
  path_solver.solve(solution);
  ROS_INFO_STREAM("improve the solution to cost = " << solution->cost());

//  solution->resample(0.3);

  sampler->setCost(solution->cost());
  solver.getStartTree()->addBranch(solution->getConnections());


  pathplan::LocalInformedSamplerPtr local_sampler = std::make_shared<pathplan::LocalInformedSampler>(start_conf, goal_conf, lb, ub);

  for (unsigned int isol = 0; isol < solution->getConnections().size() - 1; isol++)
  {
    pathplan::ConnectionPtr conn = solution->getConnections().at(isol);
    if (conn->getChild()->getConfiguration().size() != 7)
    {
      ROS_ERROR_STREAM("size = " << conn->getChild()->getConfiguration().size());
      ROS_WARN_STREAM("node " << conn->getChild());
    }
    try
    {
      local_sampler->addBall(conn->getChild()->getConfiguration(), solution->cost() * 0.1);
    }
    catch (...)
    {
      ROS_ERROR_STREAM("size = " << conn->getChild()->getConfiguration().size());
      ROS_WARN_STREAM("node " << conn->getChild());
      return 0;
    }
  }
  local_sampler->setCost(solution->cost());

  ROS_INFO("Improving solution with RRT*");
  pathplan::RRTStar opt_solver(metrics, checker, local_sampler);
  opt_solver.addStartTree(solver.getStartTree());
  opt_solver.addGoal(goal_node);
  opt_solver.config(nh);

  std::vector<pathplan::NodePtr> white_list;
  white_list.push_back(goal_node);

  ros::Duration max_time(3);
  ros::Time t0 = ros::Time::now();

  int stall_gen = 0;
  int max_stall_gen = 200;

  std::mt19937 gen;
  std::uniform_int_distribution<> id = std::uniform_int_distribution<>(0, max_stall_gen);

  for (unsigned int idx = 0; idx < 1000; idx++)
  {

    if (ros::Time::now() - t0 > max_time)
      break;

    if (opt_solver.update(solution))
    {
      stall_gen = 0;
      ROS_INFO_STREAM("Iteration = " << idx << " , rrt* reduce cost to = " << solution->cost());
      path_solver.setPath(solution);
      solution->setTree(opt_solver.getStartTree());
//      path_solver.solve(solution);
      ROS_INFO_STREAM("local solver improve the solution to cost = " << solution->cost());

      local_sampler->setCost(solution->cost());
      sampler->setCost(solution->cost());
      opt_solver.getStartTree()->purgeNodes(sampler, white_list, true);

      local_sampler->clearBalls();
      for (unsigned int isol = 0; isol < solution->getConnections().size() - 1; isol++)
      {
        pathplan::ConnectionPtr conn = solution->getConnections().at(isol);
        if (conn->getChild()->getConfiguration().size() != 7)
        {
          ROS_ERROR_STREAM("size = " << conn->getChild()->getConfiguration().size());
          ROS_WARN_STREAM("node " << conn->getChild());
        }
        //if (conn->getCost()>1.01*conn->norm())
        {
          try
          {
            local_sampler->addBall(conn->getChild()->getConfiguration(), solution->cost() * 0.1);
          }
          catch (...)
          {
            ROS_ERROR_STREAM("size = " << conn->getChild()->getConfiguration().size());
            ROS_WARN_STREAM("node " << conn->getChild());
            return 0;
          }
        }
      }
    }
    else
    {
      opt_solver.getStartTree()->purgeNodes(sampler, white_list, false);
      stall_gen++;
    }


    if (idx % 10 == 0)
      ROS_INFO("iter=%u,tree with %u nodes", idx, opt_solver.getStartTree()->getNumberOfNodes());

    if (id(gen) < stall_gen)
      opt_solver.setSampler(sampler);
    else
      opt_solver.setSampler(local_sampler);

    if (stall_gen >= max_stall_gen)
      break;
  }

  double opt_cost = solution->cost();
  ROS_INFO_STREAM("solution cost = " << solution->cost());
  ROS_INFO("tree with %u nodes", opt_solver.getStartTree()->getNumberOfNodes());

  path_solver.setPath(solution);
  path_solver.solve(solution);
  ROS_INFO_STREAM("local solver improve the solution to cost = " << solution->cost());

  ROS_INFO_STREAM("solution\n" << *solution);

  return 0;
}
