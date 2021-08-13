#include <ros/ros.h>

#include <graph_core/moveit_collision_checker.h>
#include <graph_core/parallel_moveit_collision_checker.h>
#include <graph_core/sampler.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <graph_core/graph/graph_display.h>
#include <graph_core/solvers/multigoal.h>


int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_net");
  ros::NodeHandle nh;

  ros::AsyncSpinner aspin(4);

  std::string group_name = "manipulator";
  moveit::planning_interface::MoveGroupInterface move_group(group_name);
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr       kinematic_model = robot_model_loader.getModel();
  planning_scene::PlanningScenePtr planning_scene = std::make_shared<planning_scene::PlanningScene>(kinematic_model);


  int num_threads =nh.param("number_of_threads",5);
  double steps=nh.param("steps",0.01);
  double max_time=nh.param("computation_time",5);

  robot_state::RobotState state = planning_scene->getCurrentState();

  pathplan::CollisionCheckerPtr checker = std::make_shared<pathplan::ParallelMoveitCollisionChecker>(planning_scene, group_name,num_threads,steps);
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
  display.clearMarkers();

  int n_waypoints=nh.param("net_waypoints",10);
  ROS_INFO("Creating %d waypoints",n_waypoints);
  std::vector<pathplan::NodePtr> waypoints;

  while ((int)waypoints.size()<n_waypoints)
  {
    if(!ros::ok())
      break;
    Eigen::VectorXd q=sampler->sample();
    if (checker->check(q))
    {
      pathplan::NodePtr n=std::make_shared<pathplan::Node>(q);
      waypoints.push_back(n);
      display.displayNode(n);
    }
  }

  pathplan::MetricsPtr metrics_=std::make_shared<pathplan::Metrics>();


  std::map<std::pair<pathplan::NodePtr,pathplan::NodePtr>,pathplan::PathPtr> net;
  double icolor=0;
  for (const pathplan::NodePtr& start: waypoints)
  {
    std::cout << "node " << start <<" is connected with: ";
    int connections=0;
    icolor++;
    std::vector<double> color(4);
    color.at(3)=1.0;
    color.at(0)=icolor++/n_waypoints;

    for (const pathplan::NodePtr& goal: waypoints)
    {
      if (start==goal)
      {
        continue;
      }
      if (not ros::ok())
        return 0;
      pathplan::TreeSolverPtr solver=std::make_shared<pathplan::MultigoalSolver>(metrics_, checker, sampler);
      if (!solver->config(nh))
      {
        ROS_ERROR("Unable to configure the planner");
        return 0;
      }
      solver->addStart(start);
      pathplan::NodePtr tmp_goal=std::make_shared<pathplan::Node>(goal->getConfiguration());
      solver->addGoal(tmp_goal);
      pathplan::PathPtr solution;
      if (solver->solve(solution,1000000,max_time))
      {
        std::pair<pathplan::NodePtr,pathplan::NodePtr> p(start,goal);
        std::pair<std::pair<pathplan::NodePtr,pathplan::NodePtr>,pathplan::PathPtr> s(p,solution);
        net.insert(s);
        connections++;
        display.displayPath(solution,"pathplan",color);

      }
    }
    std::cout << connections << " waypoints"<< std::endl;
  }

  return 0;
}

