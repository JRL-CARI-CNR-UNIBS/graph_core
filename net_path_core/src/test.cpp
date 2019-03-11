#include <ros/ros.h>
#include <net_path_core/net_path_core.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit_msgs/ExecuteTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "cartesian_trajectory_client_from_yaml");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(4);
  spinner.start();


  actionlib::SimpleActionClient<moveit_msgs::ExecuteTrajectoryAction> ac(nh, "/execute_trajectory");
  moveit::planning_interface::MoveGroupInterface move_group("ur10");


  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
  planning_scene::PlanningScenePtr planning_scene=std::make_shared<planning_scene::PlanningScene>(kinematic_model);
  robot_state::RobotState state = planning_scene->getCurrentState();

  moveit::core::JointModelGroup* group=kinematic_model->getJointModelGroup("ur10");

  unsigned int dof=group->getActiveJointModelNames().size();
//  moveit::core::JointBoundsVector limits=group->getActiveJointModelsBounds();
//  moveit::core::JointModel::Bounds bounds;
//  bounds.at(0).acceleration_bounded_;


  ha_planner::Net net(dof,planning_scene);


  std::vector<double> start_point{0.5331597005659249, -1.0113056136201222, 1.7873012825471348, -0.7861373812211763, 2.124856370025035, 0.9378274160454515};
  std::vector<std::vector<double>> end_points;
  std::vector<double> end_point1{1.1, 0.5491663106062279, 1, -1.5367506072642376, -1.9161961994887118, -0.5040556377982144};
//  end_points.push_back(end_point1);
  std::vector<double> end_point2(dof,-1.3);
//  end_points.push_back(end_point2);
  std::vector<double> end_point3{2.9822665515445386, 1.9739849851131333, 1.623626013297375, -0.45623103496656403, -2.9821901802623207, 3.1415176553450577};
  end_points.push_back(end_point3);


//  ros::Time t1g=ros::Time::now();
//  net.generateNodesFromGrid(number_of_nodes,lower_bound,upper_bound);
//  ros::Time t2g=ros::Time::now();
//  ROS_FATAL("creating net in %f second",(t2g-t1g).toSec());

  ros::Time t1r=ros::Time::now();
  net.generateNodesFromStartAndEndPoints(start_point,end_points);
  ros::Time t2r=ros::Time::now();
  ROS_FATAL("creating net in %f second",(t2r-t1r).toSec());

  ros::Time t1rrt=ros::Time::now();
  net.rrtConnect();
  ros::Time t2rrt=ros::Time::now();
  ROS_FATAL("RRTConnect in %f second",(t2rrt-t1rrt).toSec());

  ha_planner::Path simple=net.simplifyIntermidiatePoint(net.getBestPathRef());
  ROS_FATAL("cost %f, best=%f",net.computePathCost(simple),net.computePathCost(net.getBestPathRef()));
  net.printPath(simple);

  net.printBestPath();


  unsigned int trials=1000;
  unsigned int number_of_nodes_with_solution=500;
  unsigned int n_ants_with_solution=5000;
  unsigned int max_stall_gen_with_solution=10;

  unsigned int number_of_nodes_without_solution=5000;
  unsigned int n_ants_without_solution=1000;
  unsigned int max_stall_gen_without_solution=100;

  net.printBestPath();

//  net.print();


  if (!net.isSolutionFound())
  {
    ROS_FATAL("not found to every goal");
    net.generateNodesFromEllipsoid(number_of_nodes_without_solution-net.getNodeNumber());
  }
  else
  {
    ROS_FATAL("found to every goal");
    net.generateNodesFromEllipsoid(number_of_nodes_with_solution-net.getNodeNumber());
  }
//  net.print();


  net.updateNodeHeuristic();



  unsigned int stall_gen=0;


  ros::Time t0alg=ros::Time::now();
  for (unsigned int itrial=0;itrial<trials;itrial++)
  {
    unsigned int number_of_nodes;
    unsigned int n_ants;
    unsigned int max_stall_gen;
    if (!net.isSolutionFound())
    {
      number_of_nodes=number_of_nodes_without_solution;
      n_ants=n_ants_without_solution;
      max_stall_gen=max_stall_gen_without_solution;
    }
    else
    {
      number_of_nodes=number_of_nodes_with_solution;
      n_ants=n_ants_with_solution;
      max_stall_gen=max_stall_gen_with_solution;
    }

    if (!net.runAntCycle(n_ants))
      stall_gen++;
    else
      stall_gen=0;

    net.evaporatePheromone();
    net.distributePheromone(1);

    if (1) //(itrial%10==0)
    {
      ros::Time t0remc=ros::Time::now();
      unsigned int removed_conn=net.removeLowPheromoneConnections(net.getNodeNumber()*0.5);
      ros::Time t1remc=ros::Time::now();
      ROS_FATAL("removed %u low pheromone connections (in %f ms)",removed_conn,1e3*(t1remc-t0remc).toSec());

      ros::Time t0rem=ros::Time::now();
      unsigned int removed_nodes=net.removeUnconnectedNodes();
      ros::Time t1rem=ros::Time::now();
      ROS_FATAL("removed %u nodes (in %f ms) because they were unconnected",removed_nodes,1e3*(t1rem-t0rem).toSec());

      unsigned int add_node_number=0;
      if (net.getNodeNumber()<number_of_nodes)
        add_node_number=number_of_nodes-net.getNodeNumber();
      ros::Time t0add=ros::Time::now();
      net.generateNodesFromEllipsoid(add_node_number);
      net.updateNodeHeuristic();
      ros::Time t1add=ros::Time::now();
      ROS_FATAL("add %u nodes (in %f ms), net now has %u nodes",add_node_number,1e3*(t1add-t0add).toSec(),net.getNodeNumber());
      ROS_FATAL("stall generation = %u",stall_gen);

//      net.print();
//      net.printBestPath();
    }

    if (stall_gen>=max_stall_gen)
      break;
  }
  ros::Time t1alg=ros::Time::now();
  ROS_INFO("compute in %f ms",1e3*(t1alg-t0alg).toSec());

  net.printBestPath();

  std::vector<std::vector<double>> waypoints=net.getBestPath();

  moveit_msgs::ExecuteTrajectoryGoal goal;
  goal.trajectory.joint_trajectory.joint_names=group->getActiveJointModelNames();

  ros::Duration time_from_start=ros::Duration(0);
  for (const std::vector<double>& waypoint: waypoints )
  {
    trajectory_msgs::JointTrajectoryPoint point;
//    point.positions.resize(waypoint.size());
//    std::copy(waypoint.begin(),waypoint.end(),point.positions.begin());
    point.positions=waypoint;
    point.velocities.resize(waypoint.size(),0.0);
    point.accelerations.resize(waypoint.size(),0.0);
    point.time_from_start=time_from_start;
    time_from_start+=ros::Duration(3);
    goal.trajectory.joint_trajectory.points.push_back(point);

  }

  move_group.setJointValueTarget(start_point);
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success;
  success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  if (!success)
  {
    ROS_ERROR("Planning failed");
    return 0;
  }
  move_group.move();

  ROS_FATAL("qui");
  if (!ac.waitForServer(ros::Duration(5)))
    ROS_ERROR("no server found");
  else
    ac.sendGoalAndWait(goal);
}
