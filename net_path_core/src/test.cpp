#include <ros/ros.h>
#include <net_path_core/net_path_core.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit_msgs/ExecuteTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/trajectory_processing/iterative_spline_parameterization.h>
int main(int argc, char **argv)
{
  ros::init(argc, argv, "cartesian_trajectory_client_from_yaml");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(4);
  spinner.start();
  ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);

  bool only_plan=true;


  //  rrt cost=16.557902
  //  bif=10.995101
  //  local=10.995101
  //  aco cost=5.918889
  //  [FATAL, main,234]: RRTConnect in 0.208236 second
  //  [FATAL, main,235]: Warping and splitting in 0.573436 second
  //  [FATAL, main,236]: biforcation in 2.712688 second (11 iteration)
  //  [ INFO, main,237]: ant in 399703.990169 ms (best solution in 448)
  //  [ INFO, main,269]: iptp trj duration 8.914172
  //  [ INFO, main,276]: isp trj duration 10.005595

  std::string group_name="vs060a3";

  moveit::planning_interface::MoveGroupInterface move_group(group_name);


  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr       kinematic_model = robot_model_loader.getModel();
  planning_scene::PlanningScenePtr planning_scene=std::make_shared<planning_scene::PlanningScene>(kinematic_model);
  robot_state::RobotState state = planning_scene->getCurrentState();

  moveit::core::JointModelGroup* group=kinematic_model->getJointModelGroup(group_name);

  unsigned int dof=group->getActiveJointModelNames().size();
  //  moveit::core::JointBoundsVector limits=group->getActiveJointModelsBounds();
  //  moveit::core::JointModel::Bounds bounds;
  //  bounds.at(0).acceleration_bounded_;


  std::vector<double> max_velocity(6);
  max_velocity.at(0)=0.40317105721069;
  max_velocity.at(1)=0.392175482923126;
  max_velocity.at(2)=0.725707902979242;
  max_velocity.at(3)=0.739146938219599;
  max_velocity.at(4)=0.739146938219599;
  max_velocity.at(5)=1.1093312725676;

  std::vector<double> scaling(6);
  for (unsigned int idx=0;idx<scaling.size();idx++)
    scaling.at(idx)=1.0/max_velocity.at(idx);
  ha_planner::Net net(dof,group_name,planning_scene,scaling);


  std::vector<double> start_point{-0.4883333241398625, 1.345835745411008, 1.6106694148309002, -1.7592098138836945e-05,  0.18509360138518802, -0.48831603322903483};
  std::vector<std::vector<double>> end_points;

  std::vector<double> end_point0{0.5481165799694595, 1.0414934819491388, 0.777502420568278, 5.680665896769463e-05,1.3225630716092494, 0.5481026754574536};
  std::vector<double> end_point1{-0.48833332341687274, 1.3458357251491204, 1.6106694320322434, -3.1416126113214666,  -0.1850932260114333, 2.6532789862227277};
  std::vector<double> end_point2{-0.4883333241398625, 1.345835745411008, 1.6106694148309002, -1.7592098138836945e-05,  0.18509360138518802, -0.48831603322903483};
  std::vector<double> end_point3{-0.4883333241307036, 1.3458357460587942, 1.610669414326582, -1.762597371424766e-05,   0.1850936018901623, 5.794869307832635};
  std::vector<double> end_point4{-0.48833332558079007, 1.3458355912122064, 1.6106695348037567, 3.141580330146525,   -0.18509357070276614, 2.6532713506808197};
  std::vector<double> end_point5{-0.4883333241161222, 1.3458357446041884, 1.610669415455745, -3.141610337022636,  -0.18509360178504436, -3.629908595466747};


//  end_points.push_back(end_point0);
//    end_points.push_back(end_point1);
//    end_points.push_back(end_point2);
//    end_points.push_back(end_point3);
//    end_points.push_back(end_point4);
    end_points.push_back(end_point5);


  //  ros::Time t1g=ros::Time::now();
  //  net.generateNodesFromGrid(number_of_nodes,lower_bound,upper_bound);
  //  ros::Time t2g=ros::Time::now();
  //  ROS_FATAL("creating net in %f second",(t2g-t1g).toSec());

  ros::Time t1r=ros::Time::now();
  net.generateNodesFromStartAndEndPoints(start_point,end_points);
  ros::Time t2r=ros::Time::now();

  if (net.isSolutionFound())
  {
    ROS_DEBUG("direct path");
    return 0;
  }
  ROS_FATAL("creating net in %f second",(t2r-t1r).toSec());
  ros::Time t1rrt=ros::Time::now();
  if (!net.runRRTConnect())
    return 0;
  ros::Time t2rrt=ros::Time::now();

  ros::Time t1,t2;
  double cost;
  double rrt_cost=net.getBestCost();

  ros::Time t1w=ros::Time::now();
  net.warpPath2(20);
  net.splitPath2(20);
  ros::Time t2w=ros::Time::now();





  t1=ros::Time::now();
  double last_cost=net.getBestCost();

  unsigned int stall_gen=0;
  for (unsigned int idx=0;idx<20000;idx++)
  {
    if (!ros::ok())
      return 0;
    net.runRRTConnect();

    t2=ros::Time::now();
    cost=net.getBestCost();
    if (cost<last_cost*0.999)
    {
      last_cost=cost;
      stall_gen=0;
      ROS_INFO("improvement. second=%f, cost=%f",(t2-t1).toSec(),cost);
    }
    else
    {
      stall_gen++;
      ROS_WARN("stall. second=%f, cost=%f",(t2-t1).toSec(),cost);
    }
    if (stall_gen>3)
      break;
  }

  unsigned int trials=2000;//10000;
  unsigned int number_of_nodes=200;
  unsigned int n_ants=50;
  unsigned int max_stall_gen=50;



  double local_best_cost=net.getBestCost();


//  t1=ros::Time::now();
//  net.generateNodesFromGrid(7);
//  t2=ros::Time::now();
//  cost=net.getBestCost();
//  ROS_FATAL("generateNodesFromGrid in %f second, cost=%f",(t2-t1).toSec(),cost);

  unsigned int removed_node=0;
  unsigned int rem;
  do
  {
    rem=net.removeUnconnectedNodes();
    removed_node+=rem;
  }
  while (rem>0);
  ROS_FATAL("removed %u nodes",removed_node);


  net.updateNodeHeuristic();


  ros::Time t0alg=ros::Time::now();
  unsigned int itrial=0;
  stall_gen=0;
  last_cost=net.getBestCost();
  for (itrial=0;itrial<trials;itrial++)
  {


    if (net.runAntCycle(n_ants))
    {
      net.warpPath2(20);
      net.splitPath2(20);
    }

    net.evaporatePheromone();
    net.distributePheromone(1);

    removed_node=0;
    do
    {
      rem=net.removeUnconnectedNodes();
      removed_node+=rem;
    }
    while (rem>0);


    if (itrial%5==0)
    {

      net.removeLowPheromoneConnections(net.getNodeNumber()*1);



      unsigned int add_node_number=0;
      if (net.getNodeNumber()<number_of_nodes)
        add_node_number=number_of_nodes-net.getNodeNumber();

      unsigned int tmp=net.getNodeNumber();
      net.generateNodesFromEllipsoid(add_node_number);

      net.updateNodeHeuristic();
    }

    if (net.getBestCost()<(last_cost*0.9999))
    {
      stall_gen=0;
      last_cost=net.getBestCost();
    }
    else
    {
      stall_gen++;
    }

    ROS_DEBUG("cycle=%u,cost=%f,stall=%u",itrial,last_cost,stall_gen);
    if (stall_gen>=max_stall_gen)
      break;
  }
  ros::Time t1alg=ros::Time::now();


  double aco_cost=net.getBestCost();
  ROS_FATAL("\nrrt cost=%f\nlocal=%f\naco cost=%f",rrt_cost,local_best_cost,aco_cost);
  ROS_FATAL("RRTConnect in %f second",(t2rrt-t1rrt).toSec());
  ROS_FATAL("Warping and splitting in %f second",(t2w-t1w).toSec());
  ROS_INFO("ant in %f ms (best solution in %u)",1e3*(t1alg-t0alg).toSec(),itrial-stall_gen);
  trajectory_processing::IterativeParabolicTimeParameterization iptp;
  trajectory_processing::IterativeSplineParameterization isp;


  std::vector<std::vector<double>> waypoints=net.getBestPath();

  trajectory_msgs::JointTrajectory trj;
  trj.joint_names=group->getActiveJointModelNames();

  ros::Duration time_from_start=ros::Duration(0);
  for (const std::vector<double>& waypoint: waypoints )
  {
    trajectory_msgs::JointTrajectoryPoint point;
    point.positions=waypoint;
    point.velocities.resize(waypoint.size(),0.0);
    point.accelerations.resize(waypoint.size(),0.0);
    point.time_from_start=time_from_start;
    time_from_start+=ros::Duration(0);
    trj.points.push_back(point);
  }

  robot_trajectory::RobotTrajectory trajectory(move_group.getRobotModel(), group_name);
  moveit::core::RobotState trj_state = *move_group.getCurrentState();
  trj_state.setJointGroupPositions(group_name, trj.points.front().positions);

  trajectory.setRobotTrajectoryMsg(trj_state, trj);
  iptp.computeTimeStamps(trajectory);

  moveit_msgs::ExecuteTrajectoryGoal goal;
  trajectory.getRobotTrajectoryMsg(goal.trajectory);

  ROS_INFO("iptp trj duration %f",goal.trajectory.joint_trajectory.points.back().time_from_start.toSec());

  trajectory.setRobotTrajectoryMsg(trj_state, trj);
  isp.computeTimeStamps(trajectory);

  trajectory.getRobotTrajectoryMsg(goal.trajectory);

  ROS_INFO("isp trj duration %f",goal.trajectory.joint_trajectory.points.back().time_from_start.toSec());

  if (only_plan)
    return 0;
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
  actionlib::SimpleActionClient<moveit_msgs::ExecuteTrajectoryAction> ac(nh, "/execute_trajectory");

  ROS_FATAL("qui");
  if (!ac.waitForServer(ros::Duration(5)))
    ROS_ERROR("no server found");
  else
    ac.sendGoalAndWait(goal);
}
