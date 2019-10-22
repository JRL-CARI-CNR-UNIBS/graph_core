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
  /* THIS NODE IS INTENDED FOR TEST AND DEBUG ONLY
   * the common mode to use the DGACO planner is to use the MoveIt!
   * planner package named dgaco_planner.
   */


  // init ros
  ros::init(argc, argv, "test_of_dgaco");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(4);
  spinner.start();

  // print debug information
  ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);

  // enable/disable real movements
  bool only_plan=true;


  // init MoveIt! interface
  std::string group_name="vs060a3";
  moveit::planning_interface::MoveGroupInterface move_group(group_name);
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr       kinematic_model = robot_model_loader.getModel();
  planning_scene::PlanningScenePtr planning_scene=std::make_shared<planning_scene::PlanningScene>(kinematic_model);
  robot_state::RobotState state = planning_scene->getCurrentState();
  moveit::core::JointModelGroup* group=kinematic_model->getJointModelGroup(group_name);

  // get the number of joint
  unsigned int dof=group->getActiveJointModelNames().size();


  // init joint limits
  std::vector<double> max_velocity(6);
  max_velocity.at(0)=0.40317105721069;
  max_velocity.at(1)=0.392175482923126;
  max_velocity.at(2)=0.725707902979242;
  max_velocity.at(3)=0.739146938219599;
  max_velocity.at(4)=0.739146938219599;
  max_velocity.at(5)=1.1093312725676;
  std::vector<double> lb(6,-M_PI);
  std::vector<double> ub(6,M_PI);

  // init dgaco planner
  std::vector<double> scaling(6);
  for (unsigned int idx=0;idx<scaling.size();idx++)
    scaling.at(idx)=1.0/max_velocity.at(idx);
  ha_planner::Net net(dof,group_name,planning_scene,scaling,lb,ub);


  // set start and eng points
  std::vector<double> start_point{0, 0, 0, 0, 0, 0};
  std::vector<std::vector<double>> end_points;
  std::vector<double> end_point0{-0.4883333241387038, 1.345835743383781, 1.6106694164020787, 3.1415750729640006,  -0.1850936022493632, -3.6299086982917856};
  std::vector<double> end_point1{-0.48833332341687274, 1.3458357251491204, 1.6106694320322434, -3.1416126113214666,  -0.1850932260114333, 2.6532789862227277};
  std::vector<double> end_point2{-0.4883333241398625, 1.345835745411008, 1.6106694148309002, -1.7592098138836945e-05,  0.18509360138518802, -0.48831603322903483};
  std::vector<double> end_point3{-0.4883333241307036, 1.3458357460587942, 1.610669414326582, -1.762597371424766e-05,   0.1850936018901623, 5.794869307832635};
  std::vector<double> end_point4{-0.48833332558079007, 1.3458355912122064, 1.6106695348037567, 3.141580330146525,   -0.18509357070276614, 2.6532713506808197};
  std::vector<double> end_point5{-0.4883333241161222, 1.3458357446041884, 1.610669415455745, -3.141610337022636,  -0.18509360178504436, -3.629908595466747};

  // dgaco can manager multiple end points (select the fast one)
  end_points.push_back(end_point0);
  end_points.push_back(end_point1);
  end_points.push_back(end_point2);
  end_points.push_back(end_point3);
  end_points.push_back(end_point4);
  end_points.push_back(end_point5);


  // init netword and try direct connection
  ros::Time t1r=ros::Time::now();
  net.generateNodesFromStartAndEndPoints(start_point,end_points);
  ros::Time t2r=ros::Time::now();

  // if there is a direct connection, stop
  if (net.isSolutionFound())
  {
    ROS_DEBUG("direct path");
    return 0;
  }

  // initializing net with a RRT+ fast local optimizer
  ROS_DEBUG("creating net in %f second",(t2r-t1r).toSec());
  ros::Time t1rrt=ros::Time::now();
  if (!net.runRRTConnect())
    return 0;
  ros::Time t2rrt=ros::Time::now();

  ros::Time t1,t2;
  double cost;
  double rrt_cost=net.getBestCost();

  for (unsigned int idx=0;idx<20;idx++)
  {
    rrt_cost=net.getBestCost();
    ros::Time t0local=ros::Time::now();
    net.localSearch2(2);
    ros::Time t1local=ros::Time::now();
    double local_cost=net.getBestCost();
    ROS_DEBUG("local search in %f second,\nlocal cost=%f->%f ",(t1local-t0local).toSec(),rrt_cost,local_cost);
  }
  return 0;

  // run local optimizer
  ros::Time t1w=ros::Time::now();
  net.warpPath2(20);
//  net.splitPath2(20);
  ros::Time t2w=ros::Time::now();

  // rerun RRT+local optimizer inside the solution ellipsoid
  // see Gammell, Informed RRT*, for the definition of the ellipsoid
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
  double local_best_cost=net.getBestCost();


  // run Ant Colony Optimization (ACO)

  // init params
  unsigned int trials=2000;//10000;
  unsigned int number_of_nodes=200;
  unsigned int n_ants=50;
  unsigned int max_stall_gen=50;

  // remove unconnected nodes
  unsigned int removed_node=0;
  unsigned int rem;
  do
  {
    rem=net.removeUnconnectedNodes();
    removed_node+=rem;
  }
  while (rem>0);
  ROS_DEBUG("removed %u nodes",removed_node);

  //update heuristic = 1/(square distance from end node + 1e-4)
  net.updateNodeHeuristic();


  // run ACO
  ros::Time t0alg=ros::Time::now();
  unsigned int itrial=0;
  stall_gen=0;
  last_cost=net.getBestCost();
  for (itrial=0;itrial<trials;itrial++)
  {

    // send ants
    if (net.runAntCycle(n_ants))
    {
      // if new path is found, run local optimization
      net.warpPath2(20);
      net.splitPath2(20);
      net.localSearch2(2);
    }

    // distribute pheromon on the good node
    net.evaporatePheromone();
    net.distributePheromone(1);

    // remove unconnected nodes
    removed_node=0;
    do
    {
      rem=net.removeUnconnectedNodes();
      removed_node+=rem;
    }
    while (rem>0);


    // every 5 iteration dynamically change the network
    if (itrial%5==0)
    {
      // removed unused connections
      net.removeLowPheromoneConnections(net.getNodeNumber()*1);

      // add random points in the ellipsoid
      unsigned int add_node_number=0;
      if (net.getNodeNumber()<number_of_nodes)
        add_node_number=number_of_nodes-net.getNodeNumber();
      net.generateNodesFromEllipsoid(add_node_number);
      net.updateNodeHeuristic();
    }

    // check if ACO is stalled
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



  // Print results
  ROS_DEBUG("\nrrt cost=%f\nlocal=%f\naco cost=%f",rrt_cost,local_best_cost,aco_cost);
  ROS_DEBUG("RRTConnect in %f second",(t2rrt-t1rrt).toSec());
  ROS_DEBUG("Warping and splitting in %f second",(t2w-t1w).toSec());
  ROS_INFO("ant in %f ms (best solution in %u)",1e3*(t1alg-t0alg).toSec(),itrial-stall_gen);
  trajectory_processing::IterativeParabolicTimeParameterization iptp(1000,0.001);


  // add time parametrization
  std::vector<std::vector<double>> waypoints=net.getBestPath();
  trajectory_msgs::JointTrajectory trj;
  trj.joint_names=group->getActiveJointModelNames();

  ros::Duration time_from_start=ros::Duration(0);

  for (const std::vector<double>& waypoint: waypoints )
  {
    trajectory_msgs::JointTrajectoryPoint point;
    point.positions=waypoint;
    double time=0;
    if (trj.points.size()>0)
    {
      for (int idof=0;idof<6;idof++)
      {
        time=std::max(time,std::abs(waypoint.at(idof)-trj.points.back().positions.at(idof))/max_velocity.at(idof));
      }
    }
    time_from_start+=ros::Duration(time);
    point.time_from_start=time_from_start;
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


  if (only_plan)
    return 0;

  // if only_plan==true, go to starting point
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

  // execute trajectory
  if (!ac.waitForServer(ros::Duration(5)))
    ROS_ERROR("no server found");
  else
    ac.sendGoalAndWait(goal);
}
