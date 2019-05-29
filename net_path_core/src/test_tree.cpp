#include <ros/ros.h>
#include <net_path_core/net_path_core.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit_msgs/ExecuteTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_test");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(4);
  spinner.start();

  ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);

  std::string group_name="vs060a3";
  moveit::planning_interface::MoveGroupInterface move_group(group_name);


  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
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

  std::vector<double> scaling(6,1);
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

  end_points.push_back(end_point0);
//  end_points.push_back(end_point1);
//  end_points.push_back(end_point2);
//  end_points.push_back(end_point3);
//  end_points.push_back(end_point4);
//  end_points.push_back(end_point5);


//  ros::Time t1g=ros::Time::now();
//  net.generateNodesFromGrid(number_of_nodes,lower_bound,upper_bound);
//  ros::Time t2g=ros::Time::now();
//  ROS_FATAL("creating net in %f second",(t2g-t1g).toSec());

  ros::Time t1r=ros::Time::now();
  net.generateNodesFromStartAndEndPoints(start_point,end_points);
  ros::Time t2r=ros::Time::now();
  ROS_FATAL("creating net in %f second",(t2r-t1r).toSec());

  double rrt_cost;

  ros::Time t1rrt=ros::Time::now();
  if (!net.runRRTConnect())
  {
    return 0;
  }
  ros::Time t2rrt=ros::Time::now();
  rrt_cost=net.getBestCost();
  ROS_FATAL("solving RRTconnect in %f second, cost=%f, point=%zu",(t2rrt-t1rrt).toSec(),rrt_cost,net.getBestPathRef().size());

  ros::Time t1,t2;
  double cost;

  t1=ros::Time::now();
  for (unsigned int idx=0;idx<500;idx++)
    net.curvilinearPath();
  t2=ros::Time::now();
  cost=net.getBestCost();
  ROS_FATAL("curvilinearPath in %f second, cost=%f",(t2-t1).toSec(),cost);


  int warp_type=0;
  if (!nh.getParam("warp",warp_type))
    warp_type=0;

  t1=ros::Time::now();
  net.warpPath2(20);
  t2=ros::Time::now();
  cost=net.getBestCost();
  ROS_FATAL("warpPath2 in %f second, cost=%f",(t2-t1).toSec(),cost);

  t1=ros::Time::now();
  net.splitPath2(20);
  t2=ros::Time::now();
  cost=net.getBestCost();
  ROS_FATAL("splitPath2 in %f second, cost=%f",(t2-t1).toSec(),cost);









  t1=ros::Time::now();
  net.generateNodesFromGrid(9);
  t2=ros::Time::now();
  cost=net.getBestCost();
  ROS_FATAL("generateNodesFromGrid in %f second, cost=%f",(t2-t1).toSec(),cost);


  return 0;
  ros::Time t1w=ros::Time::now();

  net.pruningPath(net.getBestPathRef());
  ros::Time t2w=ros::Time::now();
  double ws_cost=net.getBestCost();
  ROS_FATAL("Warping and splitting in %f second,cost=%f",(t2w-t1w).toSec(),ws_cost);
//  net.printBestPath();

  for (unsigned iter=0;iter<30;iter++)
  {
    ros::Time t1rrt=ros::Time::now();
    net.runRRTConnect();
    ros::Time t2rrt=ros::Time::now();
    rrt_cost=net.getBestCost();
    ROS_FATAL("solving RRTconnect in %f second, cost=%f",(t2rrt-t1rrt).toSec(),rrt_cost);
  }


  return 0;
}
