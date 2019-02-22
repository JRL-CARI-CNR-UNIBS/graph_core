#include <ros/ros.h>
#include <net_path_core/net_path_core.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "cartesian_trajectory_client_from_yaml");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(4);
  spinner.start();



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

  unsigned int number_of_nodes=500;//std::min(std::pow(5,dof),500.);
  std::vector<double> lower_bound(dof,-4);
  std::vector<double> upper_bound(dof, 4);

  std::vector<double> start_point(dof,0);
  std::vector<double> end_point1{1.446726259018521, 1.3864028649496873, 1.6440141802225232, -0.44536844766924083, 1.536547493668399, -1.8536919672547214};
  std::vector<std::vector<double>> end_points;
  end_points.push_back(end_point1);
  //  std::vector<double> end_point2(dof,-1);
  //  end_points.push_back(end_point2);



  ros::Time t1g=ros::Time::now();
  net.generateNodesFromGrid(number_of_nodes,lower_bound,upper_bound);
  ros::Time t2g=ros::Time::now();
  ROS_FATAL("creating net in %f second",(t2g-t1g).toSec());

  ros::Time t1r=ros::Time::now();
  net.generateNodesFromStartAndEndPoints(start_point,end_points);
  ros::Time t2r=ros::Time::now();
  ROS_FATAL("creating net in %f second",(t2r-t1r).toSec());
  net.updateNodeHeuristic();

  unsigned int trials=1000;
  unsigned int n_ants=100;
//  double best_cost=std::numeric_limits<double>::infinity();

  unsigned int stall_gen=0;
  unsigned int max_stall_gen=30;
  for (unsigned int itrial=0;itrial<trials;itrial++)
  {
    if (!net.runAntCycle(n_ants))
      stall_gen++;
    else
      stall_gen=0;

    net.evaporatePheromone();
    net.distributePheromone(1);

    ROS_FATAL("stall generation = %u",stall_gen);
    if (stall_gen>=max_stall_gen)
      break;
  }

  ROS_INFO("best path");
  net.printBestPath();
}
