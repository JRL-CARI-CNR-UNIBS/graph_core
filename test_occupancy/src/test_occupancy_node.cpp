#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char **argv)
{
  // init ros
  ros::init(argc, argv, "test_of_dgaco");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(4);
  spinner.start();

  std::string group_name="ur10";
  moveit::planning_interface::MoveGroupInterface group(group_name);


  group.setStartState(*group.getCurrentState());

  group.setPlanningTime(10.0);


  std::vector<double> joint_conf1(6,0); // [0,0,0,0,0,0]
  joint_conf1.at(0)=-0.5*M_PI;
  joint_conf1.at(1)=-M_PI*0.5;
  joint_conf1.at(2)=M_PI*0.5;

  std::vector<double> joint_conf2(6,0); // [0,0,0,0,0,0]
  joint_conf2.at(0)=M_PI;
  joint_conf2.at(1)=-M_PI*0.5;
  joint_conf2.at(2)=M_PI*0.5;


  while(ros::ok())
  {
    group.setStartState(*group.getCurrentState());
    group.setJointValueTarget(joint_conf1);
    moveit::planning_interface::MoveGroupInterface::Plan plan1;

    bool success = (group.plan(plan1) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (!success)
    {
      ROS_ERROR("Planning failed computing waypoint 1");
      return 0;
    }
    group.execute(plan1);


    group.setStartState(*group.getCurrentState());
    group.setJointValueTarget(joint_conf2);
    moveit::planning_interface::MoveGroupInterface::Plan plan2;

    success = (group.plan(plan2) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (!success)
    {
      ROS_ERROR("Planning failed computing waypoint 2");
      return 0;
    }
    group.execute(plan2);
  }


  return 0;
}
