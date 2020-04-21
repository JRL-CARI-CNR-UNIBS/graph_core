#include <human_probablistic_occupancy/human_probablistic_occupancy.h>
#include <subscription_notifier/subscription_notifier.h>
#include <rosdyn_core/primitives.h>
#include <sensor_msgs/JointState.h>
 #include <name_sorting/name_sorting.h>
int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_point_from_robot");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(4);
  spinner.start();

  urdf::Model model;
  model.initParam("robot_description");
  std::string base_frame = "world";
  std::string tool_frame = "tip";

  Eigen::Vector3d grav;
  grav << 0, 0, -9.806;

  rosdyn::ChainPtr chain=rosdyn::createChain(model, base_frame,tool_frame,grav);
  ros_helper::SubscriptionNotifier<sensor_msgs::JointState> js_sub(nh,"/joint_states",1);
  ros::Publisher pc_pub=nh.advertise<sensor_msgs::PointCloud>("occupancy",1);
  ros::Publisher tp_pub=nh.advertise<geometry_msgs::PoseArray>("test_point",1);

  human_occupancy::OccupancyGridPtr grid=std::make_shared<human_occupancy::OccupancyGrid>(nh);
  human_occupancy::OccupancyFilter filt(chain,grid,nh);


  double t=0;
  double st=1./50.0;
  ros::Rate lp(1./st);

  std::vector<double> position;
  std::vector<std::string> names;

  while (ros::ok())
  {

    if (js_sub.isANewDataAvailable())
    {
      position = js_sub.getData().position;
      names = js_sub.getData().name;
      if (!name_sorting::permutationName(chain->getMoveableJointNames(),names,position))
      {
        ROS_WARN("missing joints in joint_states topic");
        return -1;
      }
      Eigen::Map<Eigen::VectorXd> q(position.data(), chain->getActiveJointsNumber());
      double occ;
      ros::Time tfilt1=ros::Time::now();
      occ=filt.occupancy(q);
      ros::Time tfilt2=ros::Time::now();
      ros::Time tfilt3=ros::Time::now();
      occ=filt.occupancy(q);
      ros::Time tfilt4=ros::Time::now();
      ros::Time taa1=ros::Time::now();
      ros::Time taa2=ros::Time::now();
      ROS_INFO("occupancy   computed in %05.3f us (nope is %05.3f us, without fk %05.3f us, onlyfk %5.3f), occupancy=%f",(tfilt2-tfilt1).toSec()*1e6,(taa2-taa1).toSec()*1e6,(tfilt4-tfilt3).toSec()*1e6,((tfilt2-tfilt1)-(tfilt4-tfilt3)).toSec()*1e6,occ);
      pc_pub.publish(grid->toPointCloud());
      tp_pub.publish(filt.getTestPoints(q));

    }
    t+=st;
    lp.sleep();
  }
  return 0;
}
