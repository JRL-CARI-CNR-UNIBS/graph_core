#include <human_probablistic_occupancy/human_probablistic_occupancy.h>
#include <subscription_notifier/subscription_notifier.h>
#include <rosdyn_core/primitives.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "human_occupancy");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(4);
  spinner.start();

  ros::Publisher pc_pub=nh.advertise<sensor_msgs::PointCloud>("occupancy",1);
  ros_helper::SubscriptionNotifier<geometry_msgs::PoseArray> meas_sub(nh,"/poses",1);

  Eigen::Vector3d x_min;
  Eigen::Vector3d x_max;
  unsigned int npnt=50;
  x_min.setConstant(-3);
  x_max.setConstant(3);
  human_occupancy::OccupancyGrid grid(x_min,x_max,npnt);
  grid.getFromParam(nh);
  double t=0;
  double st=1./12.5;
  ros::Rate lp(1./st);


  while (ros::ok())
  {
//    if (meas_sub.isANewDataAvailable())
//    {
//      grid.update(meas_sub.getData());
//    }
    pc_pub.publish(grid.toPointCloud());

    t+=st;
    lp.sleep();
//    if (t>5)
//      break;
  }

//  grid.toYaml(nh);
  return 0;
}
