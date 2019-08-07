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

  double t=0;
  double st=1./12.5;
  ros::Rate lp(1./st);


  std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > points;

  for (unsigned int idx=0;idx<100;idx++)
  {
    Eigen::Vector3d p;
    p(0)=0.01*idx;
    p(1)=0.0*idx;
    p(2)=1.5;
    points.push_back(p);

  }
  while (ros::ok())
  {
    if (meas_sub.isANewDataAvailable())
    {
      grid.update(meas_sub.getData());
    }
//    geometry_msgs::PoseArray measures;
//    geometry_msgs::Pose p;
//    p.position.x=0.5*std::cos(2*M_PI*t/5.);
//    p.position.y=0.5*std::sin(2*M_PI*t/5.);
//    p.position.z=0;

//    measures.poses.push_back(p);

    ROS_INFO("occupancy=%f",grid.totalOccupancy(points));
    pc_pub.publish(grid.toPointCloud());

    t+=st;
    lp.sleep();
  }
  return 0;
}
