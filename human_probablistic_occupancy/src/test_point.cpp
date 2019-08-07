#include <human_probablistic_occupancy/human_probablistic_occupancy.h>
#include <subscription_notifier/subscription_notifier.h>
#include <rosdyn_core/primitives.h>
#include <robot_self_filter/self_see_filter.h>
int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_point_from_robot");
  ros::NodeHandle nh;
  ros::NodeHandle pnh(~);
  ros::AsyncSpinner spinner(4);
  spinner.start();

  filters::SelfFilter<pcl::PointXYZ> filter(pnh);

  filter.
  return 0;
}
