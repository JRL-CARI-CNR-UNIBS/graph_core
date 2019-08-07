#ifndef human_probablistic_occupancy_2019
#define human_probablistic_occupancy_2019

#include <Eigen/Core>
#include <ros/ros.h>
#include <unsupported/Eigen/CXX11/Tensor>
#include <geometry_msgs/PoseArray.h>
#include <sensor_msgs/PointCloud.h>
namespace human_occupancy
{
  class OccupancyGrid
  {
  protected:
    Eigen::Tensor<double,3> m_occupancy;
    Eigen::Vector3d m_x_min;
    Eigen::Vector3d m_resolution;

    unsigned int m_npnt;
    double m_frames;
    double m_max_frames_before_iir_filtering;

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    OccupancyGrid(const Eigen::Vector3d& x_min, const Eigen::Vector3d& x_max, const unsigned int& npnt);
    void update(const geometry_msgs::PoseArray& measure);
    double totalOccupancy(const std::vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d>>& points);
    sensor_msgs::PointCloud toPointCloud(ros::Time t=ros::Time::now());
  };
}

#endif
