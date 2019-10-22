#ifndef human_probablistic_occupancy_2019
#define human_probablistic_occupancy_2019

#include <Eigen/Core>
#include <ros/ros.h>
#include <unsupported/Eigen/CXX11/Tensor>
#include <geometry_msgs/PoseArray.h>
#include <sensor_msgs/PointCloud.h>
#include <rosdyn_core/primitives.h>
namespace human_occupancy
{

  bool yamlParser(const XmlRpc::XmlRpcValue& config, std::map<std::string, std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > >& points);

  class OccupancyGrid
  {
  protected:
    Eigen::Tensor<double,3> m_occupancy;
    Eigen::Vector3d m_x_min;
    Eigen::Vector3d m_resolution;
    Eigen::Vector3d m_inv_resolution;
    double m_voxel_volume;
    unsigned int m_npnt;
    double m_frames;
    double m_max_frames_before_iir_filtering;
    std::string m_reference_frame;

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    OccupancyGrid(const Eigen::Vector3d& x_min, const Eigen::Vector3d& x_max, const unsigned int& npnt);
    OccupancyGrid(ros::NodeHandle& nh);
    void update(const geometry_msgs::PoseArray& measure);
    bool fromPointCloud(const sensor_msgs::PointCloud& pc);
    double totalOccupancy(const std::vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d>>& points);
    sensor_msgs::PointCloud toPointCloud(ros::Time t=ros::Time::now());
    void toYaml(ros::NodeHandle& nh);
    bool getFromParam(ros::NodeHandle& nh);
  };

  typedef std::shared_ptr<human_occupancy::OccupancyGrid> OccupancyGridPtr;

  class OccupancyFilter
  {
  protected:
    rosdyn::ChainPtr m_chain;
    human_occupancy::OccupancyGridPtr m_grid;
    std::map<std::string,std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> >> m_test_points;
    ros::NodeHandle m_nh;
    std::vector<std::string> m_link_names;
    std::vector<Eigen::Affine3d,Eigen::aligned_allocator<Eigen::Affine3d>> m_transformations;

    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > m_points;

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    OccupancyFilter(const rosdyn::ChainPtr& chain, const OccupancyGridPtr& grid, const ros::NodeHandle& nh);
    double occupancy(const Eigen::VectorXd& q);
    geometry_msgs::PoseArray getTestPoints(const Eigen::VectorXd& q);
  };


  typedef std::shared_ptr<human_occupancy::OccupancyFilter> OccupancyFilterPtr;
}

#endif
