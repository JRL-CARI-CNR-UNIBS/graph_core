#include <human_probablistic_occupancy/human_probablistic_occupancy.h>

namespace human_occupancy {


OccupancyGrid::OccupancyGrid(const Eigen::Vector3d& x_min, const Eigen::Vector3d& x_max, const unsigned int& npnt)
{
  assert(npnt>0);
  m_x_min=x_min;
  m_resolution=(x_max-x_min)/(npnt);
  m_occupancy.resize(npnt,npnt,npnt);
  m_occupancy.setZero();
  m_npnt=npnt;
  m_frames;
  m_max_frames_before_iir_filtering=12.5*2;
}


void OccupancyGrid::update(const geometry_msgs::PoseArray &measure)
{
  Eigen::Tensor<double,3> occup(m_npnt,m_npnt,m_npnt);
  occup.setZero();

  for (const geometry_msgs::Pose& p: measure.poses)
  {
    double ix=(p.position.x-m_x_min(0))/m_resolution(0);
    double iy=(p.position.y-m_x_min(1))/m_resolution(1);
    double iz=(p.position.z-m_x_min(2))/m_resolution(2);

    if ( (ix<0) || (ix>=m_npnt))
      continue;
    if ( (iy<0) || (iy>=m_npnt))
      continue;
    if ( (iz<0) || (iz>=m_npnt))
      continue;

    occup(std::floor(ix),
         std::floor(iy),
         std::floor(iz))=1;
    occup(std::floor(ix),
         std::floor(iy),
         std::ceil(iz))=1;
    occup(std::floor(ix),
         std::ceil(iy),
         std::floor(iz))=1;
    occup(std::floor(ix),
         std::ceil(iy),
         std::ceil(iz))=1;
    occup(std::ceil(ix),
         std::floor(iy),
         std::floor(iz))=1;
    occup(std::ceil(ix),
         std::floor(iy),
         std::ceil(iz))=1;
    occup(std::ceil(ix),
         std::ceil(iy),
         std::floor(iz))=1;
    occup(std::ceil(ix),
         std::ceil(iy),
         std::ceil(iz))=1;
  }
  m_occupancy=(m_occupancy*m_frames+occup)/(m_frames+1);
  if (m_frames<m_max_frames_before_iir_filtering)
    m_frames++;
}

double OccupancyGrid::totalOccupancy(const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> >& points)
{
  double total_occupancy=0;
  for (const Eigen::Vector3d& p: points)
  {
    double ix=(p(0)-m_x_min(0))/m_resolution(0);
    double iy=(p(1)-m_x_min(1))/m_resolution(1);
    double iz=(p(2)-m_x_min(2))/m_resolution(2);


    if ( (ix<0) || (ix>=m_npnt))
      continue;
    if ( (iy<0) || (iy>=m_npnt))
      continue;
    if ( (iz<0) || (iz>=m_npnt))
      continue;
    double occupancy=0;
    occupancy=std::max(occupancy,m_occupancy(std::floor(ix),std::floor(iy),std::floor(iz)));
    occupancy=std::max(occupancy,m_occupancy(std::floor(ix),std::floor(iy),std::ceil(iz)));
    occupancy=std::max(occupancy,m_occupancy(std::floor(ix),std::ceil(iy),std::floor(iz)));
    occupancy=std::max(occupancy,m_occupancy(std::floor(ix),std::ceil(iy),std::ceil(iz)));
    occupancy=std::max(occupancy,m_occupancy(std::ceil(ix),std::floor(iy),std::floor(iz)));
    occupancy=std::max(occupancy,m_occupancy(std::ceil(ix),std::floor(iy),std::ceil(iz)));
    occupancy=std::max(occupancy,m_occupancy(std::ceil(ix),std::ceil(iy),std::floor(iz)));
    occupancy=std::max(occupancy,m_occupancy(std::ceil(ix),std::ceil(iy),std::ceil(iz)));
    total_occupancy+=occupancy*m_resolution(0)*m_resolution(1)*m_resolution(2);
  }
  return total_occupancy;
}
sensor_msgs::PointCloud OccupancyGrid::toPointCloud(ros::Time t)
{
  sensor_msgs::PointCloud pc;

  pc.header.frame_id="world";
  pc.header.stamp=t;
  pc.channels.resize(1);
  pc.channels.at(0).name="occupancy";
  geometry_msgs::Point32 p;
  for (unsigned int ix=0;ix<m_npnt;ix++)
  {
    for (unsigned int iy=0;iy<m_npnt;iy++)
    {
      for (unsigned int iz=0;iz<m_npnt;iz++)
      {
        if (m_occupancy(ix,iy,iz)>0)
        {
          p.x=m_x_min(0)+m_resolution(0)*ix;
          p.y=m_x_min(1)+m_resolution(1)*iy;
          p.z=m_x_min(2)+m_resolution(2)*iz;
          pc.points.push_back(p);
          pc.channels.at(0).values.push_back(m_occupancy(ix,iy,iz));
        }
      }
    }
  }
  return pc;
}
}
