#include <human_probablistic_occupancy/human_probablistic_occupancy.h>
#include <rosparam_utilities/rosparam_utilities.h>
#include <eigen_conversions/eigen_msg.h>

namespace human_occupancy {


bool yamlParser(const XmlRpc::XmlRpcValue &config,std::map<std::string,std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> >>& points)
{
  ;
  if (config.getType() != XmlRpc::XmlRpcValue::TypeArray)
  {
    ROS_ERROR("The param is not a list of points" );
    return false;
  }
  for(size_t i=0; i < config.size(); i++)
  {
    XmlRpc::XmlRpcValue element = config[i];
    if( element.getType() != XmlRpc::XmlRpcValue::TypeStruct)
    {
      ROS_WARN("The element #%zu is not a struct", i);
      continue;
    }
    if( !element.hasMember("link_name") )
    {
      ROS_WARN("The element #%zu has not the field 'link_name'", i);
      continue;
    }
    std::vector<std::vector<double>> array;
    if( !rosparam_utilities::getParamMatrix(element,"points",array) )
    {
      ROS_WARN("The element #%zu has not the field 'points'", i);
      continue;
    }
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > p(array.size());
    for (unsigned int idx=0;idx<array.size();idx++)
    {
      if (array.at(idx).size()!=3)
      {
        ROS_ERROR("points have to be 3-dimensional");
        return false;
      }
      p.at(idx)(0)=array.at(idx).at(0);
      p.at(idx)(1)=array.at(idx).at(1);
      p.at(idx)(2)=array.at(idx).at(2);

    }
    ROS_INFO("add %zu points to link %s",p.size(),((std::string)element["link_name"]).c_str());
    points.insert(std::pair<std::string,std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> >>((std::string)element["link_name"],p));

  }

  return true;
}

OccupancyGrid::OccupancyGrid(const Eigen::Vector3d& x_min, const Eigen::Vector3d& x_max, const unsigned int& npnt)
{
  assert(npnt>0);
  m_x_min=x_min;
  m_resolution=(x_max-x_min)/(npnt);
  m_voxel_volume=m_resolution(0)*m_resolution(1)*m_resolution(2);
  m_inv_resolution=m_resolution.cwiseInverse();
  m_occupancy.resize(npnt,npnt,npnt);
  m_occupancy.setZero();
  m_npnt=npnt;
  m_frames=0;
  m_max_frames_before_iir_filtering=12.5*5;
}

OccupancyGrid::OccupancyGrid(ros::NodeHandle &nh)
{
  if (!getFromParam(nh))
  {
    ROS_ERROR("unable to load Occupancy Grid");
    throw std::invalid_argument("unable to load Occupancy Grid");
  }
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
  double ix;
  double iy;
  double iz;


  for (const Eigen::Vector3d& p: points)
  {
    ix=(p(0)-m_x_min(0))*m_inv_resolution(0);
    if ( (ix<0) || (ix>=m_npnt))
      continue;

    iy=(p(1)-m_x_min(1))*m_inv_resolution(1);
    if ( (iy<0) || (iy>=m_npnt))
      continue;

    iz=(p(2)-m_x_min(2))*m_inv_resolution(2);
    if ( (iz<0) || (iz>=m_npnt))
      continue;

    double occupancy=            m_occupancy(std::floor(ix),std::floor(iy),std::floor(iz));
    occupancy=std::max(occupancy,m_occupancy(std::floor(ix),std::floor(iy),std::ceil(iz)));
    occupancy=std::max(occupancy,m_occupancy(std::floor(ix),std::ceil(iy),std::floor(iz)));
    occupancy=std::max(occupancy,m_occupancy(std::floor(ix),std::ceil(iy),std::ceil(iz)));
    occupancy=std::max(occupancy,m_occupancy(std::ceil(ix),std::floor(iy),std::floor(iz)));
    occupancy=std::max(occupancy,m_occupancy(std::ceil(ix),std::floor(iy),std::ceil(iz)));
    occupancy=std::max(occupancy,m_occupancy(std::ceil(ix),std::ceil(iy),std::floor(iz)));
    occupancy=std::max(occupancy,m_occupancy(std::ceil(ix),std::ceil(iy),std::ceil(iz)));
    total_occupancy+=occupancy;
  }
  total_occupancy*=m_voxel_volume;
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

void OccupancyGrid::toYaml(ros::NodeHandle& nh)
{
  ros::NodeHandle pnh(nh.getNamespace()+"/occupancy");
  pnh.setParam("time_stamp",ros::Time::now().toSec());
  pnh.setParam("reference_frame",m_reference_frame);

  std::vector<double> corner(m_x_min.size());
  Eigen::VectorXd::Map(&corner[0],m_x_min.size()) = m_x_min;
  pnh.setParam("corner",corner);

  std::vector<double> resolution(m_resolution.size());
  Eigen::VectorXd::Map(&resolution[0],m_resolution.size()) = m_resolution;
  pnh.setParam("resolution",resolution);


  std::vector<double> occupancy(m_occupancy.size());
  std::copy(m_occupancy.data(),m_occupancy.data()+m_occupancy.size(),occupancy.data());
  pnh.setParam("occupancy",occupancy);

  pnh.setParam("number_of_points",(int)m_npnt);


  pnh.setParam("frames",m_frames);
  pnh.setParam("max_frames_before_iir_filtering",m_max_frames_before_iir_filtering);

  system("rosparam dump occupancy.yaml /occupancy");
  pnh.setParam("reference_frame",m_reference_frame);

}

bool OccupancyGrid::getFromParam(ros::NodeHandle &nh)
{
  ros::NodeHandle pnh(nh.getNamespace()+"/occupancy");
  if (!pnh.getParam("reference_frame",m_reference_frame))
  {
    ROS_ERROR("no %s/reference_frame",pnh.getNamespace().c_str());
    return false;
  }

  std::vector<double> corner(m_x_min.size());
  if (!pnh.getParam("corner",corner))
  {
    ROS_ERROR("no %s/reference_frame",pnh.getNamespace().c_str());
    return false;
  }
  m_x_min=Eigen::VectorXd::Map(&corner[0],corner.size());

  std::vector<double> resolution(m_resolution.size());
  if(!pnh.getParam("resolution",resolution))
  {
    ROS_ERROR("no %s/resolution",pnh.getNamespace().c_str());
    return false;
  }
  m_resolution = Eigen::VectorXd::Map(&resolution[0],resolution.size());
  m_inv_resolution=m_resolution.cwiseInverse();
  m_voxel_volume=m_resolution(0)*m_resolution(1)*m_resolution(2);
  std::vector<double> occupancy(m_occupancy.size());
  if (!pnh.getParam("occupancy",occupancy))
  {
    ROS_ERROR("no %s/occupancy",pnh.getNamespace().c_str());
    return false;
  }

  int npnt;
  if (!pnh.getParam("number_of_points",npnt))
  {
    ROS_ERROR("no %s/occupancy",pnh.getNamespace().c_str());
    return false;
  }
  m_npnt=npnt;

  m_occupancy.resize(npnt,npnt,npnt);
  std::copy(occupancy.data(),occupancy.data()+occupancy.size(),m_occupancy.data());

  if (!pnh.getParam("frames",m_frames))
  {
    ROS_ERROR("no %s/frames",pnh.getNamespace().c_str());
    return false;
  }
  if (!pnh.getParam("max_frames_before_iir_filtering",m_max_frames_before_iir_filtering))
  {
    ROS_ERROR("no %s/max_frames_before_iir_filtering",pnh.getNamespace().c_str());
    return false;
  }
  return true;
}

bool OccupancyGrid::fromPointCloud(const sensor_msgs::PointCloud &pc)
{

  if(!pc.channels.at(0).name.compare("occupancy"))
    return false;

  for (unsigned int idx=0;idx<pc.points.size();idx++)
  {

    double ix=(pc.points.at(idx).x-m_x_min(0))/m_resolution(0);
    double iy=(pc.points.at(idx).y-m_x_min(1))/m_resolution(1);
    double iz=(pc.points.at(idx).z-m_x_min(2))/m_resolution(2);


    if ( (ix<0) || (ix>=m_npnt))
      continue;
    if ( (iy<0) || (iy>=m_npnt))
      continue;
    if ( (iz<0) || (iz>=m_npnt))
      continue;

    m_occupancy(ix,iy,iz)=pc.channels.at(0).values.at(idx);
  }
  return true;
}



OccupancyFilter::OccupancyFilter(const rosdyn::ChainPtr &chain, const OccupancyGridPtr &grid, const ros::NodeHandle& nh):
  m_chain(chain),
  m_grid(grid),
  m_nh(nh)
{
  XmlRpc::XmlRpcValue par;
  nh.getParam("occupancy_robot_points",par);

  if (!human_occupancy::yamlParser(par,m_test_points))
  {
    ROS_ERROR("occupancy_robot_points does not exist or it is uncorrect");
  }
  m_link_names=m_chain->getLinksName();
  unsigned int npnts=0;


  for (unsigned int il=0;il<m_link_names.size();il++)
  {
    if ( m_test_points.find(m_link_names.at(il)) != m_test_points.end() )
    {
      npnts+=m_test_points.at(m_link_names.at(il)).size();
    }
  }
  m_points.resize(npnts);
}

double OccupancyFilter::occupancy(const Eigen::VectorXd &q)
{
  unsigned ipnt=0;
  m_transformations=m_chain->getTransformations(q);
  for (unsigned int il=0;il<m_transformations.size();il++)
  {
    if ( m_test_points.find(m_link_names.at(il)) != m_test_points.end() )
    {
      const Eigen::Affine3d& t= m_transformations.at(il);
      const auto& pnts= m_test_points.at(m_link_names.at(il));
      for (unsigned int ip=0;ip<pnts.size();ip++)
      {
        m_points.at(ipnt++)=t*pnts.at(ip);
      }
    }
  }
  return m_grid->totalOccupancy(m_points);
}

geometry_msgs::PoseArray OccupancyFilter::getTestPoints(const Eigen::VectorXd &q)
{
  unsigned ipnt=0;
  m_transformations=m_chain->getTransformations(q);
  for (unsigned int il=0;il<m_transformations.size();il++)
  {
    if ( m_test_points.find(m_link_names.at(il)) != m_test_points.end() )
    {
      const Eigen::Affine3d& t= m_transformations.at(il);
      const auto& pnts= m_test_points.at(m_link_names.at(il));
      for (unsigned int ip=0;ip<pnts.size();ip++)
      {

        m_points.at(ipnt++)=t*pnts.at(ip);
      }
    }
  }
  geometry_msgs::PoseArray msg;
  msg.header.frame_id="world";
  msg.header.stamp=ros::Time::now();
  for (unsigned int idx=0;idx<m_points.size();idx++)
  {
    geometry_msgs::Pose pose;
    pose.orientation.x=0;
    pose.orientation.y=0;
    pose.orientation.z=0;
    pose.orientation.w=1;
    pose.position.x=m_points.at(idx)(0);
    pose.position.y=m_points.at(idx)(1);
    pose.position.z=m_points.at(idx)(2);
    msg.poses.push_back(pose);
  }
  return msg;
}
}
