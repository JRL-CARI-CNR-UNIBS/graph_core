/*
Copyright (c) 2019, Manuel Beschi CNR-STIIMA manuel.beschi@stiima.cnr.it
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the <organization> nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/


#include <dirrt_star/probabilist_hamp_time_planner.h>
#include <sensor_msgs/PointCloud.h>

namespace pathplan {
namespace dirrt_star {

ProbabilisticHAMPTimeBasedMultiGoalPlanner::ProbabilisticHAMPTimeBasedMultiGoalPlanner ( const std::string& name,
                                                               const std::string& group,
                                                               const moveit::core::RobotModelConstPtr& model ) :
  HAMPTimeBasedMultiGoalPlanner ( name, group, model )
{
  probabilistic_avoidance_metrics_=std::make_shared<pathplan::ProbabilistcAvoidanceTimeMetrics>(max_velocity_,nu_,nh_);
  avoidance_metrics_=probabilistic_avoidance_metrics_;
}

void ProbabilisticHAMPTimeBasedMultiGoalPlanner::subscribeTopic()
{
  std::string poses_topic;
  if (!nh_.getParam("occupancy_topic",poses_topic))
  {
    ROS_DEBUG("occupancy_topic is not defined, using /occupancy");
    poses_topic="/occupancy";
  }
  ROS_FATAL("subscribe topic %s",poses_topic.c_str());

  poses_sub=nh_.subscribe<sensor_msgs::PointCloud>(poses_topic,2,&ProbabilisticHAMPTimeBasedMultiGoalPlanner::occupancyCallback,this);

}

void ProbabilisticHAMPTimeBasedMultiGoalPlanner::occupancyCallback(const sensor_msgs::PointCloudConstPtr& msg)
{
  Eigen::Vector3d point;
  avoidance_metrics_->cleanPoints();
  if (msg->header.frame_id != camera_frame_)
  {
    tf::StampedTransform transform;
    try
    {
      listener_.lookupTransform(world_frame_,msg->header.frame_id,
                               ros::Time(0), transform);
    }
    catch (tf::TransformException &ex)
    {
      ROS_WARN_THROTTLE(1.0,"unable to get the transform from %s to %s because: %s",world_frame_.c_str(),msg->header.frame_id.c_str(),ex.what());
      return;
    }
    tf::transformTFToEigen(transform,T_word_camera);
    camera_frame_=msg->header.frame_id;
  }
  size_t npoints=msg->points.size();

  int ichannel=-1;

  if (msg->channels.size()==0)
  {
    ROS_ERROR_THROTTLE(1,"%s: there are no channels in the point cloud",name_.c_str());
    return;
  }
  for (size_t ich=0;ich<msg->channels.size();ich++)
  {
    if (msg->channels.at(ich).name=="occupancy")
    {
      ichannel=ich;
      break;
    }
  }
  if (ichannel<0)
  {
    ROS_ERROR_THROTTLE(1,"%s: there are no channels in the point cloud named 'occupancy'",name_.c_str());
    return;
  }

  ROS_INFO_THROTTLE(1,"add %zu points", npoints);
  for (size_t idx=0;idx<npoints;idx++)
  {
    point(0)=msg->points.at(idx).x;
    point(1)=msg->points.at(idx).y;
    point(2)=msg->points.at(idx).z;
    double occupancy=msg->channels.at(0).values.at(idx);
    probabilistic_avoidance_metrics_->addPointOccupancy(point,occupancy);
  }
}




}  // namespace dirrt_star
}  // namespace pathplan
