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


#include <dirrt_star/hamp_time_planner.h>


namespace pathplan {
namespace dirrt_star {

HAMPTimeBasedMultiGoalPlanner::HAMPTimeBasedMultiGoalPlanner ( const std::string& name,
                                                               const std::string& group,
                                                               const moveit::core::RobotModelConstPtr& model ) :
  TimeBasedMultiGoalPlanner ( name, group, model )
{
  avoidance_metrics_=std::make_shared<pathplan::AvoidanceTimeMetrics>(max_velocity_,nu_,nh_);
  metrics_=avoidance_metrics_;
  world_frame_=avoidance_metrics_->getBaseFrame();
  T_word_camera.setIdentity();
  camera_frame_=world_frame_;

  subscribeTopic();
}

void HAMPTimeBasedMultiGoalPlanner::subscribeTopic()
{
  std::string poses_topic;
  if (!nh_.getParam("poses_topic",poses_topic))
  {
    ROS_DEBUG("poses_topic is not defined, using /poses");
    poses_topic="/poses";
  }
  ROS_FATAL("subscribe topic %s",poses_topic.c_str());

  poses_sub=nh_.subscribe<geometry_msgs::PoseArray>(poses_topic,2,&HAMPTimeBasedMultiGoalPlanner::posesCallback,this);

}


void HAMPTimeBasedMultiGoalPlanner::posesCallback(const geometry_msgs::PoseArrayConstPtr& msg)
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
  ROS_INFO_THROTTLE(1,"add %zu points", msg->poses.size());
  for (const geometry_msgs::Pose& p: msg->poses)
  {
    point(0)=p.position.x;
    point(1)=p.position.y;
    point(2)=p.position.z;
    avoidance_metrics_->addPoint(point);
  }
}


}  // namespace dirrt_star
}  // namespace pathplan
