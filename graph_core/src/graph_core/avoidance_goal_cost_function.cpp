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

#include <graph_core/avoidance_goal_cost_function.h>


namespace pathplan
{

AvoidanceGoalCostFunction::AvoidanceGoalCostFunction(const ros::NodeHandle &nh):
  GoalCostFunction(),
  nh_(nh)
{

  urdf::Model model;
  model.initParam("robot_description");
  std::string base_frame = "world";
  std::string tool_frame = "tip";
  if (!nh_.getParam("base_frame", base_frame))
  {
    ROS_ERROR("%s/base_frame not defined", nh_.getNamespace().c_str());
    throw std::invalid_argument("base_frame is not defined");
  }
  if (!nh_.getParam("tool_frame", tool_frame))
  {
    ROS_ERROR("%s/tool_frame not defined", nh_.getNamespace().c_str());
    throw std::invalid_argument("base_frame is not defined");
  }
  if (!nh_.getParam("max_penalty", max_penalty_))
  {
    ROS_ERROR("%s/max_penalty not defined, use 1.0", nh_.getNamespace().c_str());
    max_penalty_=1.0;
  }
  if (!nh_.getParam("max_distance", max_distance_))
  {
    ROS_ERROR("%s/max_distance not defined, use 1.5 meter", nh_.getNamespace().c_str());
    max_distance_=1.5;
  }
  if (!nh_.getParam("min_distance", min_distance_))
  {
    ROS_ERROR("%s/min_distance not defined, use 0.5 meter", nh_.getNamespace().c_str());
    min_distance_=0.5;
  }
  if (!nh_.getParam("display_bubbles", plot))
  {
    ROS_ERROR("%s/display_bubbles not defined, use false", nh_.getNamespace().c_str());
    plot=false;
  }
  Eigen::Vector3d grav;
  grav << 0, 0, -9.806;

  chain_ = rosdyn::createChain(model, base_frame, tool_frame, grav);

  if (!nh_.getParam("links", links_))
  {
    ROS_ERROR("%s/links not defined, use all links", nh_.getNamespace().c_str());
    links_=chain_->getLinksName();
  }

  points_.resize(3,0);
  inv_delta_distance_=1.0/(max_distance_-min_distance_);

  if (plot)
  {
    marker_id_=0;
    marker_pub_ = nh_.advertise<visualization_msgs::Marker>("/goal_cost_function/avoidance_points", 1000);
    visualization_msgs::Marker marker;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.id=marker_id_;
    marker.ns = "avoidance";
    marker.header.frame_id="world";
    marker.header.stamp=ros::Time::now();
    marker.action = visualization_msgs::Marker::DELETE;

    for (unsigned int idx=0;idx<5;idx++)
    {
      marker_pub_.publish(marker);
      ros::Duration(0.01).sleep();
    }
  }

}

void AvoidanceGoalCostFunction::cleanPoints()
{
  points_.resize(3,0);

  if (!plot)
    return;

  visualization_msgs::Marker marker;
  marker.type = visualization_msgs::Marker::SPHERE;

  marker.ns = "avoidance";

  marker.header.frame_id="world";
  marker.header.stamp=ros::Time::now();
  marker.action = visualization_msgs::Marker::DELETEALL;
  marker_id_=0;

  marker_pub_.publish(marker);
  ros::Duration(0.1).sleep();

}
void AvoidanceGoalCostFunction::addPoint(const Eigen::Vector3d &point)
{
  for (int ic=0;ic<points_.cols();ic++)
  {
    if ((points_.col(ic)-point).norm()<1e-4)
      return;
  }
  points_.conservativeResize(3, points_.cols()+1);
  points_.col(points_.cols()-1) = point;

  if (!plot)
    return;

//  visualization_msgs::Marker marker;
//  marker.type = visualization_msgs::Marker::SPHERE;

//  marker.ns = "avoidance";
//  marker.pose.orientation.w=1.0;
//  tf::pointEigenToMsg(point,marker.pose.position);

//  marker.header.frame_id="world";
//  marker.header.stamp=ros::Time::now();
//  marker.action = visualization_msgs::Marker::ADD;
//  marker.id= marker_id_++;

//  marker.scale.x = 2.0*max_distance_;
//  marker.scale.y = 2.0*max_distance_;
//  marker.scale.z = 2.0*max_distance_;

//  marker.color.r = 1;
//  marker.color.g = 0;
//  marker.color.b = 0;
//  marker.color.a = 0.05;


//  marker_pub_.publish(marker);
//  ros::Duration(0.15).sleep();

//  if (min_distance_>0)
//  {
//    marker.scale.x = 2.0*min_distance_;
//    marker.scale.y = 2.0*min_distance_;
//    marker.scale.z = 2.0*min_distance_;
//    marker.id= marker_id_++;
//    marker.color.r = 1;
//    marker.color.g = 0;
//    marker.color.b = 0;
//    marker.color.a = .4;
//    marker_pub_.publish(marker);
//    ros::Duration(0.01).sleep();
//  }

}

void AvoidanceGoalCostFunction::publishPoints()
{
  visualization_msgs::Marker marker;
  marker.type = visualization_msgs::Marker::SPHERE_LIST;
  marker.ns = "avoidance";
  marker.pose.orientation.w=1.0;


  marker.header.frame_id="world";
  marker.header.stamp=ros::Time::now();
  marker.action = visualization_msgs::Marker::ADD;
  marker.id= marker_id_++;

  marker.scale.x = 2.0*max_distance_;
  marker.scale.y = 2.0*max_distance_;
  marker.scale.z = 2.0*max_distance_;

  marker.color.r = 1.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;
  marker.color.a = 0.05;

  for (int ic=0;ic<points_.cols();ic++)
  {
    Eigen::Vector3d point=points_.col(ic);
    geometry_msgs::Point p;
    tf::pointEigenToMsg(point,p);
    marker.points.push_back(p);
    marker.colors.push_back(marker.color);
  }



  marker_pub_.publish(marker);
  ros::Duration(0.15).sleep();

//  if (min_distance_>0)
//  {
//    marker.scale.x = 2.0*min_distance_;
//    marker.scale.y = 2.0*min_distance_;
//    marker.scale.z = 2.0*min_distance_;
//    marker.id= marker_id_++;
//    marker.color.r = 1;
//    marker.color.g = 0;
//    marker.color.b = 0;
//    marker.color.a = .4;
//    marker_pub_.publish(marker);
//    ros::Duration(0.01).sleep();
//  }

}

double AvoidanceGoalCostFunction::cost(const Eigen::VectorXd& configuration)
{
  double dist=std::numeric_limits<double>::infinity();
  for (const std::string& link: links_)
  {
    Eigen::Affine3d T_b_l=chain_->getTransformationLink(configuration,link);
    for (long ip=0;ip<points_.cols();ip++)
    {
      double d=(T_b_l.translation()-points_.col(ip)).norm();
      dist=(d<dist)?d:dist;
      if (dist<min_distance_)
        break;
    }
    if (dist<min_distance_)
      break;
  }

  if (dist<min_distance_)
    return max_penalty_;
  else if (dist<max_distance_)
    return max_penalty_*(max_distance_-dist)*inv_delta_distance_;

  return 0.0;
}

}
