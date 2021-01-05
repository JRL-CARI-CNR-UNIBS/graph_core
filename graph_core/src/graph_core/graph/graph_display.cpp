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

#include <graph_core/graph/graph_display.h>



namespace pathplan
{

Display::Display(const planning_scene::PlanningScenePtr planning_scene,
                 const std::string& group_name,
                 const std::string& last_link):
  planning_scene_(planning_scene),
  group_name_(group_name),
  last_link_(last_link)
{
  node_marker_scale_.resize(3,0.05);
  connection_marker_scale_.resize(3,0.02);
  tree_marker_scale_.resize(3,0.01);
  marker_id_=0;
  state_=std::make_shared<moveit::core::RobotState>(planning_scene_->getCurrentState());
  marker_pub_ = nh_.advertise<visualization_msgs::Marker>("/marker_visualization_topic", 1000);
  for (int idx=0;idx<4;idx++)
    clearMarkers();
}

void Display::clearMarkers(const std::string& ns)
{
  visualization_msgs::Marker marker;
  marker.action = visualization_msgs::Marker::DELETEALL;
  marker.header.stamp=ros::Time::now();

  marker.ns = ns;
  marker.id= marker_id_++;

  marker_pub_.publish(marker);
  ros::Duration(DISPLAYTIME).sleep();
}
void Display::clearMarker(const int& id,const std::string& ns)
{
  visualization_msgs::Marker marker;
  marker.action = visualization_msgs::Marker::DELETE;
  marker.header.stamp=ros::Time::now();

  marker.ns = ns;
  marker.id= id;

  marker_pub_.publish(marker);
  ros::Duration(DISPLAYTIME).sleep();
}

int Display::displayNode(const NodePtr &n,
                          const std::string& ns,
                          const std::vector<double> &marker_color,
                          const bool &plot_state)
{

  visualization_msgs::Marker marker;
  marker.type = visualization_msgs::Marker::SPHERE;

  marker.ns = ns;
  state_->setJointGroupPositions(group_name_,n->getConfiguration());
  tf::poseEigenToMsg(state_->getGlobalLinkTransform(last_link_),marker.pose);


  marker.header.frame_id="world";
  marker.header.stamp=ros::Time::now();
  marker.action = visualization_msgs::Marker::ADD;
  marker.id= marker_id_++;

  marker.scale.x = node_marker_scale_.at(0);
  marker.scale.y = node_marker_scale_.at(1);
  marker.scale.z = node_marker_scale_.at(2);

  marker.color.r = marker_color.at(0);
  marker.color.g = marker_color.at(1);
  marker.color.b = marker_color.at(2);
  marker.color.a = marker_color.at(3);

  marker_pub_.publish(marker);
  ros::Duration(DISPLAYTIME).sleep();
  return marker.id;
}

int Display::displayConnection(const ConnectionPtr& conn,
                                const std::string& ns,
                                const std::vector<double>& marker_color,
                                const bool& plot_state)
{
  visualization_msgs::Marker marker;
  marker.ns = ns;
  marker.type = visualization_msgs::Marker::LINE_STRIP;

  geometry_msgs::Pose pose;
  state_->setJointGroupPositions(group_name_,conn->getParent()->getConfiguration());
  tf::poseEigenToMsg(state_->getGlobalLinkTransform(last_link_),pose);
  marker.points.push_back(pose.position);

  state_->setJointGroupPositions(group_name_,conn->getChild()->getConfiguration());
  tf::poseEigenToMsg(state_->getGlobalLinkTransform(last_link_),pose);
  marker.points.push_back(pose.position);

  marker.header.frame_id="world";
  marker.header.stamp=ros::Time::now();
  marker.action = visualization_msgs::Marker::ADD;
  marker.id= marker_id_++;

  marker.scale.x = connection_marker_scale_.at(0);
  marker.scale.y = connection_marker_scale_.at(1);
  marker.scale.z = connection_marker_scale_.at(2);

  marker.color.r = marker_color.at(0);
  marker.color.g = marker_color.at(1);
  marker.color.b = marker_color.at(2);
  marker.color.a = marker_color.at(3);

  marker_pub_.publish(marker);
  ros::Duration(DISPLAYTIME).sleep();
  return marker.id;
}

int Display::displayPath(const PathPtr &path,
                          const std::string& ns,
                          const std::vector<double> &marker_color,
                          const bool &plot_state)
{
  visualization_msgs::Marker marker;
  marker.ns = ns;
  marker.type = visualization_msgs::Marker::LINE_LIST;
  marker.header.frame_id="world";
  marker.header.stamp=ros::Time::now();
  marker.action = visualization_msgs::Marker::ADD;
  marker.id= marker_id_++;

  marker.scale.x = connection_marker_scale_.at(0);
  marker.scale.y = connection_marker_scale_.at(1);
  marker.scale.z = connection_marker_scale_.at(2);

  marker.color.r = marker_color.at(0);
  marker.color.g = marker_color.at(1);
  marker.color.b = marker_color.at(2);
  marker.color.a = marker_color.at(3);


  std::vector<ConnectionPtr> connections=path->getConnections();
  if (connections.size()==0)
    return marker.id;
  for (const ConnectionPtr& conn: connections)
  {
    geometry_msgs::Pose pose;
    state_->setJointGroupPositions(group_name_,conn->getParent()->getConfiguration());
    tf::poseEigenToMsg(state_->getGlobalLinkTransform(last_link_),pose);
    marker.points.push_back(pose.position);

    state_->setJointGroupPositions(group_name_,conn->getChild()->getConfiguration());
    tf::poseEigenToMsg(state_->getGlobalLinkTransform(last_link_),pose);
    marker.points.push_back(pose.position);
  }

  marker_pub_.publish(marker);
  ros::Duration(DISPLAYTIME).sleep();
  return marker.id;
}

int Display::displayTree(const TreePtr &tree,
                          const std::string &ns,
                          const std::vector<double> &marker_color)
{
  visualization_msgs::Marker marker;
  marker.ns = ns;
  marker.type = visualization_msgs::Marker::LINE_LIST;
  marker.header.frame_id="world";
  marker.header.stamp=ros::Time::now();
  marker.action = visualization_msgs::Marker::ADD;
  marker.id= marker_id_++;

  marker.scale.x = tree_marker_scale_.at(0);
  marker.scale.y = tree_marker_scale_.at(1);
  marker.scale.z = tree_marker_scale_.at(2);

  marker.color.r = marker_color.at(0);
  marker.color.g = marker_color.at(1);
  marker.color.b = marker_color.at(2);
  marker.color.a = marker_color.at(3);
  displayTreeNode(tree->getRoot(),tree->getDirection(),marker.points);

  marker_pub_.publish(marker);
  ros::Duration(DISPLAYTIME).sleep();
  return marker.id;

}

void Display::displayTreeNode(const NodePtr &n,
                              const Direction& direction,
                              std::vector<geometry_msgs::Point>& points)
{
  std::vector<ConnectionPtr> connections;
  if (direction==Direction::Forward)
    connections=n->child_connections_;
  else
    connections=n->parent_connections_;
  if (connections.size()==0)
    return;
  for (const ConnectionPtr& conn: connections)
  {
    geometry_msgs::Pose pose;
    state_->setJointGroupPositions(group_name_,conn->getParent()->getConfiguration());
    tf::poseEigenToMsg(state_->getGlobalLinkTransform(last_link_),pose);
    points.push_back(pose.position);

    state_->setJointGroupPositions(group_name_,conn->getChild()->getConfiguration());
    tf::poseEigenToMsg(state_->getGlobalLinkTransform(last_link_),pose);
    points.push_back(pose.position);

    displayTreeNode(conn->getChild(),direction,points);
  }
}
}
