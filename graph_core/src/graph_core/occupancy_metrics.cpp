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

#include <graph_core/occupancy_metrics.h>


namespace pathplan
{
OccupancyMetrics::OccupancyMetrics(ros::NodeHandle& nh):
  Metrics(),
  nh_(nh)
{

  urdf::Model model;
  model.initParam("robot_description");
  std::string base_frame = "world";
  std::string tool_frame = "tip";
  if (!nh_.getParam("base_frame", base_frame))
  {
    ROS_ERROR("%s/base_frame not defined", nh_.getNamespace().c_str());
  }
  if (!nh_.getParam("tool_frame", tool_frame))
  {
    ROS_ERROR("%s/tool_frame not defined", nh_.getNamespace().c_str());
  }
  if (!nh_.getParam("weight", weight_))
  {
    ROS_ERROR("%s/weight not defined", nh_.getNamespace().c_str());
  }
  if (!nh_.getParam("computation_step", step_))
  {
    ROS_ERROR("%s/computation_step not defined", nh_.getNamespace().c_str());
  }

  Eigen::Vector3d grav;
  grav << 0, 0, -9.806;

  rosdyn::ChainPtr chain = rosdyn::createChain(model, base_frame, tool_frame, grav);
  human_occupancy::OccupancyGridPtr grid = std::make_shared<human_occupancy::OccupancyGrid>(nh);
  filter_ = std::make_shared<human_occupancy::OccupancyFilter>(chain, grid, nh_);
}


double OccupancyMetrics::cost(const NodePtr& node1,
                              const NodePtr& node2)
{
  return cost(node1->getConfiguration(), node2->getConfiguration());
}

double OccupancyMetrics::cost(const Eigen::VectorXd& configuration1,
                              const Eigen::VectorXd& configuration2)
{
  double length = (configuration1 - configuration2).norm();
  if (length < 1e-6)
    return length;

  double cost = length;
  unsigned int nsteps = std::ceil(length / step_);
  double inv_nsteps = 1.0 / nsteps;
  double distance = length / nsteps;
  for (unsigned int istep = 0; istep <= nsteps; istep++)
  {
    Eigen::VectorXd q = configuration1 + (configuration2 - configuration1) * inv_nsteps * istep;
    double occupancy = filter_->occupancy(q);
    if (istep == 0 || istep == nsteps)
      cost += 0.5 * occupancy * weight_ * distance;
    else
      cost += occupancy * weight_ * distance;
  }

  return cost;
}

}
