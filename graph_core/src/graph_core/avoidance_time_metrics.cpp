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

#include <graph_core/avoidance_time_metrics.h>


namespace pathplan
{

AvoidanceTimeMetrics::AvoidanceTimeMetrics(const Eigen::VectorXd &max_speed,
                                           const double &nu,
                                           const ros::NodeHandle &nh):
  TimeBasedMetrics(max_speed,nu),
  nh_(nh)
{

  urdf::Model model;
  model.initParam("robot_description");
  base_frame_ = "world";
  std::string tool_frame = "tip";
  if (!nh_.getParam("base_frame", base_frame_))
  {
    ROS_ERROR("%s/base_frame not defined", nh_.getNamespace().c_str());
    throw std::invalid_argument("base_frame is not defined");
  }
  if (!nh_.getParam("tool_frame", tool_frame))
  {
    ROS_ERROR("%s/tool_frame not defined", nh_.getNamespace().c_str());
    throw std::invalid_argument("base_frame is not defined");
  }
  if (!nh_.getParam("computation_step", step_))
  {
    ROS_ERROR("%s/computation_step not defined, use 0.1", nh_.getNamespace().c_str());
    step_=0.1;
  }

  Eigen::Vector3d grav;
  grav << 0, 0, -9.806;

  chain_ = rosdyn::createChain(model, base_frame_, tool_frame, grav);

  if (!nh_.getParam("links", links_))
  {
    ROS_ERROR("%s/links not defined, use all links", nh_.getNamespace().c_str());
    links_=chain_->getLinksName();
  }
  else
    for (const std::string& s: links_)
      ROS_INFO("link %s",s.c_str());

  ssm_=std::make_shared<ssm15066::DeterministicSSM>(chain_,nh);

  points_.resize(3,0);
}

void AvoidanceTimeMetrics::cleanPoints()
{
  points_.resize(3,0);
}
void AvoidanceTimeMetrics::addPoint(const Eigen::Vector3d &point)
{
  points_.conservativeResize(3, points_.cols()+1);
  points_.col(points_.cols()-1) = point;
  ssm_->setPointCloud(points_);
}

double AvoidanceTimeMetrics::cost(const Eigen::VectorXd& configuration1,
                                  const Eigen::VectorXd& configuration2)
{
  double nominal_time = (inv_max_speed_.cwiseProduct(configuration1 - configuration2)).cwiseAbs().maxCoeff()+nu_*(configuration2-configuration1).norm();
  if (nominal_time==0.0)
    return 0.0;

  Eigen::VectorXd nominal_velocity= (configuration1 - configuration2)/nominal_time;

  double length = (configuration1 - configuration2).norm();
  double cost=0;
  unsigned int nsteps = std::ceil(length / step_);
  double inv_nsteps = 1.0 / nsteps;
  double segment_time = nominal_time / nsteps;

  for (unsigned int istep = 0; istep < nsteps; istep++)
  {
    Eigen::VectorXd q = configuration1 + (configuration2 - configuration1) * inv_nsteps * (istep+0.5);

    double scaling=ssm_->computeScaling(q,nominal_velocity);
    cost+=segment_time/(scaling+1e-6); //avoid division by zero
  }
  return cost;
}


double AvoidanceTimeMetrics::utopia(const Eigen::VectorXd& configuration1,
                                  const Eigen::VectorXd& configuration2)
{
  return TimeBasedMetrics::utopia(configuration1,configuration2);
}

MetricsPtr AvoidanceTimeMetrics::clone()
{
  return std::make_shared<AvoidanceTimeMetrics>(max_speed_,nu_,nh_);
}


}
