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

#include <graph_core/avoidance_probabilistic_time_metrics.h>


namespace pathplan
{

ProbabilistcAvoidanceTimeMetrics::ProbabilistcAvoidanceTimeMetrics(const Eigen::VectorXd &max_speed,
                                           const double &nu,
                                           const ros::NodeHandle &nh):
  AvoidanceTimeMetrics(max_speed,nu,nh)
{
  probabilistic_ssm_=std::make_shared<ssm15066::ProbabilisticSSM>(chain_,nh);
  ssm_=probabilistic_ssm_;
  occupancy_.resize(0);
}

void ProbabilistcAvoidanceTimeMetrics::cleanPoints()
{
  points_.resize(3,0);
  occupancy_.resize(0);
}

void ProbabilistcAvoidanceTimeMetrics::addPointOccupancy(const Eigen::Vector3d &point, const double& occupancy)
{
  points_.conservativeResize(3, points_.cols()+1);
  points_.col(points_.cols()-1) = point;
  occupancy_.conservativeResize(points_.cols()+1,1);
  occupancy_(points_.cols()-1)=std::max(0.0,std::min(occupancy,1.0));
  probabilistic_ssm_->setPointCloud(points_,occupancy_);
}

MetricsPtr ProbabilistcAvoidanceTimeMetrics::clone()
{
  return std::make_shared<ProbabilistcAvoidanceTimeMetrics>(max_speed_,nu_,nh_);
}


}
