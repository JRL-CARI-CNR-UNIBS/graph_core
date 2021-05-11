#pragma once
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

#include <graph_core/metrics.h>
#include <rosdyn_core/primitives.h>
#include <visualization_msgs/MarkerArray.h>
#include <eigen_conversions/eigen_msg.h>

namespace pathplan
{
class AvoidanceMetrics;
typedef std::shared_ptr<AvoidanceMetrics> AvoidanceMetricsPtr;

// Avoidance metrics
class AvoidanceMetrics: public Metrics
{
protected:
  double step_ = 0.1;
  ros::NodeHandle nh_;
  rosdyn::ChainPtr chain_;

  double min_distance_=0.2;
  double max_distance_=1.0;
  double inv_delta_distance_;
  double max_penalty_=2.0;

  std::vector<std::string> links_;

  Eigen::Matrix<double,3,-1> points_;

  ros::Publisher marker_pub_;
  int marker_id_;
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  AvoidanceMetrics(const ros::NodeHandle& nh);

  void addPoint(const Eigen::Vector3d& point);
  void cleanPoints();

  virtual double cost(const Eigen::VectorXd& configuration1,
                      const Eigen::VectorXd& configuration2);

  virtual MetricsPtr clone();


};

}
