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
#include <human_probablistic_occupancy/human_probablistic_occupancy.h>
namespace pathplan
{



// Euclidean metrics
class OccupancyMetrics: public Metrics
{
protected:
  human_occupancy::OccupancyFilterPtr filter_;
  double step_ = 0.1;
  double weight_ = 1;
  ros::NodeHandle nh_;
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  OccupancyMetrics(ros::NodeHandle& nh);

  /*
   *
   * length total length between configurations
   * step_ checking distance
   * nsteps number of checking steps
   * d=length/nsteps distance between checking steps
   * occupancy(i) occupancy at the checking step i
   *
   * cost between checking steps i and i+1
   * cost(i) = length*(1+0.5*weigth_*(occupancy(i)+occupancy(i+1))
   *
   * cost(0)   = d*(1+0.5*weigth_*(occupancy(0)  +occupancy(1))
   * cost(1)   = d*(1+0.5*weigth_*(occupancy(1)  +occupancy(2))
   * cost(2)   = d*(1+0.5*weigth_*(occupancy(2)  +occupancy(3))
   * ....
   * cost(n-1) = d*(1+0.5*weigth_*(occupancy(n-1)+occupancy(n))
   *
   * total_cost = sum(cost(i))  with i=0,...,n-1
   * total_cost= d*nsteps+ d*weigth*(0.5*occupancy(0)+occupancy(1)+occupancy(2)+...+occupancy(n-1)+0.5*occupancy(n))
   * total_cost= length+ d*weigth*(0.5*occupancy(0)+occupancy(1)+occupancy(2)+...+occupancy(n-1)+0.5*occupancy(n))
   *
   */
  virtual double cost(const Eigen::VectorXd& configuration1,
                      const Eigen::VectorXd& configuration2);

  virtual double cost(const NodePtr& node1,
                      const NodePtr& node2);


};
typedef std::shared_ptr<OccupancyMetrics> OccupancyMetricsPtr;

}
