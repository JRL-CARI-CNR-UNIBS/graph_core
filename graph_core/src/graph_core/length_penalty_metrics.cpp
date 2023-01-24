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

#include <graph_core/length_penalty_metrics.h>


namespace pathplan
{

LengthPenaltyMetrics::LengthPenaltyMetrics():
  Metrics()
{
  obstacles_position_.clear();
}

LengthPenaltyMetrics::LengthPenaltyMetrics(const std::vector<Eigen::Vector3d>& obstacles_position):
  obstacles_position_(obstacles_position),
  Metrics()
{
}

double LengthPenaltyMetrics::cost(const NodePtr& node1,
                                  const NodePtr& node2)
{
  return LengthPenaltyMetrics::cost(node1->getConfiguration(),node2->getConfiguration());
}

double LengthPenaltyMetrics::cost(const Eigen::VectorXd& configuration1,
                                  const Eigen::VectorXd& configuration2)
{
  double lambda = getLambda(configuration1, configuration2);
  return (Metrics::cost(configuration1, configuration2))*lambda;
}

double LengthPenaltyMetrics::utopia(const NodePtr& node1,
                                    const NodePtr& node2)
{
  return LengthPenaltyMetrics::utopia(node1->getConfiguration(), node2->getConfiguration());
}


double LengthPenaltyMetrics::utopia(const Eigen::VectorXd& configuration1,
                                    const Eigen::VectorXd& configuration2)
{
  return Metrics::utopia(configuration1, configuration2);
}

double LengthPenaltyMetrics::getLambda(const Eigen::VectorXd& configuration1,  //DA FARE
                                       const Eigen::VectorXd& configuration2)
{
  ROS_ERROR("TO BE IMPLEMENTED");
  return 0;
}

MetricsPtr LengthPenaltyMetrics::clone()
{
  return std::make_shared<LengthPenaltyMetrics>();
}

}
