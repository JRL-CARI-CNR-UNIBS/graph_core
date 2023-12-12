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



#include <graph_core/sampler_base.h>
#include <graph_core/graph/path.h>

namespace pathplan {

/**
 * @class TubeInformedSampler
 * @brief A sampler class for generating configurations in a path-informed manner.
 *
 * This class inherits from SamplerBase and implements a sampling strategy that biases
 * towards a given path. It considers a tubular region around the path and samples
 * configurations within this tube.
 */
class TubeInformedSampler: public SamplerBase
{
protected:
  /**
   * @brief path_ Waypoints defining the path.
   */
  std::vector<Eigen::VectorXd> path_;

  /**
   * @brief partial_length_ Accumulated lengths along the path.
   */
  std::vector<double> partial_length_;

  /**
   * @brief partial_cost_ Accumulated costs along the path.
   */
  std::vector<double> partial_cost_;

  /**
   * @brief radius_ Radius of the tubular region around the path.
   */
  double radius_;

  /**
   * @brief length_ Length of the entire path.
   */
  double length_;

  /**
   * @brief local_bias_ Local bias factor for sampling.
   */
  double local_bias_=0.8;

  /**
   * @brief sampler_ Pointer to the underlying sampler.
   */
  SamplerPtr sampler_;

  /**
   * @brief metrics_ Pointer to the metrics for evaluating costs.
   */
  MetricsPtr metrics_;

  /**
   * @brief Check if a given configuration could improve the cost along the path.
   * @param q The configuration to be checked.
   * @return True if the configuration could improve the cost, false otherwise.
   */
  bool couldImprove(const Eigen::VectorXd &q);

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * @brief Constructor for TubeInformedSampler.
   * @param start_configuration Start configuration of the path.
   * @param stop_configuration Stop configuration of the path.
   * @param sampler Pointer to the underlying sampler.
   * @param metrics Pointer to the metrics for evaluating costs.
   */
  TubeInformedSampler(const Eigen::VectorXd& start_configuration,
                      const Eigen::VectorXd& stop_configuration,
                      const SamplerPtr& sampler,
                      const MetricsPtr& metrics):
    SamplerBase(start_configuration,
                    stop_configuration,
                    sampler->getLB(),
                    sampler->getUB(),
                    sampler->getLogger(),
                    sampler->getCost())
  {
    length_ = 0;
    radius_=0;
    sampler_=sampler;
    metrics_=metrics;
  }

  /**
   * @brief Set the path for path-informed sampling.
   * @param path The path.
   * @return True if the path is successfully set, false otherwise.
   */
  bool setPath(const pathplan::PathPtr& path);
  bool setPath(const std::vector<Eigen::VectorXd>& path);
  bool setPath(const std::vector<std::vector<double>>& path);

  /**
   * @brief Set the radius of the tubular region around the path.
   * @param radius The radius of the tubular region.
   * @return True if the radius is valid, false otherwise.
   */
  bool setRadius(const double& radius);

  /**
   * @brief Set the local bias factor for sampling.
   * @param local_bias The local bias factor (between 0 and 1).
   * @return True if the local bias is valid, false otherwise.
   */
  bool setLocalBias(const double& local_bias);

  /**
    * @brief Compute a point on the path at a specified curvilinear abscissa.
    *
    * This function calculates a point on the path corresponding to a given
    * curvilinear abscissa. If the provided abscissa is outside the path range,
    * the function returns the closest endpoint of the path.
    *
    * @param abscissa The curvilinear abscissa along the path.
    * @return A point on the path at the specified abscissa.
    */
  Eigen::VectorXd pointOnCurvilinearAbscissa(const double& abscissa);

  /**
   * @brief Generate a path-informed sample configuration.
   * @return A sampled configuration.
   */
  virtual Eigen::VectorXd sample();

};

typedef std::shared_ptr<TubeInformedSampler> TubeInformedSamplerPtr;

}    // namespace pathplan
