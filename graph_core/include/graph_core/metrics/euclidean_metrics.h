#pragma once
/*
Copyright (c) 2024, Manuel Beschi and Cesare Tonola, JRL-CARI CNR-STIIMA/UNIBS,
manuel.beschi@unibs.it, c.tonola001@unibs.it All rights reserved.

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

#include <graph_core/metrics/metrics_base.h>

namespace graph
{
namespace core
{
/**
 * @class EuclideanMetrics
 * @brief This class defines a metrics which computes the cost between
 * configurations as their Euclidean distance.
 */
class EuclideanMetrics;
typedef std::shared_ptr<EuclideanMetrics> EuclideanMetricsPtr;

// Euclidean metrics
class EuclideanMetrics : public MetricsBase
{
protected:
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * @brief Empty constructor for EuclideanMetrics. The function
   * MetricsBase::init() must be called afterwards.
   */
  EuclideanMetrics() : MetricsBase()  // set initialized_ false
  {
  }

  /**
   * @brief Constructs a EuclideanMetrics object.
   * @param logger A shared pointer to a TraceLogger for logging.
   */
  EuclideanMetrics(const cnr_logger::TraceLoggerPtr& logger) : MetricsBase(logger)  // set initialized_ true
  {
  }

  /**
   * @brief Calculates the cost between two configurations.
   * @param configuration1 The first node.
   * @param configuration2 The second node.
   * @return The cost between the two configurations.
   */
  virtual double cost(const Eigen::VectorXd& configuration1, const Eigen::VectorXd& configuration2) override
  {
    return (configuration1 - configuration2).norm();
  }
  virtual double cost(const NodePtr& node1, const NodePtr& node2) override
  {
    return cost(node1->getConfiguration(), node2->getConfiguration());
  }

  /**
   * @brief Calculates the utopia (ideal minimum cost) between two
   * configurations.
   * @param configuration1 The first configuration.
   * @param configuration2 The second configuration.
   * @return The utopia distance between the two configurations.
   */
  virtual double utopia(const Eigen::VectorXd& configuration1, const Eigen::VectorXd& configuration2) override
  {
    return (configuration1 - configuration2).norm();
  }
  virtual double utopia(const NodePtr& node1, const NodePtr& node2) override
  {
    return utopia(node1->getConfiguration(), node2->getConfiguration());
  }

  /**
   * @brief Creates a clone of the EuclideanMetrics object.
   * @return A shared pointer to the cloned EuclideanMetrics object.
   */
  virtual MetricsPtr clone() override
  {
    return std::make_shared<EuclideanMetrics>(logger_);
  }
};

}  // end namespace core
}  // end namespace graph
