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

#include <graph_core/graph/node.h>
namespace graph_core
{

/**
 * @class Metrics
 * @brief Base class for defining metrics to measure costs.
 */
class Metrics;
typedef std::shared_ptr<Metrics> MetricsPtr;


// Euclidean metrics
class Metrics
{
protected:
  /**
   * @brief Pointer to a TraceLogger instance for logging.
   *
   * This member variable represents a pointer to a TraceLogger instance, allowing
   * to perform logging operations. TraceLogger is a part of the cnr_logger library.
   * Ensure that the logger is properly configured and available for use.
   */
  const cnr_logger::TraceLoggerPtr& logger_;
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * @brief Constructs a Metrics object.
   * @param logger A shared pointer to a TraceLogger for logging.
   */
  Metrics(const cnr_logger::TraceLoggerPtr& logger);

  /**
   * @brief Calculates the cost between two configurations.
   * @param configuration1 The first node.
   * @param configuration2 The second node.
   * @return The cost between the two configurations.
   */
  virtual double cost(const Eigen::VectorXd& configuration1,
                      const Eigen::VectorXd& configuration2);
  virtual double cost(const NodePtr& node1,
                      const NodePtr& node2);

  /**
   * @brief Calculates the utopia (ideal minimum cost) between two configurations.
   * @param configuration1 The first configuration.
   * @param configuration2 The second configuration.
   * @return The utopia distance between the two configurations.
   */
  virtual double utopia(const Eigen::VectorXd& configuration1,
                        const Eigen::VectorXd& configuration2);
  virtual double utopia(const NodePtr& node1,
                        const NodePtr& node2);

  /**
   * @brief Creates a clone of the Metrics object.
   * @return A shared pointer to the cloned Metrics object.
   */
  virtual MetricsPtr clone();

};

}
