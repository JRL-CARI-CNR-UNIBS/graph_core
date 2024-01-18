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

namespace graph
{
namespace core
{

/**
 * @class MetricsBase
 * @brief Base class for defining metrics to measure costs between configurations.
 *
 * The MetricsBase class provides an interface for cost evaluation
 * in path planning. Users can derive from this class to implement custom
 * metrics.
 */
class MetricsBase;
typedef std::shared_ptr<MetricsBase> MetricsPtr;

class MetricsBase: public std::enable_shared_from_this<MetricsBase>
{
protected:

  /**
   * @brief init_ Flag to indicate whether the object is initialised, i.e. whether its members have been defined correctly.
   * It is false when the object is created with an empty constructor. In this case, call the 'init' function to initialise it.
   * The other constructors automatically initialise the object.
   * As long as the object is not initialised, it cannot perform its main functions.
   */
  bool init_;

  /**
   * @brief Pointer to a TraceLogger instance for logging.
   *
   * This member variable represents a pointer to a TraceLogger instance, allowing
   * to perform logging operations. TraceLogger is a part of the cnr_logger library.
   * Ensure that the logger is properly configured and available for use.
   */
  cnr_logger::TraceLoggerPtr logger_;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * @brief Empty constructor for MetricsBase. The function init() must be called afterwards.
   */
  MetricsBase()
  {
    init_ = false;
  }

  /**
   * @brief Constructs a MetricsBase object.
   * @param logger A shared pointer to a TraceLogger for logging.
   */
  MetricsBase(const cnr_logger::TraceLoggerPtr& logger):logger_(logger)
  {
    init_ = true;
  }

  /**
   * @brief init Initialise the object, defining its main attributes. At the end of the function, the flag 'init_' is set to true and the object can execute its main functions.
   * @param logger Pointer to a TraceLogger for logging.
   * @return True if correctly initialised, False if already initialised.
   */
  virtual bool init(const cnr_logger::TraceLoggerPtr& logger)
  {
    if(init_)
    {
      CNR_WARN(logger_,"Metrics already initialised!");
      return false;
    }

    logger_ = logger;
    init_ = true;

    return true;
  }

  /**
   * @brief getInit tells if the object has been initialised.
   * @return the 'init_' flag.
   */
  bool getInit()
  {
    return init_;
  }

  /**
   * @brief Calculates the cost between two configurations.
   * @param configuration1 The first node.
   * @param configuration2 The second node.
   * @return The cost between the two configurations.
   */
  virtual double cost(const Eigen::VectorXd& configuration1,
                      const Eigen::VectorXd& configuration2) = 0;
  virtual double cost(const NodePtr& node1,
                      const NodePtr& node2) = 0;

  /**
   * @brief Calculates the utopia (ideal minimum cost) between two configurations.
   * @param configuration1 The first configuration.
   * @param configuration2 The second configuration.
   * @return The utopia distance between the two configurations.
   */
  virtual double utopia(const Eigen::VectorXd& configuration1,
                        const Eigen::VectorXd& configuration2) = 0;
  virtual double utopia(const NodePtr& node1,
                        const NodePtr& node2) = 0;
  /**
   * @brief Creates a clone of the MetricsBase object.
   * @return A shared pointer to the cloned Metrics object.
   */
  virtual MetricsPtr clone() = 0;

};

} //end namespace core
} // end namespace graph
