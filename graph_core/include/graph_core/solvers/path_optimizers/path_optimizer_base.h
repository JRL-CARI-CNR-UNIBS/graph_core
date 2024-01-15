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

#include <graph_core/graph/path.h>
#include <graph_core/metrics/metrics_base.h>
#include <graph_core/collision_checkers/collision_checker_base.h>

namespace graph
{
namespace core
{

//class PathOptimizerBase; //Defined in util.h
typedef std::shared_ptr<PathOptimizerBase> PathOptimizerPtr;


/**
 * @brief Base class for path optimization algorithms, such as local optimizers.
 *
 * This class serves as the base class for various path optimization algorithms.
 * It provides a common interface and structure for different optimization approaches.
 */

class PathOptimizerBase: public std::enable_shared_from_this<PathOptimizerBase>
{
 friend class Path;

protected:

  /**
   * @brief config_ Configuration parameters for the optimizer.
   */
  YAML::Node config_;

  /**
   * @brief path_ The input path to be optimized.
   */
  PathPtr path_;

  /**
   * @brief checker_ Collision checker for path validity.
   */
  CollisionCheckerPtr checker_;

  /**
   * @brief metrics_ Metrics used to evaluate the quality of the path.
   */
  MetricsPtr metrics_;

  /**
   * @brief configured_ Flag indicating whether the optimizer is configured ('config' method executed).
   */
  bool configured_;

  /**
   * @brief stall_gen_ Counter for stalled generations.
   */
  unsigned int stall_gen_;

  /**
   * @brief max_stall_gen_ Maximum number of stalled generations allowed.
   */
  unsigned int max_stall_gen_;

  /**
   * @brief solved_ Flag indicating whether the optimization process has found a solution.
   */
  bool solved_;

  /**
   * @brief Pointer to a TraceLogger instance for logging.
   *
   * This member variable represents a pointer to a TraceLogger instance, allowing
   * to perform logging operations. TraceLogger is a part of the cnr_logger library.
   * Ensure that the logger is properly configured and available for use.
   */
  const cnr_logger::TraceLoggerPtr& logger_;


  /**
   * @brief Perform one optimization step. Derived classes must implement this function.
   * @return True if the optimization step was successful, false otherwise.
   */
  virtual bool step() = 0;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * @brief Constructor for PathOptimizerBase.
   * @param checker Collision checker for path validity.
   * @param metrics Metrics used to evaluate the quality of the path.
   * @param logger Logger for debugging and logging.
   */
  PathOptimizerBase(const CollisionCheckerPtr& checker,
                     const MetricsPtr& metrics,
                     const cnr_logger::TraceLoggerPtr& logger);

  /**
   * @brief Set the input path for optimization and reset the flags 'solved_' and 'stall_gen_'.
   * @param path The input path to be optimized.
   */
  virtual void setPath(const PathPtr& path);

  /**
   * @brief Get the optimized path.
   * @return The shared pointer to the optimized path.
   */
  virtual PathPtr getPath();

  /**
   * @brief Configure the optimizer with parameters from a YAML node.
   * @param config YAML node containing configuration parameters.
   */
  virtual void config(const YAML::Node& config);

  /**
   * @brief Solve the optimization problem. Use 'getPath' to get the processed path.
   * @param max_iteration Maximum number of iterations for the optimization process.
   * @param max_time Maximum time (in seconds) allowed for the optimization process.
   * @return True if a solution was found, false otherwise.
   */
  virtual bool solve(const unsigned int& max_iteration = 100, const double &max_time = std::numeric_limits<double>::infinity());
};

} //end namespace core
} // end namespace graph
