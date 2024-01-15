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


#include <graph_core/solvers/path_optimizers/path_optimizer_base.h>

namespace graph
{
namespace core
{

class PathLocalOptimizer;
typedef std::shared_ptr<PathLocalOptimizer> PathLocalOptimizerPtr;

/**
 * @class PathLocalOptimizer
 * @brief Derived class for local path optimization using warping and simplification techniques.
 *
 * This class inherits from PathOptimizerBase and provides additional methods for local path optimization.
 * It includes features such as warping to smooth the path and simplification to remove unnecessary nodes.
 */
class PathLocalOptimizer: public PathOptimizerBase
{

protected:

  /**
   * @brief Vector indicating whether warping changes were successful for each connection in the path.
   *
   * The change_warp_ vector is used to mark connections where warping changes were successful.
   * Each element in the vector corresponds to a connection in the path, and a value of true
   * indicates that warping changes were successful for the corresponding connection.
   */
  std::vector<bool> change_warp_;

  /**
   * @brief Maximum connection length threshold for path optimization 'simplify'.
   *
   * The maximum connection length is a threshold used in path optimization algorithms to determine
   * whether a connection in the path should be considered for simplification. If the
   * length of a connection is greather than this threshold, optimization technique 'simplify' may be
   * applied to remove it.
   */
  double simplify_max_conn_length_;

  /**
   * @brief Minimum connection length threshold for path optimization 'warp'.
   *
   * The minimum connection length is a threshold used in path optimization algorithms to determine
   * whether a connection in the path should be considered for warping. If the
   * length of a connection is less than this threshold, optimization technique 'warp' may be
   * applied to warp it.
   */
  double warp_min_conn_length_;

  /**
   * @brief Minimum step size for the warp process in path optimization.
   *
   * The minimum step size is a parameter used in the bisection process (warp optimization) of the path optimization
   * algorithm. It defines the minimum amount by which the configuration of an intermediate node
   * is adjusted during bisection to improve the overall cost of the path.
   */
  double warp_min_step_size_;

  /**
   * @brief Perform bisection to improve the path between two connections.
   *
   * This method applies bisection to refine the path between two connections in the path.
   * It adjusts the configuration of an intermediate node to improve the overall cost of the path.
   *
   * @param connection_idx The index of the connection in the path where bisection is applied.
   * @param center The center point for bisection.
   * @param direction The direction vector for bisection.
   * @param min_step_size The minimum step size for bisection.
   * @param max_distance The maximum distance for bisection.
   * @param min_distance The minimum distance for bisection.
   * @return Returns true if the bisection process results in an improvement, false otherwise.
   */
  bool bisection(const size_t& connection_idx,
                 const Eigen::VectorXd& center,
                 const Eigen::VectorXd& direction,
                 const double min_step_size,
                 double max_distance,
                 double min_distance);

  /**
   * @brief Perform one optimization step.
   * @param solution Output parameter where the optimized path will be stored.
   * @return True if the optimization step was successful, false otherwise.
   */
  bool step() override;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  PathLocalOptimizer(const CollisionCheckerPtr& checker,
                     const MetricsPtr& metrics,
                     const cnr_logger::TraceLoggerPtr& logger);

  /**
   * @brief Configure the optimizer with parameters from a YAML node.
   * @param config YAML node containing configuration parameters.
   */
  virtual void config(const YAML::Node& config);

  /**
   * @brief Set the input path for optimization. It calls base class setPath and resets 'change_warp_'.
   * @param path The input path to be optimized.
   */
  void setPath(const PathPtr &path) override;

  /**
   * @brief Attempt to bevel the path by adjusting configurations.
   *
   * This function attempts to warp the path by moving configurations along connections
   * where the norm of the connection is greater than the specified minimum distance. The
   * warping process aims to smooth the path and it is controlled by the change_warp_ vector,
   * and the maximum time limit for warping is given by max_time.
   *
   * @param min_conn_length The minimum connection's length threshold to consider a connection for warping.
   * @param min_step_size The minimum step size for the warping process.
   * @param max_time The maximum time limit for the warping process.
   * @return Returns true if any warping changes were made, false otherwise.
   */
  bool warp(const double& min_conn_length = 0.1,
            const double min_step_size = 0.01,
            const double& max_time = std::numeric_limits<double>::infinity());

  /**
   * @brief Simplify the path by skipping nodes.
   *
   * This method simplifies the path by removing connections shorter than a threshold. If the length of a connection is
   * shorter than 'min_conn_length', the algorithm skips the connection's child by connecting the parent of the previous connection
   * with the current child, if it results in a collision free connection.
   *
   * @param min_conn_length The minimum connection's length threshold for node removal.
   * @return True if the path is simplified, false otherwise.
   */
  bool simplify(const double &min_conn_length = 0.1);
};

} //end namespace core
} // end namespace graph
