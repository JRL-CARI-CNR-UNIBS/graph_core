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

#include <graph_core/util.h>
#include <random>

namespace graph_core
{

/**
 * @brief Base class for sampling configurations in a planning problem.
 *
 * This class provides a base interface for sampling configurations within the specified
 * bounds and handling cost-related functionality. Derived classes should implement
 * specific sampling strategies and behaviors.
 */
class SamplerBase;
typedef std::shared_ptr<SamplerBase> SamplerPtr;

class SamplerBase: public std::enable_shared_from_this<SamplerBase>
{
protected:

  /**
   * @brief lower_bound_ Lower bounds for configuration sampling.
   */
  Eigen::VectorXd lower_bound_;

  /**
   * @brief upper_bound_ Upper bounds for configuration sampling.
   */
  Eigen::VectorXd upper_bound_;

  /**
   * @brief Pointer to a TraceLogger instance for logging.
   *
   * This member variable represents a pointer to a TraceLogger instance, allowing
   * to perform logging operations. TraceLogger is a part of the cnr_logger library.
   * Ensure that the logger is properly configured and available for use.
   */
  const cnr_logger::TraceLoggerPtr& logger_;

  /**
   * @brief cost_ Cost associated with the sampler. The base implementation of the Sampler does not use the cost
   * in the sampling procedure.
   */
  double cost_;

  /**
   * @brief ndof_ Number of degrees of freedom in the configuration.
   */
  unsigned int ndof_;

  /**
   * @brief rd_ Random device for seed generation.
   */
  std::random_device rd_;

  /**
   * @brief gen_  Mersenne Twister engine for random number generation.
   */
  std::mt19937 gen_;

  /**
   * @brief ud_ Uniform distribution for random numbers.
   */
  std::uniform_real_distribution<double> ud_;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * @brief Constructor for the SamplerBase class.
   *
   * @param lower_bound Lower bounds for configuration sampling.
   * @param upper_bound Upper bounds for configuration sampling.
   * @param logger Pointer to a TraceLogger instance for logging.
   * @param cost Cost associated with the sampler (default: infinity).
   */
  SamplerBase(const Eigen::VectorXd& lower_bound,
              const Eigen::VectorXd& upper_bound,
              const cnr_logger::TraceLoggerPtr& logger,
              const double& cost = std::numeric_limits<double>::infinity()):
    lower_bound_(lower_bound),
    upper_bound_(upper_bound),
    logger_(logger),
    cost_(cost),
    gen_{rd_()}
  {
    srand((unsigned int)time(NULL)); //randomize seed
    ud_ = std::uniform_real_distribution<double>(0, 1);
  }

  /**
   * @brief Get the cost associated with the sampler.
   * @return Cost associated with the sampler.
   */
  const double& getCost(){return cost_;}

  /**
   * @brief Set the cost associated with the sampler.
   *
   * Derived classes should implement this method.
   * @param cost Cost to be set.
   */
  virtual void setCost(const double& cost) = 0;

  /**
   * @brief Sample a configuration using the sampling strategy.
   *
   * Derived classes should implement this method.
   * @return Sampled configuration.
   */
  virtual Eigen::VectorXd sample()=0;

  /**
   * @brief Check if a given configuration is within bounds.
   * @param q Configuration to check.
   * @return True if the configuration is within bounds, false otherwise.
   */
  virtual bool inBounds(const Eigen::VectorXd& q)
  {
    for (unsigned int iax = 0; iax < ndof_; iax++)
    {
      if (q(iax) > upper_bound_(iax) || q(iax) < lower_bound_(iax))
      {
        return false;
      }
    }
    return true;
  }

  /**
   * @brief Check if the sampler should collapse.
   * A sampler collapses when it cannot generate new samples.
   * Examples:
   *  - An Informed Sampler collapses when the cost that defines the ellipsoid is less than the distance
   *    between the focii of the ellipsoid
   *  - A standard uniform sampler never collapses (i.e., the function returns always false).
   *
   * Derived classes should implement this method.
   * @return True if the sampler should collapse, false otherwise.
   */
  virtual bool collapse()=0;

  /**
   * @brief Get the lower bounds of the sampler.
   * @return Lower bounds of the sampler.
   */
  const Eigen::VectorXd& getLB(){return lower_bound_;}

  /**
   * @brief Get the upper bounds of the sampler.
   * @return Upper bounds of the sampler.
   */
  const Eigen::VectorXd& getUB(){return upper_bound_;}

  /**
   * @brief Get the dimension of the sampler.
   * @return Dimension of the sampler.
   */
  const unsigned int& getDimension()const {return ndof_;}

  /**
   * @brief Get the logger associated with the sampler.
   * @return Pointer to the TraceLogger instance.
   */
  const cnr_logger::TraceLoggerPtr& getLogger(){return logger_;}

  /**
   * @brief Creates a clone of the Sampler object.
   *
   * Derived classes should implement this method.
   * @return A shared pointer to the cloned Sampler object.
   */
  virtual SamplerPtr clone() = 0;
};

}  // namespace graph_core
