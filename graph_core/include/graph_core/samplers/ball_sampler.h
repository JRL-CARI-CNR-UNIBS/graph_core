#pragma once
/*
Copyright (c) 2024, Cesare Tonola UNIBS c.tonola001@unibs.it
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

#include <graph_core/samplers/sampler_base.h>

#include <eigen3/Eigen/Core>
#include <graph_core/util.h>
#include <random>

namespace graph
{
namespace core
{

/**
 * @class BallSampler
 * @brief Sampling in a defined ball.
 *
 * The BallSampler class inherits from SamplerBase and samples in a ball
 * with radius set by "cost" and center provided by input.
 */
class BallSampler;
typedef std::shared_ptr<BallSampler> BallSamplerPtr;

class BallSampler: public SamplerBase
{
protected:

  /**
   * @brief ball_center_ center of the ball.
   */
  Eigen::VectorXd ball_center_;

  //double radius_;  //SamplerBase::cost_ is considered as the radius

  /**
   * @brief ball_ the ball to sample from.
   */
  Eigen::VectorXd ball_;

  /**
   * @brief Configure the sampler parameters.
   */
  virtual void config();

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * @brief Empty constructor for BallSampler. The function init() must be called afterwards.
   */
  BallSampler():SamplerBase() //set initialized_ false
  {}

  /**
   * @brief Constructor for BallSampler.
   * @param ball_center Center of the ball.
   * @param lower_bound Lower bounds for each dimension.
   * @param upper_bound Upper bounds for each dimension.
   * @param logger TraceLogger for logging.
   * @param radius radius of the ball (default: infinity).
   */
  BallSampler(const Eigen::VectorXd& ball_center,
              const Eigen::VectorXd& lower_bound,
              const Eigen::VectorXd& upper_bound,
              const cnr_logger::TraceLoggerPtr& logger,
              const double& radius):
    SamplerBase(lower_bound,upper_bound,logger,radius), //set initialized_ true
    ball_center_(ball_center)
  {
    config();
  }

  /**
   * @brief init Initialise the object, defining its main attributes. At the end of the function, the flag 'initialized_' is set to true and the object can execute its main functions.
   * @param ball_center Center of the ball.
   * @param lower_bound Lower bounds for configuration sampling.
   * @param upper_bound Upper bounds for configuration sampling.
   * @param logger Pointer to a TraceLogger instance for logging.
   * @param radius radius of the ball (default: infinity).
   * @return True if correctly initialised, False if already initialised.
   */
  virtual bool init(const Eigen::VectorXd& ball_center,
                    const Eigen::VectorXd& lower_bound,
                    const Eigen::VectorXd& upper_bound,
                    const cnr_logger::TraceLoggerPtr& logger,
                    const double& radius = std::numeric_limits<double>::infinity())
  {
    if(not SamplerBase::init(lower_bound,upper_bound,logger,radius))
      return false;

    ball_center_ = ball_center;
    config();

    return true;
  }

  /**
   * @brief Generate a sampled configuration.
   * @return Sampled configuration.
   */
  virtual Eigen::VectorXd sample() override;

  /**
   * @brief Check if a configuration is within the bounds.
   * @param q Configuration to check.
   * @return True if in bounds, false otherwise.
   */
  virtual bool inBounds(const Eigen::VectorXd& q) override;

  /**
   * @brief Get the ball center of the ball sampler.
   * @return Ball center of the ball sampler.
   */
  const Eigen::VectorXd& getBallCenter() {return ball_center_;}

  /**
   * @brief Set the cost associated with the sampler.
   *
   * Derived classes should implement this method.
   * @param cost Cost to be set.
   */
  virtual void setCost(const double& cost) override;

  /**
   * @brief Check if the informed bounds collapse (focii distance exceeds cost).
   * @return True if bounds collapse, false otherwise.
   */
  virtual bool collapse() override {return false;}

  /**
   * @brief Creates a clone of the BallSampler object.
   * @return A shared pointer to the cloned BallSampler object.
   */
  virtual SamplerPtr clone() override;
};

} //end namespace core
} // end namespace graph
