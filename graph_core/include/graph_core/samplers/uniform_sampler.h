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
 * @class UniformSampler
 * @brief Sampling uniformly in the search space.
 *
 * The UniformSampler class inherits from SamplerBase and samples uniformly
 * in the search space considering the lower and upper bounds.
 */
class UniformSampler;
typedef std::shared_ptr<UniformSampler> UniformSamplerPtr;

class UniformSampler: public SamplerBase
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * @brief Empty constructor for UniformSampler. The function init() must be called afterwards.
   */
  UniformSampler():SamplerBase() //set initialized_ false
  {}

  /**
   * @brief Constructor for UniformSampler.
   * @param lower_bound Lower bounds for each dimension.
   * @param upper_bound Upper bounds for each dimension.
   * @param logger TraceLogger for logging.
   */
  UniformSampler(const Eigen::VectorXd& lower_bound,
                 const Eigen::VectorXd& upper_bound,
                 const cnr_logger::TraceLoggerPtr& logger):
    SamplerBase(lower_bound,upper_bound,logger) //set initialized_ true
  {}

  /**
   * @brief Generate a sampled configuration.
   * @return Sampled configuration.
   */
  virtual Eigen::VectorXd sample() override;

  /**
   * @brief Set the cost associated with the sampler. It has no effect on this sampler.
   *
   * @param cost Cost to be set.
   */
  virtual void setCost(const double& cost) override {cost_ = cost;}

  /**
   * @brief Check if the sampler collapse. It has no effect on this sampler.
   * @return True if bounds collapse, false otherwise.
   */
  virtual bool collapse() override {return false;}

  /**
   * @brief Creates a clone of the UniformSampler object.
   * @return A shared pointer to the cloned UniformSampler object.
   */
  virtual SamplerPtr clone() override;
};

} //end namespace core
} // end namespace graph
