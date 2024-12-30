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

#include <graph_core/plugins/samplers/sampler_base_plugin.h>
#include <graph_core/samplers/uniform_sampler.h>

namespace graph
{
namespace core
{
/**
 * @class UniformSamplerPlugin
 * @brief This class implements a wrapper to graph::core::UniformSampler to
 * allow its plugin to be defined. The class can be loaded as a plugin and
 * builds a graph::core::UniformSampler object.
 */
class UniformSamplerPlugin : public SamplerBasePlugin
{
protected:
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * @brief Empty constructor for UniformSamplerPlugin. The function init() must
   * be called afterwards.
   */
  UniformSamplerPlugin() : SamplerBasePlugin()
  {
  }

  /**
   * @brief init Initialise the object graph::core::UniformSampler, defining its
   * main attributes.
   * @param param_ns defines the namespace under which parameter are searched
   * for using cnr_param library.
   * @param focus_1 discarded, it just has no effects.
   * @param focus_2 discarded, it just has no effects.
   * @param lower_bound Lower bounds for each dimension.
   * @param upper_bound Upper bounds for each dimension.
   * @param scale Scaling factors for each dimension, no effects for this class.
   * @param logger TraceLogger for logging.
   * @param cost discarded, it just has no effects.
   * @return True if correctly initialised, False if already initialised.
   */
  virtual bool init(const std::string& param_ns, const Eigen::VectorXd& focus_1, const Eigen::VectorXd& focus_2,
                    const Eigen::VectorXd& lower_bound, const Eigen::VectorXd& upper_bound,
                    const Eigen::VectorXd& scale, const cnr_logger::TraceLoggerPtr& logger,
                    const double& cost = std::numeric_limits<double>::infinity()) override
  {
    (void)param_ns;
    (void)focus_2;
    (void)scale;
    sampler_ = std::make_shared<UniformSampler>(lower_bound, upper_bound, logger);
    return true;
  }
};

}  // namespace core
}  // namespace graph
