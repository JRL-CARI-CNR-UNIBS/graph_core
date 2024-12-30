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

#include <cnr_class_loader/register_macro.hpp>
#include <graph_core/metrics/metrics_base.h>

namespace graph
{
namespace core
{
/**
 * @class MetricsBasePlugin
 * @brief This class implements a wrapper to graph::core::MetricsBase to allow
 * its plugin to be defined. The class can be loaded as a plugin and builds a
 * graph::core::MetricsBase object.
 */
class MetricsBasePlugin;
typedef std::shared_ptr<MetricsBasePlugin> MetricsPluginPtr;

class MetricsBasePlugin : public std::enable_shared_from_this<MetricsBasePlugin>
{
protected:
  /**
   * @brief metrics_ is the graph::core::MetricsBase object built and
   * initialized by this plugin class.
   */
  graph::core::MetricsPtr metrics_;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * @brief Empty constructor for MetricsBasePlugin. The function init() must be
   * called afterwards.
   */
  MetricsBasePlugin()
  {
    metrics_ = nullptr;
  }

  /**
   * @brief Destructor for MetricsBasePlugin.
   */
  virtual ~MetricsBasePlugin()
  {
    metrics_ = nullptr;
  }

  /**
   * @brief getMetrics return the graph::core::MetricsPtr object built by the
   * plugin.
   * @return the graph::core::MetricsPtr object built.
   */
  virtual graph::core::MetricsPtr getMetrics()
  {
    return metrics_;
  }

  /**
   * @brief init Initialise the graph::core::MetricsBase object, defining its
   * main attributes.
   * @param param_ns defines the namespace under which parameter are searched
   * for using cnr_param library.
   * @param logger Pointer to a TraceLogger for logging.
   * @return True if correctly initialised, False if already initialised.
   */
  virtual bool init(const std::string& param_ns, const cnr_logger::TraceLoggerPtr& logger) = 0;
};

}  // namespace core
}  // namespace graph
