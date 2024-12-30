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

#include <cnr_class_loader/class_loader.hpp>
#include <graph_core/metrics/null_goal_cost_function.h>
#include <graph_core/plugins/metrics/goal_cost_function_base_plugin.h>

namespace graph
{
namespace core
{
/**
 * @class NullGoalCostFunctionPlugin
 * @brief This class implements a wrapper to graph::core::NullGoalCostFunction
 * to allow its plugin to be defined. The class can be loaded as a plugin and
 * builds a graph::core::NullGoalCostFunction object.
 */
class NullGoalCostFunctionPlugin : public GoalCostFunctionBasePlugin
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * @brief Empty constructor for NullGoalCostFunctionPlugin. The function
   * init() must be called afterwards.
   */
  NullGoalCostFunctionPlugin()
  {
    goal_cost_fcn_ = nullptr;
  }

  /**
   * @brief Destructor for NullGoalCostFunctionPlugin.
   */
  virtual ~NullGoalCostFunctionPlugin()
  {
    goal_cost_fcn_ = nullptr;
  }

  /**
   * @brief init Initialise the graph::core::NullGoalCostFunction object,
   * defining its main attributes.
   * @param param_ns defines the namespace under which parameter are searched
   * for using cnr_param library.
   * @param logger Pointer to a TraceLogger for logging.
   * @return True if correctly initialised, False if already initialised.
   */
  virtual bool init(const std::string& param_ns, const cnr_logger::TraceLoggerPtr& logger);
};

}  // namespace core
}  // namespace graph
