#pragma once
/*
Copyright (c) 2024, Manuel Beschi and Cesare Tonola, JRL-CARI CNR-STIIMA/UNIBS, manuel.beschi@unibs.it, c.tonola001@unibs.it
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

#include <graph_core/solvers/anytime_rrt.h>
#include <graph_core/plugins/solvers/tree_solver_plugin.h>

namespace graph
{
namespace core
{

/**
 * @class AnytimeRRTPlugin
 * @brief This class implements a wrapper to graph::core::AnytimeRRT to allow its plugin to be defined.
 * The class can be loaded as a plugin and builds a graph::core::AnytimeRRT object.
 */

class AnytimeRRTPlugin: public TreeSolverPlugin
{
protected:

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * @brief Empty constructor for AnytimeRRTPlugin. The function AnytimeRRTPlugin::init() must be called afterwards.
   */
  AnytimeRRTPlugin():TreeSolverPlugin()
  {}

  /**
   * @brief init Initialise the object, defining its main attributes.
   * @param param_ns defines the namespace under which parameter are searched for using cnr_param library.
   * @param metrics The metrics used to evaluate paths.
   * @param checker The collision checker for checking collisions.
   * @param sampler The sampler for generating random configurations.
   * @param goal_cost_fcn The function used to assign the cost of the goal. If it is not defined, the default cost function does not assign any cost to the goal.
   * @param logger The logger for logging messages.
   * @return True if correctly initialised, False if already initialised.
   */
  virtual bool init(const std::string& param_ns,
                    const graph::core::MetricsPtr& metrics,
                    const graph::core::CollisionCheckerPtr& checker,
                    const graph::core::SamplerPtr& sampler,
                    const graph::core::GoalCostFunctionPtr& goal_cost_fcn,
                    const cnr_logger::TraceLoggerPtr& logger)
  {
    solver_ = std::make_shared<graph::core::AnytimeRRT>(metrics,checker,sampler,goal_cost_fcn,logger);
    return true;
  }
};

} //namespace core
} //namespace graph
