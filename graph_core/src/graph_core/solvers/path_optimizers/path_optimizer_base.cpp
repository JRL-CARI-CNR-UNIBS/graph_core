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

PathOptimizerBase::PathOptimizerBase(const CollisionCheckerPtr &checker,
                                       const MetricsPtr &metrics,
                                       const cnr_logger::TraceLoggerPtr &logger):
  checker_(checker),
  metrics_(metrics),
  logger_(logger)
{
  solved_ = false;
  configured_ = false;
}

void PathOptimizerBase::config(const std::string& param_ns)
{
  param_ns_ = param_ns;
  get_param(logger_,param_ns_,"max_stall_gen",max_stall_gen_,(unsigned int) 10);

  stall_gen_ = 0;
  configured_ = true;
}

void PathOptimizerBase::setPath(const PathPtr &path)
{
  if(not path)
  {
    CNR_WARN(logger_,"path is null");
    return;
  }

  solved_ = false;
  stall_gen_ = 0;
  path_ = path;
}

PathPtr PathOptimizerBase::getPath()
{
  return path_;
}

bool PathOptimizerBase::solve(const unsigned int &max_iteration, const double& max_time)
{
  if(not configured_)
  {
    CNR_ERROR(logger_,"Path local solver not configured!");
    return false;
  }

  auto tic = graph_time::now();
  if(max_time<=0.0)
    return false;

  unsigned int iter = 0;
  while (iter++ < max_iteration)
  {
    if (solved_)
    {
      CNR_DEBUG(logger_,"solved in %u iterations", iter);
      return true;
    }
    step();

    auto now = graph_time::now();
    graph_duration difference = now - tic;
    if(difference.count() >= 0.98*max_time)
      break;
  }
  return solved_;
}

} //end namespace core
} // end namespace graph
