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


#include <dirrt_star/time_planner.h>



namespace pathplan {
namespace dirrt_star {

TimeBasedMultiGoalPlanner::TimeBasedMultiGoalPlanner ( const std::string& name,
                                                       const std::string& group,
                                                       const moveit::core::RobotModelConstPtr& model ) :
  MultigoalPlanner ( name, group, model )
{
  max_velocity_.resize(m_dof);

  COMMENT("read bounds");
  for (unsigned int idx=0;idx<m_dof;idx++)
  {
    COMMENT("joint %s",joint_names_.at(idx).c_str());
    const robot_model::VariableBounds& bounds = robot_model_->getVariableBounds(joint_names_.at(idx));
    if (bounds.position_bounded_)
    {
      max_velocity_(idx)=bounds.max_velocity_;
    }
  }


  if (!m_nh.getParam("norm2_weigth",nu_))
  {
    ROS_DEBUG("norm2_weigth is not set, default=1e-2");
    nu_=1e-2;
  }
  metrics_=std::make_shared<pathplan::TimeBasedMetrics>(max_velocity_,nu_);
}




void TimeBasedMultiGoalPlanner::setSampler(pathplan::SamplerPtr& sampler,
                                          pathplan::TreeSolverPtr& solver)
{

  sampler = std::make_shared<pathplan::TimeBasedInformedSampler>(m_lb, m_ub, m_lb, m_ub,max_velocity_);
  solver=std::make_shared<pathplan::TimeMultigoalSolver>(metrics_, checker, sampler,max_velocity_);
}


}  // namespace dirrt_star
}  // namespace pathplan
