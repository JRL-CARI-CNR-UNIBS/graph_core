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


#include <graph_core/collision_checker.h>
#include <moveit/planning_scene/planning_scene.h>

namespace pathplan
{

class NarrowPassChecker: public CollisionChecker
{
protected:
  double hole_radius_;
  double cilinder_radius_;
  double cilinder_width_;
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  NarrowPassChecker(const double& hole_radius, const double& cilinder_radius, const double cilinder_width):
    hole_radius_(hole_radius),
    cilinder_radius_(cilinder_radius),
    cilinder_width_(cilinder_width)
  {
  }

  virtual bool check(const Eigen::VectorXd& configuration)
  {
    if (std::abs(configuration(0))>=cilinder_width_*0.5)
      return true;

    Eigen::VectorXd sub_conf=configuration.tail(configuration.size()-1);
    if (sub_conf.norm()>=cilinder_radius_)
      return true;

//    sub_conf(1)-=2*hole_radius_;
    double x=sub_conf.norm();
    return (x<=hole_radius_);
  }

};
}
