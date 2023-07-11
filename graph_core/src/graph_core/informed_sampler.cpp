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


#include <graph_core/informed_sampler.h>

namespace pathplan
{

Eigen::MatrixXd InformedSampler::computeRotationMatrix(const Eigen::VectorXd& x1, const Eigen::VectorXd& x2)
{
  assert(x1.size() == x2.size());
  unsigned int dof = x1.size();
  Eigen::MatrixXd rot_matrix(dof, dof);
  rot_matrix.setIdentity();
  Eigen::VectorXd main_versor = (x1 - x2) / (x1 - x2).norm();

  bool is_standard_base = false;
  for (unsigned int ic = 0; ic < rot_matrix.cols(); ic++)
  {
    if (std::abs(main_versor.dot(rot_matrix.col(ic))) > 0.999)
    {
      is_standard_base = true;
      // rot_matrix is already orthonormal, put this direction as first
      Eigen::VectorXd tmp = rot_matrix.col(ic);
      rot_matrix.col(ic) = rot_matrix.col(0);
      rot_matrix.col(0) = tmp;
      break;
    }
  }

  if (!is_standard_base)
  {
    rot_matrix.col(0) = main_versor;
    // orthonormalization
    for (unsigned int ic = 1; ic < rot_matrix.cols(); ic++)
    {
      for (unsigned int il = 0; il < ic; il++)
      {
        rot_matrix.col(ic) -= (rot_matrix.col(ic).dot(rot_matrix.col(il))) * rot_matrix.col(il);
      }
      rot_matrix.col(ic) /= rot_matrix.col(ic).norm();
    }
  }
  return rot_matrix;
}

Eigen::VectorXd InformedSampler::sample()
{
  if(inf_cost_)
  {
    return center_bound_ + Eigen::MatrixXd::Random(ndof_, 1).cwiseProduct(bound_width_);
  }
  else
  {
    Eigen::VectorXd ball(ndof_);
    for (int iter = 0; iter < 100; iter++)
    {
      ball.setRandom();
      ball *= std::pow(ud_(gen_), 1.0 / (double)ndof_) / ball.norm();

      Eigen::VectorXd q = (rot_matrix_ * ellipse_axis_.asDiagonal() * ball) + ellipse_center_; //q_scaled

      if (not((q - scaled_start_configuration_).norm() + (q  - scaled_stop_configuration_).norm()) < cost_) //ELIMINA
        throw std::runtime_error("err");

      q = q.cwiseProduct(inv_scale_); //q = q_scaled/scale

      bool in_of_bounds = true;
      for (unsigned int iax = 0; iax < ndof_; iax++)
      {
        if (q(iax) > upper_bound_(iax) || q(iax) < lower_bound_(iax))
        {
          in_of_bounds = false;
          break;
        }
      }
      if (in_of_bounds)
        return q;
    }
    //    ROS_DEBUG_THROTTLE(0.1, "unable to find a feasible point in the ellipse");
    ROS_WARN("unable to find a feasible point in the ellipse");

    return center_bound_ + Eigen::MatrixXd::Random(ndof_, 1).cwiseProduct(bound_width_);
  }
}

bool InformedSampler::inBounds(const Eigen::VectorXd& q)
{
  for (unsigned int iax = 0; iax < ndof_; iax++)
  {
    if (q(iax) > upper_bound_(iax) || q(iax) < lower_bound_(iax))
    {
      return false;
    }
  }
  if (inf_cost_)
    return true;
  else
    return ((q.cwiseProduct(scale_) - scaled_start_configuration_).norm() + (q.cwiseProduct(scale_)  - scaled_stop_configuration_).norm()) < cost_;

}

void InformedSampler::setCost(const double &cost)
{
  cost_ = cost;
  inf_cost_ = cost_ >= std::numeric_limits<double>::infinity();

  if (cost_ < focii_distance_)
  {
    //    ROS_WARN("cost is %f, focci distance is %f", cost_, focii_distance_);
    cost_ = focii_distance_;
    min_radius_ = 0.0;
  }
  else
  {
    min_radius_ = 0.5 * std::sqrt(std::pow(cost_, 2) - std::pow(focii_distance_, 2));
  }
  max_radius_ = 0.5 * cost_;
  ellipse_axis_.setConstant(min_radius_);
  ellipse_axis_(0) = max_radius_;

  if (inf_cost_)
  {
    specific_volume_=std::tgamma(ndof_*0.5+1)/std::pow(M_PI,ndof_*0.5);  // inverse of the volume of unit ball

    for (unsigned int idx=0;idx<ndof_;idx++)
    {
      specific_volume_*=(upper_bound_(idx)-lower_bound_(idx));
    }
  }
  else
    specific_volume_=max_radius_*std::pow(min_radius_,ndof_-1);

  if (specific_volume_>0.0)
    specific_volume_=std::pow(specific_volume_,1./ndof_);
}

double InformedSampler::getSpecificVolume()
{
  return specific_volume_;
}

}
