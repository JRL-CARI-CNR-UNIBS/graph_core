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


#include <graph_core/samplers/ball_sampler.h>


namespace graph
{
namespace core
{
void BallSampler::config()
{
  if(not initialized_)
  {
    CNR_ERROR(logger_,"Ball sampler not initialised, cannot configure");
    return;
  }

  if(cost_ < 0.0)
  {
    CNR_FATAL(logger_,"cost should be >= 0");
    throw std::invalid_argument("cost should be >= 0");
  }

  if(ball_center_.rows() != ndof_)
  {
    CNR_FATAL(logger_,"ball center should have the same size of ndof");
    throw std::invalid_argument("ball center should have the same size of ndof");
  }

  ball_.setRandom(ndof_,1);
}

Eigen::VectorXd BallSampler::sample()
{
  if(cost_ < std::numeric_limits<double>::infinity())
  {
    for (int itrial = 0; itrial < 100; itrial++)
    {
      ball_.setRandom(ndof_,1);
      ball_ *= std::pow(ud_(gen_), 1.0 / (double)ndof_) / ball_.norm();

      Eigen::VectorXd q = cost_ * ball_ + ball_center_;
      if(inBounds(q))
        return q;
    }

    CNR_WARN(logger_,"BallSampler has not found a sample in the ball that respects the bounds");
  }

  // Sample everywhere
  return 0.5 * (lower_bound_ + upper_bound_) +
      Eigen::MatrixXd::Random(ndof_, 1).cwiseProduct(0.5 * (lower_bound_ - upper_bound_));
}

bool BallSampler::inBounds(const Eigen::VectorXd& q)
{
  for (unsigned int iax = 0; iax < ndof_; iax++)
  {
    if (q(iax) > upper_bound_(iax) || q(iax) < lower_bound_(iax))
    {
      return false;
    }
  }
  if (cost_ == std::numeric_limits<double>::infinity())
    return true;
  else
    return (q-ball_center_).norm() < cost_;
}

void BallSampler::setCost(const double& cost)
{
  if(cost_ < 0.0)
  {
    CNR_FATAL(logger_,"cost should be >= 0");
    throw std::invalid_argument("cost should be >= 0");
  }
  cost_ = cost;
}

SamplerPtr BallSampler::clone()
{
  return std::make_shared<BallSampler>(ball_center_,lower_bound_,upper_bound_,logger_,cost_);
}

} //end namespace core
} // end namespace graph
