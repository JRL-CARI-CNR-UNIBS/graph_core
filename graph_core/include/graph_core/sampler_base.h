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

#include <eigen3/Eigen/Core>
#include <graph_core/util.h>
#include <ros/ros.h>
#include <random>

namespace pathplan
{

class Sampler;
typedef std::shared_ptr<Sampler> SamplerPtr;

class Sampler: public std::enable_shared_from_this<Sampler>
{
protected:
  Eigen::VectorXd lower_bound_;
  Eigen::VectorXd upper_bound_;
  unsigned int ndof_;
  double specific_volume_; // ndof-th root of volume of the hyperellipsoid divided by the volume of unit sphere

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Sampler();
  virtual bool config()=0;

  virtual Eigen::VectorXd sample()=0;
  virtual bool inBounds(const Eigen::VectorXd& q)
  {
    for (unsigned int iax = 0; iax < ndof_; iax++)
    {
      if (q(iax) > upper_bound_(iax) || q(iax) < lower_bound_(iax))
      {
        return false;
      }
    }
    return true;
  }

  virtual bool collapse()=0;
  virtual double getSpecificVolume()=0;
  const Eigen::VectorXd& getLB(){return lower_bound_;}
  const Eigen::VectorXd& getUB(){return upper_bound_;}
  const unsigned int& getDimension()const {return ndof_;}
};



}  // namespace pathplan
