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

#include <graph_core/sampler_base.h>

#include <eigen3/Eigen/Core>
#include <graph_core/util.h>
#include <random>

namespace pathplan
{

class InformedSampler;
typedef std::shared_ptr<InformedSampler> InformedSamplerPtr;

class InformedSampler: public SamplerBase
{
protected:
  Eigen::VectorXd scale_;
  Eigen::VectorXd inv_scale_;
  Eigen::VectorXd center_bound_;
  Eigen::VectorXd bound_width_;

  Eigen::VectorXd ellipse_center_;
  Eigen::VectorXd ellipse_axis_;
  double max_radius_;
  double min_radius_;
  Eigen::MatrixXd rot_matrix_;
  double focii_distance_;
  bool inf_cost_;

  Eigen::MatrixXd computeRotationMatrix(const Eigen::VectorXd& x1, const Eigen::VectorXd&  x2);
  virtual void init();

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  InformedSampler(const Eigen::VectorXd& start_configuration,
                  const Eigen::VectorXd& stop_configuration,
                  const Eigen::VectorXd& lower_bound,
                  const Eigen::VectorXd& upper_bound,
                  const cnr_logger::TraceLoggerPtr& logger,
                  const double& cost = std::numeric_limits<double>::infinity()):
    SamplerBase(start_configuration,
                stop_configuration,
                lower_bound,
                upper_bound,
                logger,
                cost)
  {
    scale_.setOnes(lower_bound_.rows(),1);
    init();
  }

  InformedSampler(const Eigen::VectorXd& start_configuration,
                  const Eigen::VectorXd& stop_configuration,
                  const Eigen::VectorXd& lower_bound,
                  const Eigen::VectorXd& upper_bound,
                  const Eigen::VectorXd& scale,
                  const cnr_logger::TraceLoggerPtr& logger,
                  const double& cost):
    SamplerBase(start_configuration,
                stop_configuration,
                lower_bound,
                upper_bound,
                logger,
                cost),
    scale_(scale)
  {
    init();
  }

  virtual Eigen::VectorXd sample();
  void setCost(const double& cost);

  void setScale(const Eigen::VectorXd& scale)
  {
    scale_ = scale;
    init();
  }

  virtual bool inBounds(const Eigen::VectorXd& q);
  virtual bool collapse()
  {
    return focii_distance_ >= cost_;
  }

  const double& getFociiDistance(){return focii_distance_;}
  const double& getCost(){return cost_;}

  virtual double getSpecificVolume();

  virtual void sampleImproved(){}

  const Eigen::VectorXd getLB(){return lower_bound_.cwiseProduct(inv_scale_);}
  const Eigen::VectorXd getUB(){return upper_bound_.cwiseProduct(inv_scale_);}

  const Eigen::VectorXd getStartConf(){return start_configuration_.cwiseProduct(inv_scale_);}
  const Eigen::VectorXd getStopConf(){return stop_configuration_.cwiseProduct(inv_scale_);}

  const unsigned int& getDimension()const {return ndof_;}
};

}  // namespace pathplan
