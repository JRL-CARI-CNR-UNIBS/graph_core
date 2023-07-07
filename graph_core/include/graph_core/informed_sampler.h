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

class InformedSampler;
typedef std::shared_ptr<InformedSampler> InformedSamplerPtr;

class InformedSampler: public std::enable_shared_from_this<InformedSampler>
{
protected:
  Eigen::VectorXd start_configuration_;
  Eigen::VectorXd stop_configuration_;
  Eigen::VectorXd scaled_start_configuration_;
  Eigen::VectorXd scaled_stop_configuration_;
  Eigen::VectorXd scale_;
  Eigen::VectorXd inv_scale_;
  Eigen::VectorXd lower_bound_;
  Eigen::VectorXd upper_bound_;
  Eigen::VectorXd center_bound_;
  Eigen::VectorXd bound_width_;

  double cost_;
  unsigned int ndof_;

  Eigen::VectorXd ellipse_center_;
  Eigen::VectorXd ellipse_axis_;
  double max_radius_;
  double min_radius_;
  Eigen::MatrixXd rot_matrix_;
  double focii_distance_;
  bool inf_cost_;
  double specific_volume_; // ndof-th root of volume of the hyperellipsoid divided by the volume of unit sphere

  std::random_device rd_;
  std::mt19937 gen_;
  std::uniform_real_distribution<double> ud_;

  Eigen::MatrixXd computeRotationMatrix(const Eigen::VectorXd& x1, const Eigen::VectorXd&  x2);

  virtual void init()
  {
    if(cost_ < 0.0)
      throw std::invalid_argument("cost should be >= 0");

    if(start_configuration_.rows() != ndof_)
      throw std::invalid_argument("start_configuration_ should have the same size of ndof");
    if(stop_configuration_.rows() != ndof_)
      throw std::invalid_argument("stop_configuration_ should have the same size of ndof");
    if(upper_bound_.rows() != ndof_)
      throw std::invalid_argument("upper_bound_ should have the same size of ndof");
    if(lower_bound_.rows() != ndof_)
      throw std::invalid_argument("lower_bound_ should have the same size of ndof");
    if(scale_.rows() != ndof_)
      throw std::invalid_argument("scale should have the same size of ndof");

    ud_ = std::uniform_real_distribution<double>(0, 1);

    ndof_ = lower_bound_.rows();

    inv_scale_=scale_.cwiseInverse();

    scaled_start_configuration_ = start_configuration_.cwiseProduct(scale_);
    scaled_stop_configuration_  = stop_configuration_ .cwiseProduct(scale_);

    ellipse_center_ = 0.5 * (scaled_start_configuration_ + scaled_stop_configuration_);
    focii_distance_ = (scaled_start_configuration_ - scaled_stop_configuration_).norm();
    center_bound_ = 0.5 * (lower_bound_ + upper_bound_);
    bound_width_ = 0.5 * (lower_bound_ - upper_bound_);
    ellipse_axis_.resize(ndof_);

    rot_matrix_ = computeRotationMatrix(scaled_start_configuration_, scaled_stop_configuration_);

    ROS_DEBUG_STREAM("rot_matrix_:\n" << rot_matrix_);
    ROS_DEBUG_STREAM("ellipse center" << ellipse_center_.transpose());
    ROS_DEBUG_STREAM("focii_distance_" << focii_distance_);
    ROS_DEBUG_STREAM("center_bound_" << center_bound_.transpose());
    ROS_DEBUG_STREAM("bound_width_" << bound_width_.transpose());

    if (cost_ < std::numeric_limits<double>::infinity())
    {
      inf_cost_ = false;
      setCost(cost_);
    }
    else
      inf_cost_ = true;
  }

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  InformedSampler(const Eigen::VectorXd& start_configuration,
                  const Eigen::VectorXd& stop_configuration,
                  const Eigen::VectorXd& lower_bound,
                  const Eigen::VectorXd& upper_bound,
                  const double& cost = std::numeric_limits<double>::infinity()):
    start_configuration_(start_configuration),
    stop_configuration_(stop_configuration),
    lower_bound_(lower_bound),
    upper_bound_(upper_bound),
    cost_(cost),
    gen_{rd_()}//gen_(time(0))
  {
    scale_.setOnes(lower_bound_.rows(),1);

    init();
  }

  InformedSampler(const Eigen::VectorXd& start_configuration,
                  const Eigen::VectorXd& stop_configuration,
                  const Eigen::VectorXd& lower_bound,
                  const Eigen::VectorXd& upper_bound,
                  const Eigen::VectorXd& scale,
                  const double& cost):
    start_configuration_(start_configuration),
    stop_configuration_(stop_configuration),
    lower_bound_(lower_bound),
    upper_bound_(upper_bound),
    scale_(scale),
    cost_(cost),
    gen_{rd_()}//gen_(time(0))
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

  const Eigen::VectorXd& getLB(){return lower_bound_;}
  const Eigen::VectorXd& getUB(){return upper_bound_;}

  const Eigen::VectorXd& getStartConf(){return start_configuration_;}
  const Eigen::VectorXd& getStopConf(){return stop_configuration_;}
  const Eigen::VectorXd& getScaledStartConf(){return scaled_start_configuration_;}
  const Eigen::VectorXd& getScaledStopConf(){return scaled_stop_configuration_;}

  const unsigned int& getDimension()const {return ndof_;}
};


typedef std::shared_ptr<InformedSampler> SamplerPtr;

}  // namespace pathplan
