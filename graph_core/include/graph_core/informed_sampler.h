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

/**
 * @class InformedSampler
 * @brief Sampler with informed sampling strategy for path planning.
 *
 * The InformedSampler class inherits from SamplerBase and implements an informed
 * sampling strategy for path planning. It samples configurations in a ellipse-shaped
 * to guide the search towards the goal configuration efficiently.
 */
class InformedSampler;
typedef std::shared_ptr<InformedSampler> InformedSamplerPtr;

class InformedSampler: public SamplerBase
{
protected:

  /**
   * @brief scale_ A vector containing a scaling for each element of a configuration: scale.*configuration
   */
  Eigen::VectorXd scale_;

  /**
   * @brief inv_scale_ Inverse of the scaling factors.
   */
  Eigen::VectorXd inv_scale_;

  /**
   * @brief center_bound_ Center of the informed bounds.
   */
  Eigen::VectorXd center_bound_;

  /**
   * @brief bound_width_ Width of the informed bounds.
   */
  Eigen::VectorXd bound_width_;

  /**
   * @brief ellipse_center_ Center of the ellipse-shaped region.
   */
  Eigen::VectorXd ellipse_center_;

  /**
   * @brief ellipse_axis_ Axis lengths of the ellipse-shaped region.
   */
  Eigen::VectorXd ellipse_axis_;

  /**
   * @brief max_radius_ Maximum radius of the ellipse.
   */
  double max_radius_;

  /**
   * @brief min_radius_ Minimum radius of the ellipse.
   */
  double min_radius_;

  /**
   * @brief rot_matrix_ Rotation matrix for ellipse orientation.
   */
  Eigen::MatrixXd rot_matrix_;

  /**
   * @brief focii_distance_ Distance between ellipse focii.
   */
  double focii_distance_;

  /**
   * @brief inf_cost_ Flag indicating infinite cost.
   */
  bool inf_cost_;

  /**
   * @brief Compute the rotation matrix for the ellipse.
   * @param x1 Start configuration.
   * @param x2 Stop configuration.
   * @return Rotation matrix.
   */
  Eigen::MatrixXd computeRotationMatrix(const Eigen::VectorXd& x1, const Eigen::VectorXd&  x2);

  /**
   * @brief Initialize the informed sampler parameters.
   */
  virtual void init();

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * @brief Constructor for InformedSampler.
   * @param start_configuration focus1 for the ellipse.
   * @param stop_configuration focus2 for the ellipse.
   * @param lower_bound Lower bounds for each dimension.
   * @param upper_bound Upper bounds for each dimension.
   * @param scale Scaling factors for each dimension (default: 1).
   * @param logger TraceLogger for logging.
   * @param cost Cost of the path (default: infinity).
   */
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

  /**
   * @brief Generate a sampled configuration.
   * @return Sampled configuration.
   */
  virtual Eigen::VectorXd sample();

  /**
   * @brief Set the cost for the informed sampler.
   * @param cost Cost of the path.
   */
  void setCost(const double& cost);

  /**
   * @brief Set the scaling factors for the bounds.
   * @param scale Scaling factors for each dimension.
   */
  void setScale(const Eigen::VectorXd& scale)
  {
    scale_ = scale;
    init();
  }

  /**
   * @brief Check if a configuration is within the bounds.
   * @param q Configuration to check.
   * @return True if in bounds, false otherwise.
   */
  virtual bool inBounds(const Eigen::VectorXd& q);

  /**
   * @brief Check if the informed bounds collapse (focii distance exceeds cost).
   * @return True if bounds collapse, false otherwise.
   */
  virtual bool collapse()
  {
    return focii_distance_ >= cost_;
  }

  /**
   * @brief Get the distance between ellipse focii.
   * @return Focii distance.
   */
  const double& getFociiDistance(){return focii_distance_;}

  /**
   * @brief Get the specific volume of the informed bounds.
   * @return Specific volume.
   */
  virtual double getSpecificVolume();

  /**
   * @brief Get the lower bounds of the informed sampler in the unscaled space.
   * @return Lower bounds of the informed sampler in the unscaled space.
   */
  const Eigen::VectorXd getLB(){return lower_bound_.cwiseProduct(inv_scale_);}

  /**
   * @brief Get the upper bounds of the informed sampler in the unscaled space.
   * @return Upper bounds of the informed sampler in the unscaled space.
   */
  const Eigen::VectorXd getUB(){return upper_bound_.cwiseProduct(inv_scale_);}

  /**
   * @brief Get the start configuration of the informed sampler in the unscaled space, namely the focus1.
   * @return Start configuration of the informed sampler in the unscaled space.
   */
  const Eigen::VectorXd getStartConf(){return start_configuration_.cwiseProduct(inv_scale_);}

  /**
   * @brief Get the stop configuration of the informed sampler in the unscaled space, namely the focus 2.
   * @return Stop configuration of the informed sampler in the unscaled space.
   */
  const Eigen::VectorXd getStopConf(){return stop_configuration_.cwiseProduct(inv_scale_);}

  /**
   * @brief Get the dimension of the informed sampler.
   * @return Dimension of the informed sampler.
   */
  const unsigned int& getDimension()const {return ndof_;}
};

}  // namespace pathplan
