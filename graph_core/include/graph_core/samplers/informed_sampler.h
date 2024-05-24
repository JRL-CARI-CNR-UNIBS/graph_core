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

#include <graph_core/samplers/sampler_base.h>

#include <eigen3/Eigen/Core>
#include <graph_core/util.h>
#include <random>

namespace graph
{
namespace core
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
   * @brief focus_1_ Focus 1 of the ellipse.
   */
  Eigen::VectorXd focus_1_;

  /**
   * @brief focus_2_ Focus 2 of the ellipse.
   */
  Eigen::VectorXd focus_2_;

  /**
   * @brief focus_1_not_scaled_ Focus 1 of the ellipse not scaled.
   */
  Eigen::VectorXd focus_1_not_scaled_;

  /**
   * @brief focus_2_not_scaled_ Focus 2 of the ellipse not scaled.
   */
  Eigen::VectorXd focus_2_not_scaled_;

  /**
   * @brief lower_bound_not_scaled_ Lower bounds not scaled for configuration sampling.
   */
  Eigen::VectorXd lower_bound_not_scaled_;

  /**
   * @brief upper_bound_not_scaled_ Upper bounds not scaled for configuration sampling.
   */
  Eigen::VectorXd upper_bound_not_scaled_;

  /**
   * @brief scale_ A vector containing a scaling for each element of a configuration.
   * Each configuration in the scaled space is computed as the component-wise product between the configuration
   * and scale_.
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
   * @param x1 Configuration 1.
   * @param x2 Configuration 2.
   * @return Rotation matrix.
   */
  Eigen::MatrixXd computeRotationMatrix(const Eigen::VectorXd& x1, const Eigen::VectorXd&  x2);

  /**
   * @brief Configure the informed sampler parameters.
   */
  virtual void config();

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * @brief Empty constructor for InformedSampler. The function init() must be called afterwards.
   */
  InformedSampler():SamplerBase() //set initialized_ false
  {}

  /**
   * @brief Constructor for InformedSampler.
   * @param focus_1 focus 1 for the ellipse.
   * @param focus_2 focus 2 for the ellipse.
   * @param lower_bound Lower bounds for each dimension.
   * @param upper_bound Upper bounds for each dimension.
   * @param scale Scaling factors for each dimension (default: 1).
   * @param logger TraceLogger for logging.
   * @param cost Cost of the path (default: infinity).
   */
  InformedSampler(const Eigen::VectorXd& focus_1,
                  const Eigen::VectorXd& focus_2,
                  const Eigen::VectorXd& lower_bound,
                  const Eigen::VectorXd& upper_bound,
                  const Eigen::VectorXd& scale,
                  const cnr_logger::TraceLoggerPtr& logger,
                  const double& cost):
    SamplerBase(lower_bound,
                upper_bound,
                logger,
                cost), //set initialized_ true
    //at this point lower_bound, upper_bound, focus_1, focus_2 are not scaled yet, they will be scaled and
    //assigned to lower_bound_, upper_bound_, focus_1_, focus_2_ in config()
    focus_1_not_scaled_(focus_1),focus_2_not_scaled_(focus_2),
    lower_bound_not_scaled_(lower_bound),upper_bound_not_scaled_(upper_bound),
    scale_(scale)
  {
    config();
  }
  InformedSampler(const Eigen::VectorXd& focus_1,
                  const Eigen::VectorXd& focus_2,
                  const Eigen::VectorXd& lower_bound,
                  const Eigen::VectorXd& upper_bound,
                  const cnr_logger::TraceLoggerPtr& logger,
                  const double& cost = std::numeric_limits<double>::infinity()):
    SamplerBase(lower_bound,
                upper_bound,
                logger,
                cost), //set initialized_ true
    //at this point lower_bound, upper_bound, focus_1, focus_2 are not scaled yet, they will be scaled and
    //assigned to lower_bound_, upper_bound_, focus_1_, focus_2_ in config()
    focus_1_not_scaled_(focus_1),focus_2_not_scaled_(focus_2),
    lower_bound_not_scaled_(lower_bound),upper_bound_not_scaled_(upper_bound)
  {
    scale_.setOnes(lower_bound_.rows(),1);
    config();
  }

  /**
   * @brief pluginInit  This function should be called just after the plugin is loaded and initialise the graph::core::InformedSampler object, defining its main attributes.
   * @param focus_1 focus 1 for the ellipse.
   * @param focus_2 focus 2 for the ellipse.
   * @param lower_bound Lower bounds for each dimension.
   * @param upper_bound Upper bounds for each dimension.
   * @param scale Scaling factors for each dimension (default: 1).
   * @param logger TraceLogger for logging.
   * @param cost Cost of the path (default: infinity).
   * @return True if correctly initialised, False if already initialised.
   */
  virtual bool init(const Eigen::VectorXd& focus_1,
                    const Eigen::VectorXd& focus_2,
                    const Eigen::VectorXd& lower_bound,
                    const Eigen::VectorXd& upper_bound,
                    const Eigen::VectorXd& scale,
                    const cnr_logger::TraceLoggerPtr& logger,
                    const double& cost = std::numeric_limits<double>::infinity())
  {
    if(not SamplerBase::init(lower_bound,upper_bound,logger,cost))
      return false;

    focus_1_ = focus_1;
    focus_2_ = focus_2;
    scale_ = scale;

    config();

    return true;
  }

  virtual bool init(const Eigen::VectorXd& focus_1,
                    const Eigen::VectorXd& focus_2,
                    const Eigen::VectorXd& lower_bound,
                    const Eigen::VectorXd& upper_bound,
                    const cnr_logger::TraceLoggerPtr& logger,
                    const double& cost = std::numeric_limits<double>::infinity())
  {
    if(not SamplerBase::init(lower_bound,upper_bound,logger,cost))
      return false;

    focus_1_ = focus_1;
    focus_2_ = focus_2;
    scale_.setOnes(lower_bound_.rows(),1);

    config();

    return true;
  }

  /**
   * @brief Generate a sampled configuration.
   * @return Sampled configuration.
   */
  virtual Eigen::VectorXd sample() override;

  /**
   * @brief Set the cost for the informed sampler.
   * @param cost Cost of the path.
   */
  void setCost(const double& cost) override;

  /**
   * @brief Set the scaling factors for the bounds.
   * @param scale Scaling factors for each dimension.
   */
  void setScale(const Eigen::VectorXd& scale)
  {
    scale_ = scale;
    config();
  }

  /**
   * @brief Check if a configuration is within the bounds.
   * @param q Configuration to check.
   * @return True if in bounds, false otherwise.
   */
  virtual bool inBounds(const Eigen::VectorXd& q) override;

  /**
   * @brief Check if the informed bounds collapse (focii distance exceeds cost).
   * @return True if bounds collapse, false otherwise.
   */
  virtual bool collapse() override
  {
    return focii_distance_ >= cost_;
  }

  /**
   * @brief Get the distance between ellipse focii.
   * @return Focii distance.
   */
  const double& getFociiDistance(){return focii_distance_;}

  /**
   * @brief Get the lower bounds of the informed sampler in the unscaled space.
   * @return Lower bounds of the informed sampler in the unscaled space.
   */
  const Eigen::VectorXd& getLB() override {return lower_bound_not_scaled_;}

  /**
   * @brief Get the upper bounds of the informed sampler in the unscaled space.
   * @return Upper bounds of the informed sampler in the unscaled space.
   */
  const Eigen::VectorXd& getUB() override {return upper_bound_not_scaled_;}

  /**
   * @brief Get the focus 1 of the informed sampler in the unscaled space.
   * @return Focus 1 of the informed sampler in the unscaled space.
   */
  const Eigen::VectorXd& getFocus1(){return focus_1_not_scaled_;}

  /**
   * @brief Get the focus 2 of the informed sampler in the unscaled space.
   * @return Focus 2 of the informed sampler in the unscaled space.
   */
  const Eigen::VectorXd& getFocus2(){return focus_2_not_scaled_;}

  /**
   * @brief Creates a clone of the InformedSampler object.
   * @return A shared pointer to the cloned InformedSampler object.
   */
  virtual SamplerPtr clone() override;
};

} //end namespace core
} // end namespace graph
