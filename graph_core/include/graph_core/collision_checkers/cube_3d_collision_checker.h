#pragma once

#include "graph_core/collision_checkers/collision_checker_base.h"

namespace graph
{
namespace core
{
/**
 * @class Cube3dCollisionChecker
 * @brief Collision checker for a 3D cube-shaped environment.
 *
 * The Cube3dCollisionChecker class inherits from CollisionCheckerBase and
 * implements a collision checker for a 3D environment represented as a cube.
 * The collision check is performed by verifying if the absolute values of the
 * configuration exceed a threshold, indicating a collision with the cube.
 */

class Cube3dCollisionChecker;
typedef std::shared_ptr<Cube3dCollisionChecker> Cube3dCollisionCheckerPtr;

class Cube3dCollisionChecker : public CollisionCheckerBase
{
public:
  /**
   * @brief If each joint has absolute value grater than 'abs_joint_threshold_',
   * the configuration will be considered collision-free.
   */
  double abs_joint_threshold_;

  /**
   * @brief Constructor for Cube3dCollisionChecker.
   * @param logger Pointer to a TraceLogger for logging.
   * @param abs_joint_threshold is the value each joint must exceed in absolute
   * value for the configuration to be collision-free.
   * @param min_distance Distance between configurations checked for collisions
   * along a connection.
   */
  Cube3dCollisionChecker(const cnr_logger::TraceLoggerPtr& logger, const double& abs_joint_threshold,
                         const double& min_distance)
    : CollisionCheckerBase(logger, min_distance), abs_joint_threshold_(abs_joint_threshold)
  {
  }

  /**
   * @brief Constructor for Cube3dCollisionChecker.
   * @param logger Pointer to a TraceLogger for logging.
   * @param min_distance Distance between configurations checked for collisions
   * along a connection.
   */
  Cube3dCollisionChecker(const cnr_logger::TraceLoggerPtr& logger, const double& min_distance = 0.01)
    : CollisionCheckerBase(logger, min_distance), abs_joint_threshold_(1.0)
  {
  }

  /**
   * @brief Check for collision with the 3D cube.
   * @param configuration The robot configuration to check for collision.
   * @return True if the configuration is in collision, false otherwise.
   */
  virtual bool check(const Eigen::VectorXd& configuration) override
  {
    return configuration.cwiseAbs().maxCoeff() > abs_joint_threshold_;
  }

  /**
   * @brief Clone the collision checker.
   * @return A shared pointer to the cloned collision checker.
   */
  virtual CollisionCheckerPtr clone() override
  {
    return std::make_shared<Cube3dCollisionChecker>(logger_, min_distance_);
  }
};

}  // end namespace core
}  // end namespace graph
