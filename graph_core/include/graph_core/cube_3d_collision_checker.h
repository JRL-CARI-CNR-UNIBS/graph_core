#ifndef CUBE_3D_COLLISION_CHECKER_H
#define CUBE_3D_COLLISION_CHECKER_H

#include "graph_core/collision_checker_base.h"

/**
 * @class Cube3dCollisionChecker
 * @brief Collision checker for a 3D cube-shaped environment.
 *
 * The Cube3dCollisionChecker class inherits from CollisionCheckerBase and
 * implements a collision checker for a 3D environment represented as a cube.
 * The collision check is performed by verifying if the absolute values of the
 * configuration exceed a threshold, indicating a collision with the cube.
 */
class Cube3dCollisionChecker: public CollisionCheckerBase
{
public:

  /**
   * @brief Check for collision with the 3D cube.
   * @param configuration The robot configuration to check for collision.
   * @return True if the configuration is in collision, false otherwise.
   */
  virtual bool check(const Eigen::VectorXd& configuration)
  {
    return configuration.cwiseAbs().maxCoeff() > 1;
  }
};
#endif // CUBE_3D_COLLISION_CHECKER_H
