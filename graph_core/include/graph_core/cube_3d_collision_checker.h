#ifndef CUBE_3D_COLLISION_CHECKER_H
#define CUBE_3D_COLLISION_CHECKER_H

#include "graph_core/collision_checker_base.h"

class Cube3dCollisionChecker: public CollisionCheckerBase
{
public:
  virtual bool check(const Eigen::VectorXd& configuration)
  {
    return configuration.cwiseAbs().maxCoeff() > 1;
  }
};
#endif // CUBE_3D_COLLISION_CHECKER_H
