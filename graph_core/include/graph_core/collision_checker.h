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

#include <ros/ros.h>
#include <eigen3/Eigen/Core>

namespace pathplan {

class CollisionChecker
{
  double min_distance_=0.01;
public:
  CollisionChecker(const double& min_distance=0.01):
    min_distance_(min_distance)
  {

  }

  // collision check: true if it is valid
  virtual bool check(const Eigen::VectorXd& configuration)
  {
    return true;
  }

  bool checkPath(const Eigen::VectorXd &configuration1, const Eigen::VectorXd &configuration2)
  {
    Eigen::VectorXd conf;
    return checkPath(configuration1,configuration2,conf);
  }

  bool checkPath(const Eigen::VectorXd& configuration1,
                         const Eigen::VectorXd& configuration2,
                         Eigen::VectorXd& conf)
  {
    if (!check(configuration1))
      return false;
    if (!check(configuration2))
      return false;

    double dist=(configuration2-configuration1).norm();
    unsigned int npnt=std::ceil(dist/min_distance_);
    if (npnt>1)
    {
      Eigen::VectorXd step=(configuration2-configuration1)/npnt;
      for (unsigned int ipnt=1;ipnt<npnt;ipnt++)
      {
        conf=configuration1+step*ipnt;
        if (!check(conf))
        {
          // return last feasible configuration
          conf=configuration1+step*(ipnt-1);
          return false;
        }
      }
    }
    return true;
  }
};

typedef std::shared_ptr<CollisionChecker> CollisionCheckerPtr;

class Cube3dCollisionChecker: public CollisionChecker
{
public:
  virtual bool check(const Eigen::VectorXd& configuration)
  {
    return configuration.cwiseAbs().maxCoeff()>1;
  }
};


}
