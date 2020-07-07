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
#include <graph_core/util.h>

namespace pathplan
{


Eigen::MatrixXd computeRotationMatrix(const Eigen::VectorXd& x1, const Eigen::VectorXd& x2)
{
  assert(x1.size() == x2.size());
  unsigned int dof = x1.size();
  Eigen::MatrixXd rot_matrix(dof, dof);
  rot_matrix.setIdentity();
  Eigen::VectorXd main_versor = (x1 - x2) / (x1 - x2).norm();

  bool is_standard_base = false;
  for (unsigned int ic = 0; ic < rot_matrix.cols(); ic++)
  {
    if (std::abs(main_versor.dot(rot_matrix.col(ic))) > 0.999)
    {
      is_standard_base = true;
      // rot_matrix is already orthonormal, put this direction as first
      Eigen::VectorXd tmp = rot_matrix.col(ic);
      rot_matrix.col(ic) = rot_matrix.col(0);
      rot_matrix.col(0) = tmp;
      break;
    }
  }

  if (!is_standard_base)
  {
    rot_matrix.col(0) = main_versor;
    // orthonormalization
    for (unsigned int ic = 1; ic < rot_matrix.cols(); ic++)
    {
      for (unsigned int il = 0; il < ic; il++)
      {
        rot_matrix.col(ic) -= (rot_matrix.col(ic).dot(rot_matrix.col(il))) * rot_matrix.col(il);
      }
      rot_matrix.col(ic) /= rot_matrix.col(ic).norm();
    }
  }
  return rot_matrix;
}

}
