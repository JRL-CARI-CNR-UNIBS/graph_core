#include <net_path_core/net_path_core.h>

namespace ha_planner {


Eigen::MatrixXd computeRotationMatrix(const std::vector<double>& p1, const std::vector<double>& p2)
{
  assert(p1.size()==p2.size());
  unsigned int dof=p1.size();
  Eigen::MatrixXd rot_matrix(dof,dof);
  rot_matrix.setIdentity();
  const Eigen::Map<const Eigen::VectorXd> x1(p1.data(),dof);
  const Eigen::Map<const Eigen::VectorXd> x2(p2.data(),dof);
  Eigen::VectorXd main_versor =(x1-x2)/(x1-x2).norm();

  bool is_standard_base=false;
  for (unsigned int ic=0;ic<rot_matrix.cols();ic++)
  {
    if (std::abs(main_versor.dot(rot_matrix.col(ic)))>0.999)
    {
      is_standard_base=true;
      // rot_matrix is already orthonormal, put this direction as first
      Eigen::VectorXd tmp=rot_matrix.col(ic);
      rot_matrix.col(ic)=rot_matrix.col(0);
      rot_matrix.col(0)=tmp;
      break;
    }
  }

  if (!is_standard_base)
  {
    rot_matrix.col(0)=main_versor;
    // orthonormalization
    for (unsigned int ic=1;ic<rot_matrix.cols();ic++)
    {
      for (unsigned int il=0;il<ic;il++)
      {
        rot_matrix.col(ic)-= (rot_matrix.col(ic).dot(rot_matrix.col(il)))*rot_matrix.col(il);
      }
      rot_matrix.col(ic)/=rot_matrix.col(ic).norm();
    }
  }
  return rot_matrix;
}


Eigen::VectorXd computeEllipsoid(const std::vector<double> &p1,
                                 const std::vector<double> &p2,
                                 const double &cost)
{
  //  rot_matrix*semiaxes.asDiagonalMatrix()*ball+xcenter
  //  ball is a set of random number in a ball
  //  xcenter is the mean point betwen p1 and p2
  //  semiaxes is the vector of ellipsoid semiaxes
  //  rot_matrix is a n-dimensional rotational matrix orientated as the ellipsoid

  double min_dist=std::sqrt(squareDistance(p1,p2));
  double major_semiaxis;
  double minor_semiaxis;

  if (cost<std::numeric_limits<double>::infinity())
  {
    major_semiaxis=cost*0.5;
    minor_semiaxis=std::sqrt(std::pow(cost,2.0)-std::pow(min_dist,2.0))*0.5;
  }
  else
  {
    major_semiaxis=min_dist*40;
    minor_semiaxis=min_dist*40;
  }

  Eigen::VectorXd semiaxes(p1.size());
  semiaxes.setConstant(minor_semiaxis);
  semiaxes(0)=major_semiaxis;

  return semiaxes;
}


double squareDistance(const std::vector<double> &q1, const std::vector<double> &q2)
{
  assert(q1.size()==q2.size());
  double square_length=0;
  for (size_t idx=0;idx<q1.size();idx++)
  {
    square_length+=std::pow((q1.at(idx)-q2.at(idx)),2.0);
  }
  return square_length;
}

std::vector<std::vector<double>> intermediatePoints(const std::vector<double> &q1, const std::vector<double> &q2, const std::vector<double> &unscaling, const double& distance_step)
{
  assert(distance_step>0);
  std::vector<std::vector<double>> output;
  double distance=std::sqrt(squareDistance(q1,q2));
  if (distance<=distance_step)
    return output;
  unsigned int npnt=std::ceil(distance/distance_step);


  for (unsigned int ipnt=1;ipnt<npnt;ipnt++)
  {
    std::vector<double> q(q1.size());
    for (size_t i_dof=0;i_dof<q1.size();i_dof++)
      q.at(i_dof)=q1.at(i_dof)+unscaling.at(i_dof)*(q2.at(i_dof)-q1.at(i_dof))*((double)ipnt)/((double)npnt);

    output.push_back(q);
  }
  return output;
}

}

