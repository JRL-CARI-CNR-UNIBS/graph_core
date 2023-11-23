#pragma once
/*
Copyright (c) 2023, Manuel Beschi UNIBS manuel.beschi@unibs.it
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
#include <xmlrpcpp/XmlRpcValue.h>
#include <random>

namespace pathplan
{



class StateInformationBase;
typedef std::shared_ptr<StateInformationBase> StateInformationPtr;

class StateInformationBase: public std::enable_shared_from_this<StateInformationBase>
{
protected:
  Eigen::VectorXd lower_bound_;
  Eigen::VectorXd upper_bound_;
  Eigen::VectorXd scales_;
  std::vector<std::string> names_;
  std::vector<StateSpacePtr> state_spaces_;
  std::map<std::string,StateSpacePtr> state_spaces_map;
  unsigned int ndof_;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  StateInformationBase();
  virtual bool config(const XmlRpc::XmlRpcValue& config)=0;
  virtual bool setLB(const Eigen::VectorXd& lower_bound){lower_bound_=lower_bound;}
  virtual bool setUB(const Eigen::VectorXd& upper_bound){upper_bound_=upper_bound;}
  const std::vector<std::string>& getNames(){return names_};
  const std::vector<StateSpacePtr>& getStateSpace(){return state_spaces_;}
  const std::map<std::string,StateSpacePtr>& getStateSpaceMap(){return state_spaces_map;}
  const Eigen::VectorXd& getLB(){return lower_bound_;}
  const Eigen::VectorXd& getUB(){return upper_bound_;}
  const unsigned int& getDimension()const {return ndof_;}
};



}  // namespace pathplan
