/*
Copyright (c) 2020, JRL-CARI CNR-STIIMA/UNIBS
Manuel Beschi manuel.beschi@unibs.it
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

PSEUDO CODE :
- https://www.cs.cmu.edu/~ckingsf/bioinfo-lectures/kdtrees.pdf
- http://andrewd.ces.clemson.edu/courses/cpsc805/references/nearest_search.pdf
*/
#pragma once
#include <graph_core/graph/node.h>
namespace pathplan
{


class NearestNeighbors: public std::enable_shared_from_this<NearestNeighbors>
{
public:
  NearestNeighbors()
  {
    delete_nodes_=0;
    size_=0;
  }

  virtual void insert(const NodePtr& node)=0;
  virtual void nearestNeighbor(const Eigen::VectorXd& configuration,
                          NodePtr &best,
                          double &best_distance)=0;

  virtual NodePtr nearestNeighbor(const Eigen::VectorXd& configuration)
  {
    {
      double best_distance=std::numeric_limits<double>::infinity();
      NodePtr best;
      nearestNeighbor(configuration,best,best_distance);
      return best;
    }
  }

  virtual std::map<double, NodePtr> near(const Eigen::VectorXd& configuration,
                            const double& radius)=0;

  virtual std::map<double,NodePtr> kNearestNeighbors(const Eigen::VectorXd& configuration,
                                 const size_t& k)=0;

  virtual bool findNode(const NodePtr& node)=0;

  virtual bool deleteNode(const NodePtr& node,
                  const bool& disconnect_node=false)=0;

  virtual bool restoreNode(const NodePtr& node)=0;

  virtual unsigned int size(){return size_;}

  virtual std::vector<NodePtr> getNodes()=0;

protected:
  unsigned int size_;
  unsigned int delete_nodes_;
};

typedef std::shared_ptr<NearestNeighbors> NearestNeighborsPtr;

}  // namespace pathplan
