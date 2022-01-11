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

#include <graph_core/datastructure/nearest_neighbors.h>
namespace pathplan
{

class KdTree;
typedef std::shared_ptr<KdTree> KdTreePtr;
class KdNode;
typedef std::shared_ptr<KdNode> KdNodePtr;

enum SearchDirection {Left,Right};

class KdNode: public std::enable_shared_from_this<KdNode>
{
public:
  KdNode(const NodePtr& node,
         const int& dimension);

  NodePtr node();

  int dimension();

  KdNodePtr pointer()
  {
    return shared_from_this();
  }

  KdNodePtr left();

  KdNodePtr right();

  void left(const KdNodePtr& kdnode);

  void right(const KdNodePtr& kdnode);

  void deleteNode(const bool& disconnect_node=false);

  void restoreNode();

  void insert(const NodePtr& node);

  KdNodePtr findMin(const int& dim);

  void nearestNeighbor(const Eigen::VectorXd& configuration,
                          NodePtr &best,
                       double &best_distance);

  void near(const Eigen::VectorXd& configuration,
            const double& radius,
            std::map<double, pathplan::NodePtr>& nodes);

  void kNearestNeighbors(const Eigen::VectorXd& configuration,
             const size_t& k,
             std::map<double,NodePtr>& nodes);

  // return false if the node is not found
  bool findNode(const NodePtr& node,
                KdNodePtr& kdnode);

  void getNodes(std::vector<NodePtr>& nodes);
protected:
  NodePtr node_;
  KdNodePtr left_;
  KdNodePtr right_;
  int dimension_;
  bool deleted_;
};

class KdTree: public NearestNeighbors
{
public:
  KdTree();

  virtual void insert(const NodePtr& node) override;

  NodePtr findMin(const int& dim);

  virtual void nearestNeighbor(const Eigen::VectorXd& configuration,
                          NodePtr &best,
                          double &best_distance) override;


  virtual std::map<double, NodePtr> near(const Eigen::VectorXd& configuration,
                            const double& radius) override;

  virtual std::map<double,NodePtr> kNearestNeighbors(const Eigen::VectorXd& configuration,
                                 const size_t& k) override;

  // return false if the node is not found
  bool findNode(const NodePtr& node,
                KdNodePtr& kdnode);

  virtual bool findNode(const NodePtr& node) override;

  virtual bool deleteNode(const NodePtr& node,
                  const bool& disconnect_node=false) override;

  virtual bool restoreNode(const NodePtr& node) override;

  virtual std::vector<NodePtr> getNodes() override;
protected:
  KdNodePtr root_;
};

}  // namespace pathplan
