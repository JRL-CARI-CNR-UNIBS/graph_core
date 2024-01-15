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
*/


#include <graph_core/datastructure/kdtree.h>

namespace graph
{
namespace core
{
/*
 *        ---- KdNode ----
 */

KdNode::KdNode(const NodePtr& node,
               const int &dimension,
               const cnr_logger::TraceLoggerPtr &logger):
  node_(node),
  dimension_(dimension),
  logger_(logger)
{
  deleted_=false;
}

KdNode::~KdNode()
{
//  CNR_DEBUG(logger_,"destroying kdnode "<<this);
  left_  = nullptr;
  right_ = nullptr;
  node_  = nullptr;
}

NodePtr KdNode::node()
{
  return node_;
}

KdNodePtr KdNode::left()
{
  return left_;
}

KdNodePtr KdNode::right()
{
  return right_;
}

KdNodeWeakPtr KdNode::parent()
{
  return parent_;
}

void KdNode::left(const KdNodePtr& kdnode)
{
  left_=kdnode;
}

void KdNode::right(const KdNodePtr& kdnode)
{
  right_=kdnode;
}

void KdNode::parent(const KdNodeWeakPtr& kdnode)
{
  parent_=kdnode;
}

int KdNode::dimension()
{
  return dimension_;
}

void KdNode::insert(const NodePtr& node)
{
  int size=node->getConfiguration().size();
  int next_dim=(dimension_==(size-1))?0:dimension_+1;
  if (node->getConfiguration()(dimension_)>=node_->getConfiguration()(dimension_))  // goRight
  {
    if (not right_)
    {
      right_=std::make_shared<KdNode>(node,next_dim,logger_);
      right_->parent(pointer());
    }
    else
      right_->insert(node);
  }
  else //goLeft
  {
    if (not left_)
    {
      left_=std::make_shared<KdNode>(node,next_dim,logger_);
      left_->parent(pointer());
    }
    else
      left_->insert(node);
  }
}

KdNodePtr KdNode::findMin(const int& dim)
{
  if (dimension_==dim)
  {
    if (not left_)
      return pointer();
    else
      return left_->findMin(dim);
  }
  else
  {
    if(not left_ and not right_)
      return pointer();

    KdNodePtr min_left, min_right;

    if(left_)
      min_left  = left_->findMin(dim);

    if(right_)
      min_right = right_->findMin(dim);

    KdNodePtr min_kdnode = pointer();

    if(min_left && (min_left->node()->getConfiguration()(dim)<min_kdnode->node()->getConfiguration()(dim)))
      min_kdnode = min_left;
    if(min_right && (min_right->node()->getConfiguration()(dim)<min_kdnode->node()->getConfiguration()(dim)))
      min_kdnode = min_right;

    return min_kdnode;
  }
}

void KdNode::nearestNeighbor(const Eigen::VectorXd& configuration,
                             NodePtr& best,
                             double& best_distance)
{
  double distance=(configuration-node_->getConfiguration()).norm();
  if ((not deleted_) and distance<best_distance)
  {
    best_distance=distance;
    best=node_;
  }

  SearchDirection dir=SearchDirection::Left;
  if (configuration(dimension_)>node_->getConfiguration()(dimension_))
    dir=SearchDirection::Right;

  if (dir==SearchDirection::Left)
  {
    if (left_ &&
        (configuration(dimension_)-best_distance)<=node_->getConfiguration()(dimension_))
    {
      left_->nearestNeighbor(configuration,best,best_distance);
    }
    if (right_ &&
        (configuration(dimension_)+best_distance)>=node_->getConfiguration()(dimension_))
    {
      right_->nearestNeighbor(configuration,best,best_distance);
    }
  }
  else  //  (dir==SearchDirection::Right)
  {
    if (right_ &&
        (configuration(dimension_)+best_distance)>=node_->getConfiguration()(dimension_))
    {
      right_->nearestNeighbor(configuration,best,best_distance);
    }
    if (left_ &&
        (configuration(dimension_)-best_distance)<=node_->getConfiguration()(dimension_))
    {
      left_->nearestNeighbor(configuration,best,best_distance);
    }
  }
}

void KdNode::near(const Eigen::VectorXd& configuration,
                  const double& radius,
                  std::multimap<double, NodePtr> &nodes)
{
  double distance=(configuration-node_->getConfiguration()).norm();

  if ((not deleted_) and distance<radius)
  {
    nodes.insert(std::pair<double,NodePtr>(distance,node_));
  }

  if (left_ &&
      (configuration(dimension_)-radius)<=node_->getConfiguration()(dimension_))
  {
    left_->near(configuration,radius,nodes);
  }
  if (right_ &&
      (configuration(dimension_)+radius)>=node_->getConfiguration()(dimension_))
  {
    right_->near(configuration,radius,nodes);
  }
}

void KdNode::kNearestNeighbors(const Eigen::VectorXd& configuration,
                               const size_t& k,
                               std::multimap<double, NodePtr> &nodes)
{
  double last_distance;
  if (nodes.empty())
    last_distance=std::numeric_limits<double>::infinity();
  else
    last_distance=std::prev(nodes.end())->first;

  double distance=(configuration-node_->getConfiguration()).norm();
  if ((not deleted_) and nodes.size()<k)
  {
    nodes.insert(std::pair<double,NodePtr>(distance,node_));
    last_distance=std::prev(nodes.end())->first;
  }
  else if ((not deleted_) and distance<last_distance)
  {
    nodes.erase(std::prev(nodes.end()));
    nodes.insert(std::pair<double,NodePtr>(distance,node_));
    last_distance=std::prev(nodes.end())->first;
  }

  SearchDirection dir=SearchDirection::Left;
  if (configuration(dimension_)>node_->getConfiguration()(dimension_))
    dir=SearchDirection::Right;

  if (dir==SearchDirection::Left)
  {
    if (left_ &&
        (configuration(dimension_)-last_distance)<=node_->getConfiguration()(dimension_))
    {
      left_->kNearestNeighbors(configuration,k,nodes);
    }
    if (right_ &&
        (configuration(dimension_)+last_distance)>=node_->getConfiguration()(dimension_))
    {
      right_->kNearestNeighbors(configuration,k,nodes);
    }
  }
  else  //  (dir==SearchDirection::Right)
  {
    if (right_ &&
        (configuration(dimension_)+last_distance)>=node_->getConfiguration()(dimension_))
    {
      right_->kNearestNeighbors(configuration,k,nodes);
    }
    if (left_ &&
        (configuration(dimension_)-last_distance)<=node_->getConfiguration()(dimension_))
    {
      left_->kNearestNeighbors(configuration,k,nodes);
    }
  }
}

bool KdNode::findNode(const NodePtr& node,
                      KdNodePtr& kdnode)
{
  // is this node?
  if (node_==node)
  {
    kdnode=pointer();
    return true;
  }
  // otherwise search the node
  if (node->getConfiguration()(dimension_)>=node_->getConfiguration()(dimension_))  // goRight
  {
    if (not right_)
      return false;
    return right_->findNode(node,kdnode);
  }
  else  // goLefta
  {
    if (not left_)
      return false;
    return left_->findNode(node,kdnode);
  }
}

void KdNode::deleteNode(const bool& disconnect_node)
{
  deleted_=true;
  if (disconnect_node)
    node_->disconnect();
}

void KdNode::restoreNode()
{
  deleted_=false;
}

void KdNode::getNodes(std::vector<NodePtr>& nodes)
{
  if (not deleted_)
    nodes.push_back(node_);
  if (left_)
    left_->getNodes(nodes);
  if (right_)
    right_->getNodes(nodes);
}

void KdNode::disconnectNodes(const std::vector<NodePtr>& white_list)
{
  if (std::find(white_list.begin(),white_list.end(),node_)==white_list.end())
    node_->disconnect();
  if (left_)
    left_->disconnectNodes(white_list);
  if (right_)
    right_->disconnectNodes(white_list);
}

std::ostream& operator<<(std::ostream& os, const KdNode& kdnode)
{
  os << "node-> "<<kdnode.node_->getConfiguration().transpose()<<" ("<<kdnode.node_<<")"<<std::endl;
  os << "dimension-> "<<kdnode.dimension_<<" deleted-> "<<kdnode.deleted_<<std::endl;
  os << "parent-> "<<kdnode.parent_.lock()<<" left child-> "<<kdnode.left_<<" right child-> "<<kdnode.right_<<std::endl;
  return os;
}

/*
 *        ---- KdTree ----
 */

KdTree::KdTree(const cnr_logger::TraceLoggerPtr &logger):
  NearestNeighbors(logger)
{
  print_deleted_nodes_ = false;
  deleted_nodes_threshold_ = std::numeric_limits<unsigned int>::max();
}

KdTree::~KdTree()
{
  root_ = nullptr;
}

void KdTree::insert(const NodePtr& node)
{
  size_++;
  // if not root, initialize it
  if (not root_)
  {
    root_=std::make_shared<KdNode>(node,0,logger_); //parent_=nullptr by default
    return;
  }
  root_->insert(node);
  return;
}

NodePtr KdTree::findMin(const int& dim)
{
  if (not root_)
    return nullptr;
  return root_->findMin(dim)->node();
}

void KdTree::nearestNeighbor(const Eigen::VectorXd& configuration,
                             NodePtr &best,
                             double &best_distance)
{
  best_distance=std::numeric_limits<double>::infinity();
  if (not root_)
    return;
  root_->nearestNeighbor(configuration,best,best_distance);
}


std::multimap<double, NodePtr> KdTree::near(const Eigen::VectorXd& configuration,
                                                        const double& radius)
{
  std::multimap<double, NodePtr> nodes;
  if (not root_)
    return nodes;

  root_->near(configuration,radius,nodes);
  return nodes;
}

std::multimap<double, NodePtr> KdTree::kNearestNeighbors(const Eigen::VectorXd& configuration,
                                                         const size_t& k)
{
  std::multimap<double, NodePtr> nodes;
  if (not root_)
    return nodes;

  root_->kNearestNeighbors(configuration,k,nodes);
  return nodes;
}

bool KdTree::findNode(const NodePtr& node,
                      KdNodePtr& kdnode)
{
  if (not root_)
    return false;
  return root_->findNode(node,kdnode);
}


bool KdTree::findNode(const NodePtr& node)
{
  KdNodePtr kdnode;
  return findNode(node,kdnode);
}


bool KdTree::deleteNode(const NodePtr& node,
                        const bool& disconnect_node)
{
  KdNodePtr kdnode;
  if (not findNode(node,kdnode))
    return false;

  size_--;
  deleted_nodes_++;
  kdnode->deleteNode(disconnect_node);

  if(deleted_nodes_>deleted_nodes_threshold_ && size_>0)
  {
    CNR_DEBUG(logger_,"number of deleted nodes ("<<deleted_nodes_<<") is greater than the threshold ("
              <<deleted_nodes_threshold_<<"), kdtree is built from scratch");

    bool root_was_deleted = root_->deleted_;
    root_->restoreNode(); //set deleted_ to false to have it into nodes vector below (we want the root regardless its "deleted_" flag)
    std::vector<NodePtr> nodes = getNodes(); //contains also the root

    root_ = nullptr; //clear the KdTree

    size_ = 0;
    deleted_nodes_ = 0;

    for(const NodePtr& n:nodes)
      insert(n);

    if(root_was_deleted)
      deleteNode(root_->node_,disconnect_node);
  }

  return true;
}

unsigned int KdTree::deletedNodesThreshold()
{
  return deleted_nodes_threshold_;
}

void KdTree::deletedNodesThreshold(const unsigned int t)
{
  if(t<=1)
  {
    CNR_WARN(logger_, "deleted_nodes_threshold_ cannot bet set because it should be at least 2 and you are trying to set "<<t);
    return;
  }
  deleted_nodes_threshold_ = t;
}

bool KdTree::restoreNode(const NodePtr& node)
{
  KdNodePtr kdnode;
  if (not findNode(node,kdnode))
    return false;
  size_++;
  deleted_nodes_--;
  kdnode->restoreNode();
  return true;
}

std::vector<NodePtr> KdTree::getNodes()
{
  std::vector<NodePtr> nodes;
  if (not root_)
    return nodes;

  root_->getNodes(nodes);
  return nodes;
}

void KdTree::disconnectNodes(const std::vector<NodePtr>& white_list)
{
  root_->disconnectNodes(white_list);
}

std::ostream& operator<<(std::ostream& os, const KdTree& kdtree)
{
  os<<"root-> "<<kdtree.root_<<std::endl;
  os<<"size-> "<<kdtree.size_<<" deleted nodes-> "<<kdtree.deleted_nodes_<<" print deleted nodes-> "<<kdtree.print_deleted_nodes_<<"\n"<<std::endl;

  std::function<void(const KdNodePtr&)> fcn;
  fcn = [&](const KdNodePtr& root) ->void{
    if(root)
    {
      if((kdtree.print_deleted_nodes_) || (not kdtree.print_deleted_nodes_ && not root->deleted_))
      {
        os << "   --- kdnode "<<root<<" ---"<<std::endl;
        os<<*root<<std::endl;
      }

      fcn(root->left_);
      fcn(root->right_);
    }
    return;
  };

  fcn(kdtree.root_);

  return os;
}

} //end namespace core
} // end namespace graph

