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

#include <graph_core/solvers/path_optimizers/path_local_optimizer.h>

namespace graph
{
namespace core
{
PathLocalOptimizer::PathLocalOptimizer(const CollisionCheckerPtr &checker,
                                       const MetricsPtr &metrics,
                                       const cnr_logger::TraceLoggerPtr &logger):
  PathOptimizerBase(checker,metrics,logger)
{
}

void PathLocalOptimizer::config(const YAML::Node& config)
{
  PathOptimizerBase::config(config);

  if (!config_["simplify_max_conn_length"])
  {
    CNR_WARN(logger_,"simplify_max_conn_length is not set, using 0.1");
    simplify_max_conn_length_ = 0.1;
  }
  else
  {
    simplify_max_conn_length_ = config_["simplify_max_conn_length"].as<double>();
  }

  if (!config_["warp_min_conn_length"])
  {
    CNR_WARN(logger_,"warp_min_conn_length is not set, using 0.01");
    warp_min_conn_length_ = 0.01;
  }
  else
  {
    warp_min_conn_length_ = config_["warp_min_conn_length"].as<double>();
  }

  if (!config_["warp_min_step_size"])
  {
    CNR_WARN(logger_,"warp_min_step_size is not set, using 0.01");
    warp_min_step_size_ = 0.01;
  }
  else
  {
    warp_min_step_size_ = config_["warp_min_step_size"].as<double>();
  }
}

void PathLocalOptimizer::setPath(const PathPtr &path)
{
  PathOptimizerBase::setPath(path);

  change_warp_.clear();
  for(size_t i=0;i<path_->getConnectionsSize();i++)
    change_warp_.push_back(true);

  change_warp_.at(0) = false;
}

bool PathLocalOptimizer::bisection(const size_t &connection_idx,
                                   const Eigen::VectorXd &center,
                                   const Eigen::VectorXd &direction,
                                   const double min_step_size,
                                   double max_distance,
                                   double min_distance)
{
  assert(connection_idx < path_->getConnectionsSize());
  assert(connection_idx > 0);

  std::vector<ConnectionPtr> connections = path_->getConnections();
  ConnectionPtr& conn12 = connections.at(connection_idx - 1);  //ref to connection
  ConnectionPtr& conn23 = connections.at(connection_idx);      //ref to connection

  NodePtr parent = conn12->getParent();
  NodePtr child = conn23->getChild();

  bool improved = false;
  double cost = conn12->getCost() + conn23->getCost();

  unsigned int iter = 0;
  double distance;

  while ((iter++ < 5) && ((max_distance - min_distance) > min_step_size))
  {
    if (iter > 0)
      distance = 0.5 * (max_distance + min_distance);
    else
      distance = min_distance;

    Eigen::VectorXd p = center + direction * distance;
    assert(p.size() == center.size());
    double cost_pn = metrics_->cost(parent->getConfiguration(), p);
    double cost_nc = metrics_->cost(p, child->getConfiguration());
    double cost_n = cost_pn + cost_nc;

    if (cost_n >= cost)
    {
      min_distance = distance;
      continue;
    }
    bool is_valid = checker_->checkConnection(parent->getConfiguration(), p) && checker_->checkConnection(p, child->getConfiguration());
    if (not is_valid)
    {
      min_distance = distance;
      continue;
    }

    improved = true;
    max_distance = distance;
    cost = cost_n;
    // conn12->remove(); // Do not remove, other nodes could be linked to conn12's child

    bool is_net = conn23->isNet();
    conn23->remove();

    NodePtr n = std::make_shared<Node>(p,logger_);
    conn12 = std::make_shared<Connection>(parent, n,logger_);

    is_net? (conn23 = std::make_shared<Connection>(n, child, logger_, true)):
            (conn23 = std::make_shared<Connection>(n, child, logger_, false));

    conn12->setCost(cost_pn);
    conn23->setCost(cost_nc);
    conn12->add();
    conn23->add();

    assert(child->getParentConnectionsSize() == 1);
    assert(conn23->getChild()->getParentConnectionsSize() == 1);

    if (path_->getTree())
      path_->getTree()->addNode(n, false);
  }

  if (improved)
    path_->setConnections(connections);  //updates cost too

  return improved;
}

bool PathLocalOptimizer::warp(const double &min_conn_length, const double min_step_size, const double &max_time)
{
  if(max_time > 0)
  {
    std::vector<ConnectionPtr> connections = path_->getConnections();
    std::chrono::time_point<std::chrono::system_clock> tic = std::chrono::system_clock::now();
    for (unsigned int idx = 1; idx < connections.size(); idx++)
    {
      if(connections.at(idx-1)->norm()>min_conn_length && connections.at(idx)->norm()>min_conn_length)
      {
        if (change_warp_.at(idx - 1) || change_warp_.at(idx))
        {
          Eigen::VectorXd center = 0.5 * (connections.at(idx - 1)->getParent()->getConfiguration() +
                                          connections.at(idx)->getChild()->getConfiguration());
          Eigen::VectorXd direction = connections.at(idx - 1)->getChild()->getConfiguration() - center;
          double max_distance = direction.norm();
          double min_distance = 0;

          direction.normalize();

          bisection(idx,center,direction,min_step_size,max_distance,min_distance)?
                (change_warp_.at(idx) = true):
                (change_warp_.at(idx) = false);
        }
      }

      std::chrono::time_point<std::chrono::system_clock> now = std::chrono::system_clock::now();
      std::chrono::duration<double> difference = now - tic;
      if(difference.count() >= 0.98*max_time) break;
    }
  }

  return std::any_of(change_warp_.cbegin(), change_warp_.cend(), [](bool i){return i;});
}

bool PathLocalOptimizer::simplify(const double& min_conn_length)
{
  bool simplified = false;
  bool reconnect_first_conn = false;

  std::vector<ConnectionPtr> connections = path_->getConnections();

  if(connections.size()>1)
  {
    if(connections.front()->norm() < min_conn_length)
      reconnect_first_conn = true;
  }

  unsigned int ic = 1;
  while (ic < connections.size())
  {
    if (connections.at(ic)->norm() > min_conn_length) //connection longer than the threshold, skip
    {
      /* If the connection at pos 1 is longer than the threshold but the first connection
       * was shorter than the threshold, simplify the conneection.*/
      if(not (ic == 1 && reconnect_first_conn))
      {
        ic++;
        continue;
      }
    }

    /* If connection from previous parent and current child is possible connect them and remove the middle node (current parent)*/
    if (checker_->checkConnection(connections.at(ic - 1)->getParent()->getConfiguration(),
                                  connections.at(ic)->getChild()->getConfiguration()))
    {
      simplified = true;
      double cost = metrics_->cost(connections.at(ic - 1)->getParent(),
                                   connections.at(ic)->getChild());

      ConnectionPtr conn = std::make_shared<Connection>(connections.at(ic - 1)->getParent(),
                                                        connections.at(ic)->getChild(),
                                                        logger_,connections.at(ic)->isNet());
      conn->setCost(cost);
      conn->add();

      connections.at(ic)->remove();
      assert(conn->getChild()->getParentConnectionsSize() == 1);

      connections.erase(connections.begin() + (ic - 1), connections.begin() + ic + 1);
      connections.insert(connections.begin() + (ic - 1), conn);

      change_warp_.erase(change_warp_.begin() + ic);
      if (ic>1)
        change_warp_.at(ic - 1) = true;
    }
    else
      ic++;
  }

  path_->setConnections(connections);

  return simplified;
}

bool PathLocalOptimizer::step()
{
  if (not path_)
    return false;

  bool improved = false;
  if (solved_)
    return false;

  double cost = path_->cost();

  bool solved = not warp(warp_min_conn_length_,warp_min_step_size_);

  if (cost <= (1.001 * path_->cost())) //warp didn't improve the path (cost remained the same)
  {
    if (stall_gen_ == 0) //try with simplify
      simplify(simplify_max_conn_length_)? stall_gen_++ : solved = false;
    else
      stall_gen_++;
  }
  else
  {
    improved = true;
    stall_gen_ = 0;
  }
  solved_ = solved || (stall_gen_ >= max_stall_gen_);
  return improved;
}

} //end namespace core
} // end namespace graph
