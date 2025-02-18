﻿#pragma once
/*
Copyright (c) 2024, Manuel Beschi and Cesare Tonola, JRL-CARI CNR-STIIMA/UNIBS,
manuel.beschi@unibs.it, c.tonola001@unibs.it All rights reserved.

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

#include <graph_core/graph/path.h>
#include <graph_core/graph/tree.h>
#include <graph_core/metrics/goal_cost_function_base.h>
#include <graph_core/samplers/sampler_base.h>

namespace graph
{
namespace core
{
class TreeSolver;
typedef std::shared_ptr<TreeSolver> TreeSolverPtr;

/**
 * @class TreeSolver
 * @brief A base class for tree-based path planning solvers.
 *
 * This class provides a base for implementing tree-based path planning
 * algorithms. It includes functionality for configuring the solver, setting up
 * the problem, and solving for a solution path.
 */
class TreeSolver : public std::enable_shared_from_this<TreeSolver>
{
protected:
  /**
   * @brief Pointer to the metrics used for evaluating paths.
   */
  MetricsPtr metrics_;

  /**
   * @brief Pointer to the collision checker used to verify the validity of
   * paths.
   */
  CollisionCheckerPtr checker_;

  /**
   * @brief Pointer to the sampler responsible for generating random
   * configurations.
   */
  SamplerPtr sampler_;

  /**
   * @brief Pointer to the goal cost function used to evaluate the cost
   * associated with reaching the goal.
   */
  GoalCostFunctionPtr goal_cost_fcn_;

  /**
   * @brief Flag indicating  whether the path planning task is solved, i.e., a
   * solution has been found.
   */
  bool solved_ = false;

  /**
   * @brief  Flag indicating whether the path found can still be improved, i.e.
   * whether the optimal solution has not been found for an optimal path
   * planner.
   */
  bool can_improve_ = true;

  /**
   * @brief Flag indicating whether the problem has been defined, i.e., start
   * and goal node defined.
   */
  bool problem_set_ = false;

  /**
   * @brief Flag indicating whether the solver is configured, i.e, parameters
   * retrived.
   */
  bool configured_ = false;

  /**
   * @brief Pointer to the tree of the path planning problem.
   */
  TreePtr start_tree_;

  /**
   * @brief The degree of freedom of the search space of the path planning
   * problem.
   */
  unsigned int dof_;

  /**
   * @brief The namespace under which the parameters are searched for.
   */
  std::string param_ns_;

  /**
   * @brief Maximum distance considered by the solver for tree expansion.
   */
  double max_distance_;

  /**
   * @brief Flag indicating whether to use extend or connect algorithm for tree
   * expansion.
   */
  bool extend_;

  /**
   * @brief Tolerance for the utopia value. A solution is considered optimal if
   * its cost is less than utopia_tolerance_*best_utopia_;
   */
  double utopia_tolerance_;

  /**
   * @brief Flag indicating whether to use KD-tree for efficient nearest
   * neighbor search or a std::vector.
   */
  bool use_kdtree_;

  /**
   * @brief initialized_ Flag to indicate whether the object is initialised,
   * i.e. whether its members have been defined correctly. It is false when the
   * object is created with an empty constructor. In this case, call the 'init'
   * function to initialise it. The other constructors automatically initialise
   * the object. As long as the object is not initialised, it cannot perform its
   * main functions.
   */
  bool initialized_ = false;

  /**
   * @brief Pointer to the goal node of the path planning problem.
   */
  NodePtr goal_node_;

  /**
   * @brief Cost of the path associated with the current solution.
   */
  double path_cost_ = 0;

  /**
   * @brief Goal cost associated with the current solution.
   */
  double goal_cost_ = 0;

  /**
   * @brief Total cost, sum of path cost and goal cost.
   */
  double cost_ = 0;

  /**
   * @brief Pointer to the solution path of the path planning problem.
   */
  PathPtr solution_;

  /**
   * @brief Best utopia value associated with the solver.
   */
  double best_utopia_ = std::numeric_limits<double>::infinity();

  /**
   * @brief Pointer to a TraceLogger instance for logging.
   *
   * This member variable represents a pointer to a TraceLogger instance,
   * allowing to perform logging operations. TraceLogger is a part of the
   * cnr_logger library. Ensure that the logger is properly configured and
   * available for use.
   */
  cnr_logger::TraceLoggerPtr logger_;

  /**
   * @brief Set the path planning problem and attempt to find a solution.
   *
   * The function initializes the path planning problem by setting the start
   * tree, goal node, and other relevant parameters. It computes the goal cost
   * and initializes the best utopia value. If the goal node is already in the
   * start tree, a direct connection is established, and the solution path is
   * set. Otherwise, it attempts to directly connect the start tree to the goal
   * node. If successful, the solution path is set, and the solver is marked as
   * solved. The total cost, path cost, and other relevant values are updated
   * accordingly.
   *
   * @param max_time The maximum allowed time for trying to directly connect the
   * start to the goal.
   * @return true if setting the problem and finding a solution is successful,
   * false otherwise.
   */
  virtual bool setProblem(const double& max_time = std::numeric_limits<double>::infinity());

  /**
   * @brief printMyself
   * @param os
   */
  virtual void printMyself(std::ostream& os) const;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * @brief Empty constructor for TreeSolver. The function init() must be called
   * afterwards.
   */
  TreeSolver()
  {
    path_cost_ = std::numeric_limits<double>::infinity();
    goal_cost_ = 0.0;
    cost_ = std::numeric_limits<double>::infinity();
    goal_cost_fcn_ = std::make_shared<GoalCostFunctionBase>();

    initialized_ = false;
  }

  /**
   * @brief Constructor for TreeSolver class.
   *
   * @param metrics The metrics used to evaluate paths.
   * @param checker The collision checker for checking collisions.
   * @param sampler The sampler for generating random configurations.
   * @param goal_cost_fcn The function used to assign the cost of the goal. If
   * it is not defined, the default cost function does not assign any cost to
   * the goal.
   * @param logger The logger for logging messages.
   */
  TreeSolver(const MetricsPtr& metrics, const CollisionCheckerPtr& checker, const SamplerPtr& sampler,
             const GoalCostFunctionPtr& goal_cost_fcn, const cnr_logger::TraceLoggerPtr& logger)
    : metrics_(metrics), checker_(checker), sampler_(sampler), goal_cost_fcn_(goal_cost_fcn), logger_(logger)
  {
    init(metrics, checker, sampler, goal_cost_fcn, logger);
  }

  TreeSolver(const MetricsPtr& metrics, const CollisionCheckerPtr& checker, const SamplerPtr& sampler,
             const cnr_logger::TraceLoggerPtr& logger)
    : metrics_(metrics), checker_(checker), sampler_(sampler), logger_(logger)
  {
    init(metrics, checker, sampler, logger);
  }

  /**
   * @brief init Initialise the object, defining its main attributes. At the end
   * of the function, the flag 'initialized_' is set to true and the object can
   * execute its main functions.
   * @param metrics The metrics used to evaluate paths.
   * @param checker The collision checker for checking collisions.
   * @param sampler The sampler for generating random configurations.
   * @param goal_cost_fcn The function used to assign the cost of the goal. If
   * it is not defined, the default cost function does not assign any cost to
   * the goal.
   * @param logger The logger for logging messages.
   * @return True if correctly initialised, False if already initialised.
   */
  virtual bool init(const MetricsPtr& metrics, const CollisionCheckerPtr& checker, const SamplerPtr& sampler,
                    const GoalCostFunctionPtr& goal_cost_fcn, const cnr_logger::TraceLoggerPtr& logger)
  {
    if (initialized_)
    {
      CNR_WARN(logger_, "Solver already initialised!");
      return false;
    }

    metrics_ = metrics;
    checker_ = checker;
    sampler_ = sampler;
    goal_cost_fcn_ = goal_cost_fcn;
    logger_ = logger;

    path_cost_ = std::numeric_limits<double>::infinity();
    goal_cost_ = 0.0;
    cost_ = std::numeric_limits<double>::infinity();

    initialized_ = true;

    return true;
  }

  virtual bool init(const MetricsPtr& metrics, const CollisionCheckerPtr& checker, const SamplerPtr& sampler,
                    const cnr_logger::TraceLoggerPtr& logger)
  {
    GoalCostFunctionPtr goal_cost_fcn = std::make_shared<GoalCostFunctionBase>();
    return init(metrics, checker, sampler, goal_cost_fcn, logger);
  }

  /**
   * @brief Configure the solver with parameters.
   *
   * This function reads parameters using cnr_param and configures the solver
   * accordingly.
   *
   * @param param_ns The namespace under which the parameters are searched for.
   * @return true if configuration is successful, false otherwise.
   */
  virtual bool config(const std::string& param_ns);

  /**
   * @brief getInitialized tells if the object has been initialised.
   * @return the 'initialized_' flag.
   */
  bool getInitialized()
  {
    return initialized_;
  }

  /**
   * @brief Update is the single step of the search for a solution.
   * It is repeatedly called by the solve() function to expand the tree towards
   * the goal.
   *
   * This pure virtual function should be implemented by derived classes to
   * progress the search. There are two variants of this function that already
   * provide the update function with the configuration or node towards which to
   * expand the tree. Their implementation is optional.
   *
   * @param solution The output solution path.
   * @return true if the solution is updated, false otherwise.
   */
  virtual bool update(PathPtr& solution) = 0;
  virtual bool update(const NodePtr& n, PathPtr& solution)
  {
    return false;
  }
  virtual bool update(const Eigen::VectorXd& configuration, PathPtr& solution)
  {
    return false;
  }

  /**
   * @brief Solve for a path.
   *
   * This function attempts to find a solution path using the configured solver.
   *
   * @param solution The output solution path.
   * @param max_iter The maximum number of iterations.
   * @param max_time The maximum allowed time for solving.
   * @return true if a solution is found, false otherwise.
   */
  virtual bool solve(PathPtr& solution, const unsigned int& max_iter = 100,
                     const double& max_time = std::numeric_limits<double>::infinity());

  /**
   * @brief Set the start node of the path planning problem.
   *
   * This function adds a start node to the path planning problem. The solver
   * must be configured before using this function.
   *
   * @param start_node The start node to be added.
   * @param max_time The maximum allowed time for adding the start node.
   * @return true if the start node is added successfully, false otherwise.
   */
  virtual bool addStart(const NodePtr& start_node,
                        const double& max_time = std::numeric_limits<double>::infinity()) = 0;

  /**
   * @brief Set the goal node of the path planning problem.
   *
   * This function adds a goal node to the path planning problem. The solver
   * must be configured before using this function.
   *
   * @param goal_node The goal node to be added.
   * @param max_time The maximum allowed time for adding the goal node.
   * @return true if the goal node is added successfully, false otherwise.
   */
  virtual bool addGoal(const NodePtr& goal_node, const double& max_time = std::numeric_limits<double>::infinity()) = 0;

  /**
   * @brief Set a tree to be used to solve the path planning problem.
   *
   * This function adds a start tree to the path planning problem. The solver
   * must be configured before using this function.
   *
   * @param start_tree The start tree to be added.
   * @param max_time The maximum allowed time for adding the start tree.
   * @return true if the start tree is added successfully, false otherwise.
   */
  virtual bool addStartTree(const TreePtr& start_tree,
                            const double& max_time = std::numeric_limits<double>::infinity()) = 0;

  /**
   * @brief Compute a path planning path between given configurations.
   *
   * This function computes a path planning path between the given start and
   * goal configurations.
   *
   * @param start_conf The start configuration.
   * @param goal_conf The goal configuration.
   * @param param_ns The namespace under which the parameters should be searched
   * for.
   * @param solution The output solution path.
   * @param max_time The maximum allowed time for solving.
   * @param max_iter The maximum number of iterations.
   * @return true if a solution is found, false otherwise.
   */
  virtual bool computePath(const NodePtr& start_node, const NodePtr& goal_node, const std::string& param_ns,
                           PathPtr& solution, const double& max_time = std::numeric_limits<double>::infinity(),
                           const unsigned int& max_iter = 10000);
  virtual bool computePath(const Eigen::VectorXd& start_conf, const Eigen::VectorXd& goal_conf,
                           const std::string& param_ns, PathPtr& solution,
                           const double& max_time = std::numeric_limits<double>::infinity(),
                           const unsigned int& max_iter = 10000);

  /**
   * @brief Reset the path planning problem.
   *
   * This pure virtual function should be implemented by derived classes to
   * reset the path planning problem.
   */
  virtual void resetProblem() = 0;

  /**
   * @brief Set the solution for the solver.
   *
   * This function sets the solution for the solver and updates relevant
   * information. If the solution is valid it becomes the current solution for
   * the TreeSolver and costs and relevant information are updated accordingly.
   *
   * The solution is considered valid if its cost is finite. In this case it
   * marks the problem as solved. Additionally, the problem is marked as
   * completed if the solution cost is below a certain tolerance factor
   * multiplied by the utopia value.
   *
   * The solver needs to be configured before (calling config method).
   *
   * Note that the function determines the solution's validity without
   * performing collision checks on the path and tree, but simply relying on the
   * current solution cost.
   *
   * @param solution A shared pointer to a Path object representing the
   * solution.
   * @return Returns true if the solution is valid and set successfully, false
   * otherwise.
   */
  virtual bool setSolution(const PathPtr& solution);

  /**
   * @brief Import solver parameters from another solver.
   *
   * This function imports common parameters from another solver into the
   * current solver. It copies the general configuration usign the config
   * function and set the current solution, if available.
   *
   * @param solver The source solver.
   * @return true if the import is successful, false otherwise.
   */
  virtual bool importFromSolver(const TreeSolverPtr& solver);

  /**
   * @brief Get the cost associated with the current solution.
   *
   * This function returns the cost associated with the current solution.
   *
   * @return A constant reference to the cost.
   */
  const double& cost() const
  {
    return cost_;
  }

  /**
   * @brief Set the goal cost function for the solver.
   *
   * This function sets the goal cost function for the solver.
   *
   * @param goal_cost_fcn The goal cost function to be set.
   */
  void setGoalCostFunction(const GoalCostFunctionPtr& goal_cost_fcn)
  {
    goal_cost_fcn_ = goal_cost_fcn;
  }

  /**
   * @brief Check if the path found can still be improved.
   *
   * This function tells whether the path found can be improved.
   * When a path planner finds a solution the flag 'solved_' is enabled. If the
   * path planner is still able to improve the solution found, 'can_improve_' is
   * set to true. Some algorithms stop as soon as a solution is found (e.g.,
   * RRT), others continue to improve the solution untile the optimal one is
   * found (e.g., RRT*).
   *
   * @return A constant reference to 'can_improve_', which tells if the planner
   * could be able to improve the solution.
   */
  const bool& canImprove() const
  {
    return can_improve_;
  }

  /**
   * @brief Check if a solution has been found.
   *
   * This function returns a constant reference indicating whether a solution
   * has been found. A path planning problem is:
   *  - solved: A solution has been found;
   *  - completed: The optimal solution has been found.
   *
   * @return A constant reference to the solution status.
   */
  const bool& solved() const
  {
    return solved_;
  }

  /**
   * @brief Check if the problem has been set, i.e., the setProblem function has
   * been executed successfully.
   *
   * @return A constant reference to the problem status.
   */
  const bool& problemStatus() const
  {
    return problem_set_;
  }

  /**
   * @brief Check if the solver is configured, i.e., the config function has
   * been executed successfully.
   *
   * This function returns a constant reference indicating whether the solver is
   * configured.
   *
   * @return A constant reference to the configuration status.
   */
  const bool& configured() const
  {
    return configured_;
  }

  /**
   * @brief Get the degree of freedom of the search space of this path planning
   * problem.
   *
   * This function returns a constant reference to the degree of freedom of the
   * search space of this path planning problem.
   *
   * @return A constant reference to the degree of freedom.
   */
  const unsigned int& dof() const
  {
    return dof_;
  }

  /**
   * @brief Get the goal cost function.
   *
   * This function returns the goal cost function used by the solver.
   *
   * @return A pointer to the goal cost function.
   */
  GoalCostFunctionPtr getGoalCostFunction() const
  {
    return goal_cost_fcn_;
  }

  /**
   * @brief Get the start tree of the path planning problem.
   *
   * This function returns the start tree of the path planning problem.
   *
   * @return A pointer to the start tree.
   */
  TreePtr getStartTree() const
  {
    return start_tree_;
  }

  /**
   * @brief Get the solution path.
   *
   * This function returns the solution path of the path planning problem.
   *
   * @return A pointer to the solution path.
   */
  PathPtr getSolution() const
  {
    return solution_;
  }

  /**
   * @brief Set the sampler.
   * @param sampler The sampler to be set.
   */
  void setSampler(const SamplerPtr& sampler)
  {
    sampler_ = sampler;
  }

  /**
   * @brief Get the sampler used by the solver.
   * @return A pointer to the sampler.
   */
  SamplerPtr getSampler() const
  {
    return sampler_;
  }

  /**
   * @brief Get the cost of the path.
   *
   * This function returns the cost of the path associated with the current
   * solution.
   *
   * @return The cost of the path.
   */
  double getPathCost() const
  {
    return path_cost_;
  }

  /**
   * @brief Get the goal cost.
   *
   * This function returns the goal cost associated with the current solution.
   *
   * @return The goal cost.
   */
  double getGoalCost() const
  {
    return goal_cost_;
  }

  /**
   * @brief Get the total cost.
   *
   * This function returns the total cost, which is the sum of the path cost and
   * the goal cost.
   *
   * @return The total cost.
   */
  double getCost() const
  {
    return cost_;
  }

  /**
   * @brief Set the collision checker for the solver.
   *
   * This function sets the collision checker for the solver.
   *
   * @param checker The collision checker to be set.
   */
  void setChecker(const CollisionCheckerPtr& checker)
  {
    checker_ = checker;
  }

  /**
   * @brief Get the collision checker used by the solver.
   *
   * This function returns the collision checker used by the solver.
   *
   * @return A pointer to the collision checker.
   */
  CollisionCheckerPtr getChecker() const
  {
    return checker_;
  }

  /**
   * @brief Set the metrics for evaluating paths.
   *
   * This function sets the metrics for evaluating paths.
   *
   * @param metrics The metrics to be set.
   */
  void setMetrics(const MetricsPtr& metrics)
  {
    metrics_ = metrics;
  }

  /**
   * @brief Get the metrics used by the solver.
   *
   * This function returns the metrics used by the solver.
   *
   * @return A pointer to the metrics.
   */
  MetricsPtr getMetrics() const
  {
    return metrics_;
  }

  /**
   * @brief Get the goal node of the path planning problem.
   *
   * This virtual function returns the goal node of the path planning problem.
   *
   * @return A pointer to the goal node.
   */
  virtual NodePtr getGoal()
  {
    return goal_node_;
  }

  /**
   * @brief Set the maximum distance used for tree expansion.
   *
   * This function sets the maximum distance for tree expansion.
   *
   * @param distance The maximum distance to be set.
   */
  void setMaxDistance(const double& distance)
  {
    max_distance_ = distance;
  }

  /**
   * @brief Get the maximum distance used by the solver.
   *
   * This function returns the maximum distance used by the solver.
   *
   * @return The maximum distance.
   */
  double getMaxDistance()
  {
    return max_distance_;
  }

  /**
   * @brief Update the cost based on the solution.
   *
   * This function updates the path cost, the goal cost and the total cost.
   *
   * @return The updated total cost.
   */
  double updateCost()
  {
    path_cost_ = solution_->cost();
    goal_cost_ = goal_cost_fcn_->cost(goal_node_);
    cost_ = path_cost_ + goal_cost_;

    return cost_;
  }

  /**
   * @brief Performs the last operations before solving the planning problem.
   * It must be called after setProblem() and before solve() (see computePath()
   * as an example). By default it does nothing, but in general it can be used
   * to read additional parameters specified by param_ns. Parameters are read
   * under the namespace param_ns_ set with the function config().
   * @return true if successfull.
   */
  virtual bool finalizeProblem()
  {
    return true;
  }

  friend std::ostream& operator<<(std::ostream& os, const TreeSolver& solver);
};

/**
 * @brief Overloaded stream insertion operator for the TreeSolver class.
 *
 * This operator prints information about the TreeSolver object to the output
 * stream.
 *
 * @param os The output stream.
 * @param solver The TreeSolver object to be printed.
 * @return The modified output stream.
 */
inline std::ostream& operator<<(std::ostream& os, const TreeSolver& solver)
{
  solver.printMyself(os);
  return os;
}

}  // end namespace core
}  // end namespace graph
