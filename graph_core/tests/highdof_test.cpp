#include <ros/ros.h>
#include <graph_core/metrics.h>
#include <graph_core/graph/node.h>
#include <graph_core/graph/path.h>
#include <graph_core/graph/tree.h>
#include <graph_core/solvers/rrt_connect.h>
#include <graph_core/solvers/birrt.h>
#include <graph_core/solvers/rrt_star.h>
#include <graph_core/solvers/path_solver.h>
#include <graph_core/tube_informed_sampler.h>
#include <graph_core/parallel_moveit_collision_checker.h>

#include <graph_core/graph/subtree.h>
#include <graph_core/moveit_collision_checker.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit_msgs/GetPlanningScene.h>
#include <iostream>
#include <fstream>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "high_dof_test");
  ros::NodeHandle nh("~");

  bool m_tube_sampler=true;
  if (!nh.getParam("tube_sampler",m_tube_sampler))
  {
    ROS_ERROR("tube_sampler is not set");
    return 0;
  }

  bool warp=false;
  if (!nh.getParam("warp",warp))
  {
    ROS_DEBUG("warp is not set, set faluse");
    warp=false;
  }

  std::string dirrt_log="high_dof";
  if (m_tube_sampler)
    dirrt_log+="__tube";
  else if (warp)
    dirrt_log+="__warp";
  else
    dirrt_log+="__ellipse";


  double tube_radius=0.1;
  if (!nh.getParam("tube_radius",tube_radius))
  {
    ROS_ERROR("tube_radius is not set");
    return 0;
  }
  dirrt_log+=std::string("__")+"tube_radius"+std::to_string(tube_radius);


  double forgetting_factor=0.99;
  if (!nh.getParam("forgetting_factor",forgetting_factor))
  {
    ROS_ERROR("forgetting_factor is not set");
    return 0;
  }
  dirrt_log+=std::string("__")+"forgetting_factor"+std::to_string(forgetting_factor);


  int maximum_iter=1000000;


  int trials=10;
  if (!nh.getParam("trials",trials))
  {
    ROS_ERROR("trials is not set");
    return 0;
  }
  if (!nh.getParam("maximum_iter",maximum_iter))
  {
    ROS_ERROR("maximum_iter is not set, using 1000000");
    maximum_iter=1000000;
  }


  std::mt19937 m_gen(time(0));
  std::uniform_real_distribution<double> m_ud;


  std::string group_name = "manipulator";
  moveit::planning_interface::MoveGroupInterface move_group(group_name);
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr       kinematic_model = robot_model_loader.getModel();
  planning_scene::PlanningScenePtr planning_scene = std::make_shared<planning_scene::PlanningScene>(kinematic_model);
  ros::ServiceClient ps_client=nh.serviceClient<moveit_msgs::GetPlanningScene>("/get_planning_scene");

  moveit_msgs::GetPlanningScene srv;
  ps_client.call(srv);
  planning_scene->setPlanningSceneMsg(srv.response.scene);
  double steps=nh.param("steps",0.01);
  pathplan::CollisionCheckerPtr checker = std::make_shared<pathplan::MoveitCollisionChecker>(planning_scene, group_name,steps);
  pathplan::MetricsPtr metrics=std::make_shared<pathplan::Metrics>();



  std::vector<std::string> joint_names = kinematic_model->getJointModelGroup(group_name)->getActiveJointModelNames();

  unsigned int dof = joint_names.size();
  std::string log_name1=dirrt_log+"__dof"+std::to_string(dof);
  Eigen::VectorXd lb(dof);
  Eigen::VectorXd ub(dof);

  for (unsigned int idx = 0; idx < dof; idx++)
  {
    const robot_model::VariableBounds& bounds = kinematic_model->getVariableBounds(joint_names.at(idx));
    if (bounds.position_bounded_)
    {
      lb(idx) = bounds.min_position_;
      ub(idx) = bounds.max_position_;
    }
  }
  pathplan::SamplerPtr sampler = std::make_shared<pathplan::InformedSampler>(lb, ub, lb, ub);



  std::vector<double> configuration;
  if (!nh.getParam("dof"+std::to_string(dof)+"/start_configuration",configuration))
  {
    ROS_ERROR("start_configuration is not set");
    return 0;
  }
  if (configuration.size()!= dof)
  {
    ROS_ERROR("starting_configuration has wrong size. %zu instead of %u",configuration.size(),dof);
    return 0;
  }
  Eigen::VectorXd start_conf(dof);
  for (unsigned int idof=0;idof<dof;idof++)
    start_conf(idof)=configuration.at(idof);

  if (!nh.getParam("dof"+std::to_string(dof)+"/goal_configuration",configuration))
  {
    ROS_ERROR("goal_configuration is not set");
    return 0;
  }
  if (configuration.size()!= dof)
  {
    ROS_ERROR("goal_configuration has wrong size. %zu instead of %u",configuration.size(),dof);
    return 0;
  }

  Eigen::VectorXd goal_conf(dof);
  for (unsigned int idof=0;idof<dof;idof++)
    goal_conf(idof)=configuration.at(idof);


  for (int itrial=0;itrial<trials;itrial++)
  {
    ROS_INFO("DOF %d:  TRIAL %u of %d",dof, itrial,trials);
    std::string log_name=log_name1+std::string("__")+"trial"+std::to_string(itrial);

    std::ofstream m_file;
    m_file.open(log_name+".dirrt",std::ios::out | std::ios::binary);
    std::unique_ptr<char[]> m_buf;
    const size_t bufsize = 1024 * 1024;
    m_buf.reset(new char[bufsize]);
    m_file.rdbuf()->pubsetbuf(m_buf.get(), bufsize);


    pathplan::NodePtr start_node = std::make_shared<pathplan::Node>(start_conf);
    pathplan::NodePtr goal_node = std::make_shared<pathplan::Node>(goal_conf);
    std::vector<pathplan::NodePtr> white_list;
    white_list.push_back(start_node);
    white_list.push_back(goal_node);


    pathplan::TubeInformedSamplerPtr tube_sampler = std::make_shared<pathplan::TubeInformedSampler>(start_conf, goal_conf, lb, ub);
    pathplan::SamplerPtr sampler = std::make_shared<pathplan::InformedSampler>(start_conf, goal_conf, lb, ub);
    tube_sampler->setRadius(tube_radius);
    if (m_tube_sampler)
      sampler=tube_sampler;

    pathplan::TreeSolverPtr solver=std::make_shared<pathplan::BiRRT>(metrics, checker, sampler);
    solver->config(nh);
    solver->addGoal(goal_node);
    solver->addStart(start_node);



    bool found_a_solution=false;
    bool local_unoptimality=true;
    double last_purge_cost=std::numeric_limits<double>::infinity();
    double cost_before_refining=std::numeric_limits<double>::infinity();
    double local_probability=1;

    pathplan::PathPtr solution;
    pathplan::PathPtr best_solution;


    ros::WallTime start_time=ros::WallTime::now();
    int iter=0;
    for (;iter<maximum_iter;iter++)
    {
      if (!ros::ok())
      {
        ROS_INFO("Externally stopped");
        return 0;
      }
      if ((ros::WallTime::now()-start_time).toSec()>300)
        continue;

      bool used_tube_sampler=false;
      if (m_tube_sampler && found_a_solution)
      {
        if (local_unoptimality)
        {
          used_tube_sampler=true;
          tube_sampler->setLocalBias(1);
        }
        else
        {
          used_tube_sampler=false;
          tube_sampler->setLocalBias(0);
        }
      }

      double current_cost=solver->cost();
      if (solver->update(solution))
      {
        ROS_INFO_THROTTLE(1,"cost = %f",solution->cost());
        bool improved=false;
        if (!found_a_solution)
        {
          std::shared_ptr<pathplan::RRTStar> opt_solver=std::make_shared<pathplan::RRTStar>(metrics,checker,sampler);
          opt_solver->addStartTree(solver->getStartTree());
          opt_solver->addGoal(goal_node);
          opt_solver->config(nh);
          solver=opt_solver;
          local_probability=1;
          improved=true;
          found_a_solution=true;
          cost_before_refining=solution->cost();
          best_solution=solution;
          sampler->setCost(solution->cost());
          solver->getStartTree()->purgeNodesOutsideEllipsoid(sampler,white_list);
        }
        else if (solution->cost()<best_solution->cost())
        {
          improved=true;
          best_solution=solution;
          double perc_improvement=(solution->cost()/std::max(1e-3,last_purge_cost));
          if (perc_improvement<.99)
          {
            last_purge_cost=solution->cost();
            sampler->setCost(solution->cost());
            solver->getStartTree()->purgeNodesOutsideEllipsoid(sampler,white_list);
          }
        }

        if (improved and warp)
        {
          solution->setTree(solver->getStartTree());
          for (int iwarp=0;iwarp<10;iwarp++)
          {
            solution->warp();
          }
        }

        if (m_tube_sampler)
        {
          double delta_cost=1-solver->cost()/current_cost;
          tube_sampler->setPath(solution);
          if (used_tube_sampler)
            local_probability=std::min(1.,forgetting_factor*local_probability+10*(delta_cost));
        }
      }
      else
      {
        if (m_tube_sampler)
        {
          if (used_tube_sampler)
            local_probability=std::max(1e-10,forgetting_factor*local_probability);
        }
      }


      if (m_tube_sampler)
      {
        double local_is_next= local_probability;
        if (local_unoptimality && m_ud(m_gen)>(local_is_next))
          local_unoptimality=false;
        else if (!local_unoptimality && m_ud(m_gen)>(1-local_is_next))
          local_unoptimality=true;
      }

      if (found_a_solution)
      {
        ROS_INFO_THROTTLE(2,"time = %f, iter = %u, nodes=%u, local prob =%f, best solution = %f",(ros::WallTime::now()-start_time).toSec(),iter,solver->getStartTree()->getNumberOfNodes(),local_probability, best_solution->cost());
      }

      double time=(ros::WallTime::now()-start_time).toSec();
      m_file.write((char*) &time, sizeof(double));
      m_file.write((char*) &iter, sizeof(unsigned int));
      unsigned int nodes=solver->getStartTree()->getNumberOfNodes();
      m_file.write((char*) &nodes, sizeof(unsigned int));
      m_file.write((char*) &(local_probability), sizeof(double));
      if (found_a_solution)
        m_file.write((char*) &(best_solution->cost()), sizeof(double));
      else
        m_file.write((char*) &(cost_before_refining), sizeof(double));

    }



    if (!found_a_solution)
    {
      ROS_ERROR("unable to find a valid path");
      m_file.close();
      continue;
    }

    ROS_INFO("time = %f, iter = %u",(ros::WallTime::now()-start_time).toSec(),iter);
    ROS_INFO("Refinement of the solution form %f to %f.",cost_before_refining,best_solution->cost());


  }
  return 0;
}
