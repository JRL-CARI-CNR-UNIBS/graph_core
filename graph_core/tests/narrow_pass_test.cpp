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
#include <graph_core/narrow_pass_checker.h>

#include <iostream>
#include <fstream>
#include <boost/filesystem.hpp>

namespace fs = boost::filesystem;
int main(int argc, char **argv)
{
  ros::init(argc, argv, "narrow_pass_test");
  ros::NodeHandle nh("~");


  bool save_full_logs;
  if (!nh.getParam("save_full_logs",save_full_logs))
  {
    ROS_ERROR("save_full_logs is not set");
    return 0;
  }


  bool tube_sampler=true;
  if (!nh.getParam("tube_sampler",tube_sampler))
  {
    ROS_ERROR("tube_sampler is not set");
    return 0;
  }


  double cylinder_radius=1;
  if (!nh.getParam("cylinder_radius",cylinder_radius))
  {
    ROS_ERROR("cylinder_radius is not set");
    return 0;
  }


  double cylinder_width=1;
  if (!nh.getParam("cylinder_width",cylinder_width))
  {
    ROS_ERROR("cylinder_width is not set");
    return 0;
  }


  double measures_ratio=0.5;
  if (!nh.getParam("measures_ratio",measures_ratio))
  {
    ROS_ERROR("measures_ratio is not set");
    return 0;
  }

  std::vector<double> tube_radii;
  if (!nh.getParam("tube_radius",tube_radii))
  {
    ROS_ERROR("tube_radius is not set");
    return 0;
  }


  std::vector<double> forgetting_factors;
  if (!nh.getParam("forgetting_factor",forgetting_factors))
  {
    ROS_ERROR("forgetting_factor is not set");
    return 0;
  }
  std::vector<int> dofs;
  if (!nh.getParam("dofs",dofs))
  {
    ROS_ERROR("dofs is not set");
    return 0;
  }

  int maximum_iter=1000000;
  if (!nh.getParam("maximum_iter",maximum_iter))
  {
    ROS_ERROR("maximum_iter is not set");
    return 0;
  }

  int trials=10;
  if (!nh.getParam("trials",trials))
  {
    ROS_ERROR("trials is not set");
    return 0;
  }

  double exit_tolerance=1.005;
  if (!nh.getParam("exit_tolerance",exit_tolerance))
  {
    ROS_ERROR("exit_tolerance is not set");
    return 0;
  }


  for (const double& tube_radius: tube_radii)
  {
    for (const double& forgetting_factor: forgetting_factors)
    {




      std::mt19937 m_gen(time(0));
      std::uniform_real_distribution<double> m_ud;

      if (tube_sampler)
        ROS_INFO("TUBE: forgetting_factor%f,tube radius=%f",forgetting_factor,tube_radius);
      else
        ROS_INFO("ELLIPSE");

      std::string directory_name1="narrow_pass/"+std::to_string(ros::WallTime::now().toSec());

      ros::WallTime t0=ros::WallTime::now();
      for (const int& dof: dofs)
      {
        std::string directory_name=directory_name1+"/dof"+std::to_string(dof);
        fs::create_directories(directory_name);


        double hole_radius=cylinder_radius*std::pow(measures_ratio,1.0/(static_cast<double>(dof)-1.0));

        double b=cylinder_width*0.6;
        double a=(cylinder_radius+3*hole_radius)/4.0;

        double global_minimum = cylinder_width+2*std::sqrt(std::pow(0.1*cylinder_width,2)+std::pow(hole_radius-a,2));
        double local_minimum  = cylinder_width+2*std::sqrt(std::pow(0.1*cylinder_width,2)+std::pow(cylinder_radius-a,2));



        std::ofstream description_file;
        description_file.open(directory_name+"/description.yaml",std::ios::out | std::ios::binary);
        if (tube_sampler)
        {
          description_file << "tube_sampler: true"<<std::endl;
          description_file << "forgetting_factor: " << forgetting_factor<<std::endl;
          description_file << "tube_radius: " << tube_radius<<std::endl;
        }
        else
        {
          description_file << "tube_sampler: false"<<std::endl;
        }
        description_file << "cylinder_radius: " << cylinder_radius<<std::endl;
        description_file << "cylinder_width: " << cylinder_width<<std::endl;
        description_file << "measures_ratio: " << measures_ratio<<std::endl;
        description_file << "maximum_iter: " << maximum_iter<<std::endl;
        description_file << "exit_tolerance: " << exit_tolerance<<std::endl;
        description_file << "global_minimum: " << global_minimum<<std::endl;
        description_file << "local_minimum: " << local_minimum<<std::endl;
        description_file.close();



        Eigen::VectorXd start_conf(dof);
        start_conf.setConstant(0);
        start_conf(0) = -b;
        start_conf(1) = a;

        Eigen::VectorXd goal_conf(dof);
        goal_conf.setConstant(0);
        goal_conf(0) = b;
        goal_conf(1) =  a;
        Eigen::VectorXd lb(dof);
        Eigen::VectorXd ub(dof);
        lb.setConstant(-5);
        ub.setConstant(5);

        double utopy = (goal_conf-start_conf).norm();

        for (int itrial=0;itrial<trials;itrial++)
        {
          std::string log_name=directory_name+"/trial"+std::to_string(itrial);

          std::ofstream m_file;
          m_file.open(log_name+".dirrt",std::ios::out | std::ios::binary);
          std::unique_ptr<char[]> m_buf;
          const size_t bufsize = 1024 * 1024;
          m_buf.reset(new char[bufsize]);
          m_file.rdbuf()->pubsetbuf(m_buf.get(), bufsize);



          pathplan::MetricsPtr metrics = std::make_shared<pathplan::Metrics>();
          pathplan::CollisionCheckerPtr checker = std::make_shared<pathplan::NarrowPassChecker>(hole_radius,cylinder_radius,cylinder_width);


          pathplan::NodePtr start_node = std::make_shared<pathplan::Node>(start_conf);
          pathplan::NodePtr goal_node = std::make_shared<pathplan::Node>(goal_conf);
          std::vector<pathplan::NodePtr> white_list;
          white_list.push_back(start_node);
          white_list.push_back(goal_node);


          pathplan::TubeInformedSamplerPtr tube_sampler = std::make_shared<pathplan::TubeInformedSampler>(start_conf, goal_conf, lb, ub);
          pathplan::SamplerPtr sampler = std::make_shared<pathplan::InformedSampler>(start_conf, goal_conf, lb, ub);
          tube_sampler->setRadius(tube_radius);
          if (tube_sampler)
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
            if (iter>10000)
              ROS_INFO_THROTTLE(120,"iter=%d",iter);
            if (!ros::ok())
            {
              ROS_INFO("Externally stopped");
              return 0;
            }
            if ((ros::WallTime::now()-start_time).toSec()>300)
              continue;

            if (tube_sampler && found_a_solution)
            {
              if (local_unoptimality)
              {
                tube_sampler->setLocalBias(1);
              }
              else
              {
                tube_sampler->setLocalBias(0);
              }
            }

            double current_cost=solver->cost();
            if (solver->update(solution))
            {
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

              if (tube_sampler)
              {
                //double delta_cost=1-solver->cost()/current_cost;
                double delta_cost=(current_cost-solver->cost())/(current_cost-utopy);
                tube_sampler->setPath(solution);
                tube_sampler->setRadius(tube_radius*(solution->cost()-utopy));
                local_probability=std::min(1.,forgetting_factor*local_probability+(1.0-forgetting_factor)*(delta_cost));
              }
            }
            else
            {
              if (tube_sampler)
              {
                if (used_tube_sampler)
                  local_probability=std::max(1e-10,forgetting_factor*local_probability);
              }
            }


            if (tube_sampler)
            {
              local_unoptimality=m_ud(m_gen)<local_probability;
            }

            if (found_a_solution)
            {
              ROS_DEBUG_THROTTLE(2,"time = %f, iter = %u, nodes=%u, local prob =%f, best solution = %f, global minimum = %f, local minimum = %f",(ros::WallTime::now()-start_time).toSec(),iter,solver->getStartTree()->getNumberOfNodes(),local_probability, best_solution->cost(), global_minimum,local_minimum);
            }

            if (!save_full_logs &&
                found_a_solution &&
                ( (best_solution->cost()<(global_minimum*exit_tolerance)) || iter==(maximum_iter-1)) )
            {
              double time=(ros::WallTime::now()-start_time).toSec();
              double iter_d=iter;
              m_file.write((char*) &time, sizeof(double));
              m_file.write((char*) &iter_d, sizeof(double));
              double nodes=solver->getStartTree()->getNumberOfNodes();
              m_file.write((char*) &nodes, sizeof(double));
              m_file.write((char*) &(local_probability), sizeof(double));
              if (found_a_solution)
                m_file.write((char*) &(best_solution->cost()), sizeof(double));
              else
                m_file.write((char*) &(cost_before_refining), sizeof(double));
            }
            if (found_a_solution)
              if (best_solution->cost()<(global_minimum*exit_tolerance))
                break;
          }



          if (!found_a_solution)
          {
            ROS_ERROR("unable to find a valid path");
            m_file.close();
            continue;
          }

          ROS_DEBUG("time = %f, iter = %u",(ros::WallTime::now()-start_time).toSec(),iter);
          ROS_DEBUG("Refinement of the solution form %f to %f, global minimum = %f, local minimum = %f.",cost_before_refining,best_solution->cost(), global_minimum,local_minimum);
          ROS_INFO("DOF %d:  TRIAL %u of %d, cost=%f, global minimum = %f, local minimum = %f.",dof, itrial+1,trials,best_solution->cost(), global_minimum,local_minimum);


        }
      }

      ROS_INFO("Executed in %f seconds",(ros::WallTime::now()-t0).toSec());
      if (!tube_sampler)
        return 0;
    }
  }

  return 0;
}
