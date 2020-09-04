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

int main(int argc, char **argv)
{
  ros::init(argc, argv, "narrow_pass_test");
  ros::NodeHandle nh("~");

  bool m_tube_sampler=true;
  if (!nh.getParam("tube_sampler",m_tube_sampler))
  {
    ROS_ERROR("tube_sampler is not set");
    return 0;
  }

  std::string dirrt_log="narrow_pass";
  if (m_tube_sampler)
    dirrt_log+="__tube";
  else
    dirrt_log+="__ellipse";

  double cylinder_radius=1;
  if (!nh.getParam("cylinder_radius",cylinder_radius))
  {
    ROS_ERROR("cylinder_radius is not set");
    return 0;
  }
  dirrt_log+="__cylinder_radius"+std::to_string(cylinder_radius);

  double cylinder_width=1;
  if (!nh.getParam("cylinder_width",cylinder_width))
  {
    ROS_ERROR("cylinder_width is not set");
    return 0;
  }
  dirrt_log+="__cylinder_width"+std::to_string(cylinder_width);



  double measures_ratio=0.5;
  if (!nh.getParam("measures_ratio",measures_ratio))
  {
    ROS_ERROR("measures_ratio is not set");
    return 0;
  }
  dirrt_log+=std::string("__")+"measures_ratio"+std::to_string(measures_ratio);

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


  std::mt19937 m_gen(time(0));
  std::uniform_real_distribution<double> m_ud;

  int max_dof=10;
  if (!nh.getParam("max_dof",max_dof))
  {
    ROS_ERROR("max_dof is not set");
    return 0;
  }

  for (int dof=2;dof<=max_dof;dof++)
  {
    std::string log_name1=dirrt_log+"__dof"+std::to_string(dof);



    double hole_radius=cylinder_radius*std::pow(measures_ratio,1.0/(static_cast<double>(dof)-1.0));

    double b=cylinder_width*0.6;
    double a=(cylinder_radius+3*hole_radius)/4.0;

    double global_minimum = cylinder_width+2*std::sqrt(std::pow(0.1*cylinder_width,2)+std::pow(hole_radius-a,2));
    double local_minimum  = cylinder_width+2*std::sqrt(std::pow(0.1*cylinder_width,2)+std::pow(cylinder_radius-a,2));





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

    for (unsigned int itrial=0;itrial<trials;itrial++)
    {
      ROS_INFO("DOF %d:  TRIAL %u of %d",dof, itrial,trials);
      std::string log_name=log_name1+std::string("__")+"trial"+std::to_string(itrial);

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
          ROS_INFO_THROTTLE(2,"time = %f, iter = %u, nodes=%u, local prob =%f, best solution = %f, global minimum = %f, local minimum = %f",(ros::WallTime::now()-start_time).toSec(),iter,solver->getStartTree()->getNumberOfNodes(),local_probability, best_solution->cost(), global_minimum,local_minimum);
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

        if (found_a_solution)
          if (best_solution->cost()<(global_minimum*1.005))
            break;
      }



      if (!found_a_solution)
      {
        ROS_ERROR("unable to find a valid path");
        m_file.close();
        continue;
      }

      ROS_INFO("time = %f, iter = %u",(ros::WallTime::now()-start_time).toSec(),iter);
      ROS_INFO("Refinement of the solution form %f to %f, global minimum = %f, local minimum = %f.",cost_before_refining,best_solution->cost(), global_minimum,local_minimum);


    }
  }
  return 0;
}
