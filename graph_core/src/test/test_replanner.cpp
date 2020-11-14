#include <ros/ros.h>
#include <graph_core/metrics.h>
#include <graph_core/occupancy_metrics.h>
#include <graph_core/graph/node.h>
#include <graph_core/graph/path.h>
#include <graph_core/graph/tree.h>
#include <graph_core/solvers/rrt_connect.h>
#include <graph_core/solvers/birrt.h>
#include <graph_core/solvers/rrt_star.h>
#include <graph_core/solvers/path_solver.h>
#include <graph_core/moveit_collision_checker.h>
#include <graph_core/local_informed_sampler.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <rviz_visual_tools/rviz_visual_tools.h>
#include <eigen_conversions/eigen_msg.h>

#define COMMENT(...) ROS_LOG(::ros::console::levels::Debug, ROSCONSOLE_DEFAULT_NAME, __VA_ARGS__)

int main(int argc, char **argv)
{
    //std::string test = "sharework";
    std::string test = "panda";

    unsigned int n_paths = 3;

    ros::init(argc, argv, "node_replanner");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::NodeHandle nh;

    ros::Publisher display_publisher = nh.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("/marker_visualization_topic", 1);

    std::string group_name;
    if(test == "sharework")
    {
       group_name = "manipulator";
    }
    else
    {

       group_name = "panda_arm";
    }
    moveit::planning_interface::MoveGroupInterface move_group(group_name);
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
    planning_scene::PlanningScenePtr planning_scene = std::make_shared<planning_scene::PlanningScene>(kinematic_model);
    robot_state::RobotState start_state = planning_scene->getCurrentState();  //stato iniziale

    //moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(group_name); //kinematic_model->getJointModelGroup(group_name);
    std::vector<std::string> joint_names = joint_model_group->getActiveJointModelNames();

    unsigned int dof = joint_names.size();
    Eigen::VectorXd lb(dof);
    Eigen::VectorXd ub(dof);

    for (unsigned int idx = 0; idx < dof; idx++)
    {
        const robot_model::VariableBounds& bounds = kinematic_model->getVariableBounds(joint_names.at(idx));  //bounds dei joints definito in urdf e file joints limit
        if (bounds.position_bounded_)
        {
            lb(idx) = bounds.min_position_;
            ub(idx) = bounds.max_position_;
            ROS_FATAL("joint name =%s, bound = [%f, %f]", joint_names.at(idx).c_str(), lb(idx), ub(idx));
        }
    }

    Eigen::VectorXd start_conf(dof);
    if(test == "sharework")
    {
        start_conf << 0.0,0.0,0.0,0.0,0.0,0.0; //sharework
    }
    else
    {
        start_conf << 0.0,0.0,0.0,-1.5,0.0,1.5,0.50; //panda
    }

    Eigen::VectorXd goal_conf(dof);
    if(test == "sharework")
    {
      goal_conf << -2.4568206461416158, -3.2888890061467357, 0.5022868201292561, -0.3550610439856255, 2.4569204968195355, 3.1415111410868053; //sharework
    }
    else
    {
        goal_conf <<  1.5, 0.5, 0.0, -1.0, 0.0, 2.0, 1.0; //panda
    }

    pathplan::NodePtr start_node = std::make_shared<pathplan::Node>(start_conf);
    pathplan::NodePtr goal_node = std::make_shared<pathplan::Node>(goal_conf);

    pathplan::MetricsPtr metrics = std::make_shared<pathplan::Metrics>();
    pathplan::CollisionCheckerPtr checker = std::make_shared<pathplan::MoveitCollisionChecker>(planning_scene, group_name);
    // ////////////////////////////////////////////////////////////////////////PATH PLAN/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    for (unsigned int i =0; i<n_paths; i++)
    {
        pathplan::SamplerPtr sampler = std::make_shared<pathplan::InformedSampler>(start_conf, goal_conf, lb, ub);

        ROS_INFO("Test solver");  pathplan::BiRRT solver(metrics, checker, sampler);
        solver.config(nh);
        solver.addGoal(goal_node);
        solver.addStart(start_node);

        pathplan::PathPtr solution;
        if (!solver.solve(solution, 1000))
        {
            ROS_INFO("No solutions found");
            return 0;
        }

        double rrt_cost = solution->cost();
        ROS_INFO_STREAM("found a solution with cost = " << solution->cost());

        pathplan::PathLocalOptimizer path_solver(checker, metrics);
        path_solver.config(nh);

        solution->setTree(solver.getStartTree());
        path_solver.setPath(solution);
        path_solver.solve(solution);
        ROS_INFO_STREAM("improve the solution to cost = " << solution->cost());

        sampler->setCost(solution->cost());
        solver.getStartTree()->addBranch(solution->getConnections());

        pathplan::LocalInformedSamplerPtr local_sampler = std::make_shared<pathplan::LocalInformedSampler>(start_conf, goal_conf, lb, ub);

        for (unsigned int isol = 0; isol < solution->getConnections().size() - 1; isol++)
        {
            pathplan::ConnectionPtr conn = solution->getConnections().at(isol);
            if (conn->getChild()->getConfiguration().size() != dof)
            {
                ROS_ERROR_STREAM("size = " << conn->getChild()->getConfiguration().size());
                ROS_WARN_STREAM("node " << conn->getChild());
            }
            try
            {
                local_sampler->addBall(conn->getChild()->getConfiguration(), solution->cost() * 0.1);
            }
            catch (...)
            {
                ROS_ERROR_STREAM("size = " << conn->getChild()->getConfiguration().size());
                ROS_WARN_STREAM("node " << conn->getChild());
                return 0;
            }
        }
        local_sampler->setCost(solution->cost());

        ROS_INFO("Improving solution with RRT*");
        pathplan::RRTStar opt_solver(metrics, checker, local_sampler);
        opt_solver.addStartTree(solver.getStartTree());
        opt_solver.addGoal(goal_node);
        opt_solver.config(nh);

        std::vector<pathplan::NodePtr> white_list;
        white_list.push_back(goal_node);

        ros::Duration max_time(3);
        ros::Time t0 = ros::Time::now();

        int stall_gen = 0;
        int max_stall_gen = 200;

        std::mt19937 gen;
        std::uniform_int_distribution<> id = std::uniform_int_distribution<>(0, max_stall_gen);

        for (unsigned int idx = 0; idx < 1000; idx++)
        {
            if (ros::Time::now() - t0 > max_time)
                break;

            if (opt_solver.update(solution))
            {
                stall_gen = 0;
                ROS_INFO_STREAM("Iteration = " << idx << " , rrt* reduce cost to = " << solution->cost());
                path_solver.setPath(solution);
                solution->setTree(opt_solver.getStartTree());
                //      path_solver.solve(solution);
                ROS_INFO_STREAM("local solver improve the solution to cost = " << solution->cost());

                local_sampler->setCost(solution->cost());
                sampler->setCost(solution->cost());
                opt_solver.getStartTree()->purgeNodes(sampler, white_list, true);

                local_sampler->clearBalls();
                for (unsigned int isol = 0; isol < solution->getConnections().size() - 1; isol++)
                {
                    pathplan::ConnectionPtr conn = solution->getConnections().at(isol);
                    if (conn->getChild()->getConfiguration().size() != dof)
                    {
                        ROS_ERROR_STREAM("size = " << conn->getChild()->getConfiguration().size());
                        ROS_WARN_STREAM("node " << conn->getChild());
                    }
                    //if (conn->getCost()>1.01*conn->norm())
                    {
                        try
                        {
                            local_sampler->addBall(conn->getChild()->getConfiguration(), solution->cost() * 0.1);
                        }
                        catch (...)
                        {
                            ROS_ERROR_STREAM("size = " << conn->getChild()->getConfiguration().size());
                            ROS_WARN_STREAM("node " << conn->getChild());
                            return 0;
                        }
                    }
                }
            }
            else
            {
                opt_solver.getStartTree()->purgeNodes(sampler, white_list, false);
                stall_gen++;
            }


            if (idx % 10 == 0)
                ROS_INFO("iter=%u,tree with %u nodes", idx, opt_solver.getStartTree()->getNumberOfNodes());

            if (id(gen) < stall_gen)
                opt_solver.setSampler(sampler);
            else
                opt_solver.setSampler(local_sampler);

            if (stall_gen >= max_stall_gen)
                break;
        }

        double opt_cost = solution->cost();
        ROS_INFO_STREAM("solution cost = " << solution->cost());
        ROS_INFO("tree with %u nodes", opt_solver.getStartTree()->getNumberOfNodes());

        path_solver.setPath(solution);
        path_solver.solve(solution);
        ROS_INFO_STREAM("local solver improve the solution to cost = " << solution->cost());

        ROS_INFO_STREAM("solution\n" << *solution);
        ros::Duration(0.1).sleep();

    // /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        COMMENT("Get waypoints");
        std::vector<Eigen::VectorXd> waypoints=solution->getWaypoints();

        //Definizione della traiettoria, noti i waypoints del path
        robot_trajectory::RobotTrajectoryPtr trj = std::make_shared<robot_trajectory::RobotTrajectory>(kinematic_model,group_name);

        COMMENT("processing %zu waypoints..", waypoints.size());
        std::vector<moveit::core::RobotState> wp_state_vector;
        for (const Eigen::VectorXd& waypoint: waypoints )
        {
            COMMENT("processing waypoint");
            moveit::core::RobotState wp_state=start_state;
            wp_state.setJointGroupPositions(group_name,waypoint);
            wp_state.update();
            trj->addSuffixWayPoint(wp_state,0); //non gli do la time parametrization

            wp_state_vector.push_back(wp_state);
        }

        //Trasformo la traiettoria in msg inviabile
        moveit_msgs::RobotTrajectory trj_msg;
        trj->getRobotTrajectoryMsg(trj_msg);

        moveit_msgs::DisplayTrajectory disp_trj;
        disp_trj.trajectory.push_back(trj_msg);
        disp_trj.model_id=kinematic_model->getName();
        moveit::core::robotStateToRobotStateMsg(start_state,disp_trj.trajectory_start);

        /*Visualize the trajectory*/
        namespace rvt = rviz_visual_tools;   //PERCHE' LO SCRIVONO TUTTI?
        moveit_visual_tools::MoveItVisualToolsPtr visual_tools;
        std::string topic = "/rviz_visual_tools";
        if(test == "sharework")
        {
            visual_tools.reset(new moveit_visual_tools::MoveItVisualTools("base_link",topic));
        }
        else
        {
            visual_tools.reset(new moveit_visual_tools::MoveItVisualTools("panda_link0",topic));
        }
        visual_tools->loadRobotStatePub("/display_robot_state");
        visual_tools->enableBatchPublishing();
        if(i==0){visual_tools->deleteAllMarkers();}
        visual_tools->trigger();

        /* Remote control is an introspection tool that allows users to step through a high level script
     via buttons and keyboard shortcuts in RViz */
        visual_tools->loadRemoteControl();

        /* We can also use visual_tools to wait for user input */
        visual_tools->prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");

        std::vector<rviz_visual_tools::colors> colors {rviz_visual_tools::GREEN, rviz_visual_tools::BLUE, rviz_visual_tools::RED};

        visual_tools->publishRobotState(planning_scene->getCurrentStateNonConst(), colors.at(i));
        visual_tools->trigger();

        ROS_INFO("Stai per pubblicare");
        visual_tools->publishTrajectoryLine(disp_trj.trajectory.back(), joint_model_group); //non dovrebbe pubblicare lei la linea della traiettoria?
        ros::Duration(0.1).sleep();
        ROS_INFO("pubblicato");
        visual_tools->trigger();

        display_publisher.publish(disp_trj);  //to display the robot following the trj
        visual_tools->trigger();

        //  ////////////////////////////////

        ROS_INFO("Pubishing markers....");
        // Set our initial shape type
        uint32_t shape = visualization_msgs::Marker::LINE_STRIP;
        //uint32_t shape = visualization_msgs::Marker::SPHERE;

        visualization_msgs::Marker marker;
        marker.type = shape;

        for(unsigned int t=0; t<wp_state_vector.size(); t++)
        {
            if(shape == visualization_msgs::Marker::SPHERE)
            {
                if(test == "sharework")
                {
                    tf::poseEigenToMsg(wp_state_vector.at(t).getGlobalLinkTransform("open_tip"),marker.pose);
                }
                else
                {
                    tf::poseEigenToMsg(wp_state_vector.at(t).getGlobalLinkTransform("panda_link8"),marker.pose);

                }

                marker.header.frame_id="world";
                marker.header.stamp=ros::Time::now();
                marker.action = visualization_msgs::Marker::ADD;
                marker.id=pow(t+1,i+1);  //per avere id sempre diversi e non perdere marker (t+1)^(i+1)  (+1 per non avere 0)

                marker.scale.x = 0.1;
                marker.scale.y = 0.1;
                marker.scale.z = 0.1;

                // Set the color -- be sure to set alpha to something non-zero!
                if(i==0)
                {

                }
                else if (i==1)
                {
                    marker.color.r = 0.0f;
                    marker.color.g = 0.0f;
                    marker.color.b = 1.0;
                    marker.color.a = 1.0;
                }
                else if( i==2)
                {
                    marker.color.r = 1.0;
                    marker.color.g = 0.0f;
                    marker.color.b = 0.0f;
                    marker.color.a = 1.0;
                }


                marker_pub.publish(marker);
                ROS_INFO_STREAM("marker \"sphere\" published\n"<<marker);
                ros::Duration(0.1).sleep();
            }
            if(shape == visualization_msgs::Marker::LINE_STRIP)
            {
                geometry_msgs::Pose pose;
                geometry_msgs::Point p;

                if(test == "sharework")
                {
                    //tf::poseEigenToMsg(wp_state_vector.at(i).getGlobalLinkTransform("open_tip"),pose);  //sharework
                    tf::poseEigenToMsg(wp_state_vector.at(t).getGlobalLinkTransform("ee_link"),pose);

                }
                else
                {
                    tf::poseEigenToMsg(wp_state_vector.at(t).getGlobalLinkTransform("panda_link8"),pose);

                }
                p = pose.position;
                marker.points.push_back(p);
            }

        }

        if(shape == visualization_msgs::Marker::LINE_STRIP)
        {
            marker.header.frame_id="world";
            marker.header.stamp=ros::Time::now();
            marker.action = visualization_msgs::Marker::ADD;
            marker.id= - i;

            marker.scale.x = 0.01;
            marker.color.a = 1.0;

            if(i==0){marker.color.g = 1.0;}
            else if(i==1){marker.color.b = 1.0;}
            else if(i==2){marker.color.r = 1.0;}

            marker_pub.publish(marker);
            ROS_INFO_STREAM("marker \"line\" published\n"<<marker);
            ros::Duration(0.1).sleep();
        }
    }
    return 0;
}
