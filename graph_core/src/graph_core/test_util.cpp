#include "graph_core/test_util.h"

namespace pathplan
{


TestUtil::TestUtil(ros::NodeHandle& nh,
                   robot_model::RobotModelPtr& kinematic_model,
                   planning_scene::PlanningScenePtr& planning_scene,
                   std::string& group_name,
                   //robot_state::RobotState& start_state,
                   const moveit::core::JointModelGroup *joint_model_group,
                   std::string& base_link,
                   std::string& last_link,
                   ros::Publisher& display_publisher,
                   ros::Publisher & marker_pub){
    nh_ = nh;
    kinematic_model_ = kinematic_model;
    planning_scene_ = planning_scene;
    group_name_ = group_name;
    //start_state_ = start_state;
    joint_model_group_ = joint_model_group;
    base_link_ = base_link;
    last_link_ = last_link;
    display_publisher_ = display_publisher;
    marker_pub_ = marker_pub;
}

PathPtr TestUtil::computeBiRRTPath(const NodePtr &start_node, NodePtr &goal_node, const Eigen::VectorXd& lb, const Eigen::VectorXd& ub, const MetricsPtr& metrics, const CollisionCheckerPtr& checker, const bool& optimizePath)
{
    pathplan::SamplerPtr sampler = std::make_shared<pathplan::InformedSampler>(start_node->getConfiguration(), goal_node->getConfiguration(), lb, ub);

    pathplan::BiRRT solver(metrics, checker, sampler);
    solver.config(nh_);
    solver.addGoal(goal_node);
    solver.addStart(start_node);

    pathplan::PathPtr solution;
    if (!solver.solve(solution, 1000))
    {
        ROS_INFO("No solutions found");
        assert(0);
    }

    //ROS_INFO_STREAM("found a solution with cost = " << solution->cost());

    if(optimizePath)
    {
        pathplan::PathLocalOptimizer path_solver(checker, metrics);
        path_solver.config(nh_);

        solution->setTree(solver.getStartTree());
        path_solver.setPath(solution);
        path_solver.solve(solution);
        //ROS_INFO_STREAM("improve the solution to cost = " << solution->cost());

        sampler->setCost(solution->cost());
        solver.getStartTree()->addBranch(solution->getConnections());

        pathplan::LocalInformedSamplerPtr local_sampler = std::make_shared<pathplan::LocalInformedSampler>(start_node->getConfiguration(), goal_node->getConfiguration(), lb, ub);

        for (unsigned int isol = 0; isol < solution->getConnections().size() - 1; isol++)
        {
            pathplan::ConnectionPtr conn = solution->getConnections().at(isol);
            if (conn->getChild()->getConfiguration().size() != start_node->getConfiguration().size())
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
                assert(0);
            }
        }
        local_sampler->setCost(solution->cost());

        //ROS_INFO("Improving solution with RRT*");
        pathplan::RRTStar opt_solver(metrics, checker, local_sampler);
        opt_solver.addStartTree(solver.getStartTree());
        opt_solver.addGoal(goal_node);
        opt_solver.config(nh_);

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
                //ROS_INFO_STREAM("Iteration = " << idx << " , rrt* reduce cost to = " << solution->cost());
                path_solver.setPath(solution);
                solution->setTree(opt_solver.getStartTree());
                //      path_solver.solve(solution);
                //ROS_INFO_STREAM("local solver improve the solution to cost = " << solution->cost());

                local_sampler->setCost(solution->cost());
                sampler->setCost(solution->cost());
                opt_solver.getStartTree()->purgeNodes(sampler, white_list, true);

                local_sampler->clearBalls();
                for (unsigned int isol = 0; isol < solution->getConnections().size() - 1; isol++)
                {
                    pathplan::ConnectionPtr conn = solution->getConnections().at(isol);
                    if (conn->getChild()->getConfiguration().size() != start_node->getConfiguration().size())
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
                            assert(0);
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
                //ROS_INFO("iter=%u,tree with %u nodes", idx, opt_solver.getStartTree()->getNumberOfNodes());

            if (id(gen) < stall_gen)
                opt_solver.setSampler(sampler);
            else
                opt_solver.setSampler(local_sampler);

            if (stall_gen >= max_stall_gen)
                break;
        }

        //ROS_INFO_STREAM("solution cost = " << solution->cost());
        //ROS_INFO("tree with %u nodes", opt_solver.getStartTree()->getNumberOfNodes());

        path_solver.setPath(solution);
        path_solver.solve(solution);
        //ROS_INFO_STREAM("local solver improve the solution to cost = " << solution->cost());

        ROS_INFO_STREAM("solution\n" << *solution);

    }

    return solution;
}

std::vector<moveit::core::RobotState> TestUtil::displayTrajectoryOnMoveitRviz(const PathPtr& solution, const std::vector<double> t_vector,  const rviz_visual_tools::colors color, const bool button)
{

    COMMENT("Get waypoints");
    std::vector<Eigen::VectorXd> waypoints=solution->getWaypoints();

    //Definizione della traiettoria, noti i waypoints del path
    robot_trajectory::RobotTrajectoryPtr trj = std::make_shared<robot_trajectory::RobotTrajectory>(kinematic_model_,group_name_);

    COMMENT("processing %zu waypoints..", waypoints.size());
    std::vector<moveit::core::RobotState> wp_state_vector;
    for(unsigned int j=0; j<waypoints.size();j++)
    {
        Eigen::VectorXd waypoint = waypoints.at(j);

        COMMENT("processing waypoint");
        moveit::core::RobotState wp_state=planning_scene_->getCurrentState();
        wp_state.setJointGroupPositions(group_name_,waypoint);
        wp_state.update();
        trj->addSuffixWayPoint(wp_state,t_vector.at(j)); //time parametrization

        wp_state_vector.push_back(wp_state);
    }

    //Trasformo la traiettoria in msg inviabile
    moveit_msgs::RobotTrajectory trj_msg;
    trj->getRobotTrajectoryMsg(trj_msg);

    moveit_msgs::DisplayTrajectory disp_trj;
    disp_trj.trajectory.push_back(trj_msg);
    disp_trj.model_id=kinematic_model_->getName();
    moveit::core::robotStateToRobotStateMsg(planning_scene_->getCurrentState(),disp_trj.trajectory_start);

    /*Visualize the trajectory*/
    namespace rvt = rviz_visual_tools;   //PERCHE' LO SCRIVONO TUTTI?
    moveit_visual_tools::MoveItVisualToolsPtr visual_tools;
    std::string topic = "/rviz_visual_tools";

    visual_tools.reset(new moveit_visual_tools::MoveItVisualTools(base_link_,topic));

    visual_tools->loadRobotStatePub("/display_robot_state");
    visual_tools->enableBatchPublishing();
    //if(i==0){visual_tools->deleteAllMarkers();}
    visual_tools->trigger();

    visual_tools->publishRobotState(planning_scene_->getCurrentStateNonConst(), color);
    visual_tools->trigger();

    visual_tools->publishTrajectoryLine(disp_trj.trajectory.back(), joint_model_group_, color); //non dovrebbe pubblicare lei la linea della traiettoria?
    visual_tools->trigger();

    display_publisher_.publish(disp_trj);  //to display the robot following the trj
    visual_tools->trigger();

    /* Remote control is an introspection tool that allows users to step through a high level script
    via buttons and keyboard shortcuts in RViz */
    visual_tools->loadRemoteControl();

    /* We can also use visual_tools to wait for user input */
    if(button){visual_tools->prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");}

    return wp_state_vector;
}

void TestUtil::displayPathNodesRviz(const std::vector<moveit::core::RobotState>& wp_state_vector, const uint32_t& shape, const std::vector<int>& marker_id, const std::vector<double>& marker_scale, const std::vector<double>& marker_color)
{
    //uint32_t shape = visualization_msgs::Marker::LINE_STRIP;
    //uint32_t shape = visualization_msgs::Marker::SPHERE;

    visualization_msgs::Marker marker;
    marker.type = shape;

    for(unsigned int t=0; t<wp_state_vector.size(); t++)
    {
        if(shape == visualization_msgs::Marker::SPHERE)
        {
            tf::poseEigenToMsg(wp_state_vector.at(t).getGlobalLinkTransform(last_link_),marker.pose);

            marker.header.frame_id="world";
            marker.header.stamp=ros::Time::now();
            marker.action = visualization_msgs::Marker::ADD;
            marker.id= marker_id.at(t);  //pow(t+1,i+1);  //per avere id sempre diversi e non perdere marker (t+1)^(i+1)  (+1 per non avere 0)

            marker.scale.x = marker_scale.at(0);
            marker.scale.y = marker_scale.at(1);
            marker.scale.z = marker_scale.at(2);

            // Set the color -- be sure to set alpha to something non-zero!

            marker.color.r = marker_color.at(0);
            marker.color.g = marker_color.at(1);
            marker.color.b = marker_color.at(2);
            marker.color.a = marker_color.at(3);

            marker_pub_.publish(marker);
            ros::Duration(0.1).sleep();
        }

        if(shape == visualization_msgs::Marker::LINE_STRIP)
        {
            geometry_msgs::Pose pose;
            geometry_msgs::Point p;

            tf::poseEigenToMsg(wp_state_vector.at(t).getGlobalLinkTransform(last_link_),pose);
            p = pose.position;
            marker.points.push_back(p);
        }

    }

    if(shape == visualization_msgs::Marker::LINE_STRIP)
    {
        marker.header.frame_id="world";
        marker.header.stamp=ros::Time::now();
        marker.action = visualization_msgs::Marker::ADD;
        marker.id= marker_id.back();

        marker.scale.x = marker_scale.at(0);

        marker.color.r = marker_color.at(0);
        marker.color.g = marker_color.at(1);
        marker.color.b = marker_color.at(2);
        marker.color.a = marker_color.at(3);

        marker_pub_.publish(marker);
        ros::Duration(0.1).sleep();
    }
}

}
