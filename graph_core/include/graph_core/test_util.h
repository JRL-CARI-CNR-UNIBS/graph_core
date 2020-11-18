#pragma once
#ifndef TEST_UTIL_14_11_2020_H
#define TEST_UTIL_14_11_2020_H

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
#include <graph_core/test_util.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <rviz_visual_tools/rviz_visual_tools.h>
#include <eigen_conversions/eigen_msg.h>

#define COMMENT(...) ROS_LOG(::ros::console::levels::Debug, ROSCONSOLE_DEFAULT_NAME, __VA_ARGS__);

namespace pathplan
{
    class TestUtil;
    typedef std::shared_ptr<TestUtil> TestUtilPtr;

    class TestUtil: public std::enable_shared_from_this<TestUtil>
    {
        protected:
        ros::NodeHandle nh_;
        robot_model::RobotModelPtr kinematic_model_;
        planning_scene::PlanningScenePtr planning_scene_;
        std::string group_name_;
        //robot_state::RobotState start_state_;
        const moveit::core::JointModelGroup *joint_model_group_;
        std::string base_link_;
        std::string last_link_;
        ros::Publisher display_publisher_;
        ros::Publisher marker_pub_;

        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW

            TestUtil(ros::NodeHandle &nh,
                     moveit::core::RobotModelPtr &kinematic_model,
                     planning_scene::PlanningScenePtr &planning_scene,
                     std::string &group_name,
                     //moveit::core::RobotState &start_state,
                     const moveit::core::JointModelGroup *joint_model_group,
                     std::string &base_link,
                     std::string &last_link,
                     ros::Publisher &display_publisher,
                     ros::Publisher &marker_pub);

            TestUtilPtr pointer()
            {
                return shared_from_this();
            }

            // Compute a path suing BiRRT from start_node to goal_node and then it is optimized if optimizePath==1 . nh is the NodeHandler of a ros node.
            PathPtr computeBiRRTPath(const NodePtr &start_node, NodePtr &goal_node, const Eigen::VectorXd& lb, const Eigen::VectorXd& ub, const MetricsPtr& metrics, const CollisionCheckerPtr& checker, const bool& optimizePath);

            // Display the robot executing the trajectory on Rviz; solution is the path, t_vector is the time vector to time parametrize the trj, start_state the initial state of the robot, display_publisher the publisher:  nh.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
            std::vector<moveit::core::RobotState> displayTrajectoryOnMoveitRviz(const PathPtr& solution, const std::vector<double> t_vector,  const rviz_visual_tools::colors color, const bool button);

            // Display marker( uint32_t shape = visualization_msgs::Marker::LINE_STRIP; or SPHERE) corresponding to the waypioints of a path, marker_pub is the publsiher: ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("/marker_visualization_topic", 1);
            void displayPathNodesRviz(const std::vector<moveit::core::RobotState>& wp_state_vector, const uint32_t& shape, const std::vector<int>& marker_id, const std::vector<double>& marker_scale, const std::vector<double>& marker_color);
    };
}

#endif // TEST_UTIL_H
