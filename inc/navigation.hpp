#ifndef NAVIGATION
#define NAVIGATION

#include "iostream"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_lifecycle_manager/lifecycle_manager_client.hpp"
#include "nav2_msgs/action/navigate_through_poses.hpp"
#include "nav2_msgs/action/follow_waypoints.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "nav2_util/geometry_utils.hpp"
#include <action_msgs/msg/goal_status.hpp>

using namespace std::chrono_literals;

class NavigateToGoal {
public:
    NavigateToGoal();
    void startNavigation(geometry_msgs::msg::PoseStamped);
    void cancelNavigation();
    bool doneNavigate();
    // bool isGoalReached();

private:
    using NavigationGoalHandle = rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>;
    rclcpp::Node::SharedPtr client_node_;
    nav2_msgs::action::NavigateToPose::Goal navigation_goal_;
    NavigationGoalHandle::SharedPtr navigation_goal_handle_;
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr navigation_action_client_;
    std::chrono::milliseconds server_timeout_;
};

#endif