#ifndef PLANNER
#define PLANNER

#include <cmath>
#include <iostream>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/compute_path_to_pose.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "navigation_client.hpp"
#include "global.hpp"
#include <algorithm>
#include <cmath>
#include <vector>
#include <time.h>

using namespace std::chrono_literals;

class PathPlanningClient : public rclcpp::Node {
public:
    PathPlanningClient();
    void requestPath(const std::vector<std::shared_ptr<geometry_msgs::msg::PoseStamped>>& poses);
    double processPath(const nav_msgs::msg::Path::SharedPtr path);
private:
    rclcpp_action::Client<nav2_msgs::action::ComputePathToPose>::SharedPtr compute_path_to_pose_client_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<nav_msgs::msg::Path> getPath(const std::shared_ptr<geometry_msgs::msg::PoseStamped> &start,
                                                   const std::shared_ptr<geometry_msgs::msg::PoseStamped> &goal);
    NavigationClient nav;
};

#endif PLANNER