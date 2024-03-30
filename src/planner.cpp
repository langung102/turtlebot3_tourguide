#include <cmath>
#include <iostream>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/compute_path_to_pose.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2_ros/transform_broadcaster.h"

using namespace std::chrono_literals;

class PathPlanningClient : public rclcpp::Node {
public:
    PathPlanningClient() : Node("path_planning_client") {
        compute_path_to_pose_client_ = rclcpp_action::create_client<nav2_msgs::action::ComputePathToPose>(
            this, "compute_path_to_pose");
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    }

    void requestPath() {
        auto start_pose = std::make_shared<geometry_msgs::msg::PoseStamped>();
        start_pose->header.frame_id = "map";
        start_pose->header.stamp = now();
        start_pose->pose.position.x = 1.0;
        start_pose->pose.position.y = 1.0;
        start_pose->pose.position.z = 0.0;
        start_pose->pose.orientation.w = 1.0;
        start_pose->pose.orientation.x = 0.0;
        start_pose->pose.orientation.y = 0.0;
        start_pose->pose.orientation.z = 0.0;

        auto goal_pose = std::make_shared<geometry_msgs::msg::PoseStamped>();
        goal_pose->header.frame_id = "map";
        goal_pose->header.stamp = now();
        goal_pose->pose.position.x = 3.0;
        goal_pose->pose.position.y = 0.0;
        goal_pose->pose.position.z = 0.0;
        goal_pose->pose.orientation.w = 1.0;
        goal_pose->pose.orientation.x = 0.0;
        goal_pose->pose.orientation.y = 0.0;
        goal_pose->pose.orientation.z = 0.0;

        auto path = getPath(start_pose, goal_pose);
        if (path != nullptr) {
            processPath(path);
        } else {
            RCLCPP_ERROR(get_logger(), "Failed to get path.");
        }
    }

    void processPath(const nav_msgs::msg::Path::SharedPtr path) {
        auto prev_pose = std::make_shared<geometry_msgs::msg::PoseStamped>();
        float path_length = 0;
        for (const auto &pose : path->poses) {
            if (prev_pose != nullptr) {
                float dx = pose.pose.position.x - prev_pose->pose.position.x;
                float dy = pose.pose.position.y - prev_pose->pose.position.y;
                float segment_length = std::sqrt(dx * dx + dy * dy);
                path_length += segment_length;
            }
            prev_pose = std::make_shared<geometry_msgs::msg::PoseStamped>(pose);
        }

        std::cout << "Length of the global path: " << std::fixed << std::setprecision(2)
                  << path_length << " units" << std::endl;
    }

private:
    rclcpp_action::Client<nav2_msgs::action::ComputePathToPose>::SharedPtr compute_path_to_pose_client_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    rclcpp::Time now() const {
        return now();
    }

    std::shared_ptr<nav_msgs::msg::Path> getPath(const std::shared_ptr<geometry_msgs::msg::PoseStamped> &start,
                                                   const std::shared_ptr<geometry_msgs::msg::PoseStamped> &goal) {
        while (!compute_path_to_pose_client_->wait_for_action_server(1s)) {
            RCLCPP_INFO(get_logger(), "Waiting for 'ComputePathToPose' action server...");
        }

        auto goal_msg = nav2_msgs::action::ComputePathToPose::Goal();
        goal_msg.start = *start;
        goal_msg.goal = *goal;

        RCLCPP_INFO(get_logger(), "Getting path...");
        auto send_goal_future = compute_path_to_pose_client_->async_send_goal(goal_msg);
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), send_goal_future) !=
            rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_ERROR(get_logger(), "Failed to get path.");
            return nullptr;
        }

        auto goal_handle = send_goal_future.get();
        if (!goal_handle) {
            RCLCPP_ERROR(get_logger(), "Goal was rejected.");
            return nullptr;
        }

        auto result_future = compute_path_to_pose_client_->async_get_result(goal_handle);
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) !=
            rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_ERROR(get_logger(), "Failed to get result.");
            return nullptr;
        }

        auto result = result_future.get();
        // if (result.code != nav2_msgs::action::ComputePathToPose::Result::STATUS_SUCCEEDED) {
        //     RCLCPP_ERROR(get_logger(), "Getting path failed with status code: %d", result->status);
        //     return nullptr;
        // }

        return std::make_shared<nav_msgs::msg::Path>(result.result->path);
    }
};

// int main(int argc, char **argv) {
//     rclcpp::init(argc, argv);
//     auto path_planning_client = std::make_shared<PathPlanningClient>();
//     path_planning_client->requestPath();
//     rclcpp::shutdown();
//     return 0;
// }
