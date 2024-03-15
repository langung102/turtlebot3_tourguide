#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executor.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include <iostream>
#include "update_handler.hpp"
#include "request_handler.hpp"
#include "global.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node_amcl_pose = std::make_shared<AmclPoseSubscriber>();
    auto node_battery = std::make_shared<BatterySubscriber>();
    auto node_pub = std::make_shared<RequestHandler>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node_amcl_pose);
    executor.add_node(node_battery);
    executor.add_node(node_pub);
    executor.spin();
}