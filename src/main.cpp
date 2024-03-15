#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executor.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include <iostream>
#include "publisher.hpp"
#include "update_handler.hpp"
#include "global.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node_sub = std::make_shared<AmclPoseSubscriber>();
    auto node_pub = std::make_shared<RequestHandler>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node_sub);
    executor.add_node(node_pub);
    executor.spin();
}