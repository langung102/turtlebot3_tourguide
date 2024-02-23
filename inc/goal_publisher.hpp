#ifndef PUBLISHER
#define PUBLISHER

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "global.hpp"
#include "turtlebot3_firebase.hpp"

class AmclPoseSubscriber : public rclcpp::Node
{
public:
    AmclPoseSubscriber();

private:
    void processAmclPose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
    double x;
    double y;
};

#endif