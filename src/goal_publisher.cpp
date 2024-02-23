#include "goal_publisher.hpp"

AmclPoseSubscriber::AmclPoseSubscriber() : Node("amcl_pose_subscriber")
{
    subscription_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "amcl_pose",
        10,
        [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
        {
            if (needPublishing)
                processAmclPose(msg);
        });
}

void AmclPoseSubscriber::processAmclPose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
    this->x = msg->pose.pose.position.x;
    this->y = msg->pose.pose.position.y;
    RCLCPP_INFO(get_logger(), "%f - %f", this->x, this->y);
}