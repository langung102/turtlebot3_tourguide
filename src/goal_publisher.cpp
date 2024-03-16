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
    tf2::Quaternion quaternion;
    tf2::fromMsg(msg->pose.pose.orientation, quaternion);
    double roll, pitch, yaw;
    tf2::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);

    setPosition(x, y, yaw * 180.0 / M_PI);
    RCLCPP_INFO(get_logger(), "%f - %f - %f", this->x, this->y, yaw * 180.0 / M_PI);
}