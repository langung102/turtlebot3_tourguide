#include "update_handler.hpp"

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
    this->yaw_degree = yaw * 180.0 / M_PI

    setPosition(this->x, this->y, this->yaw_degree);
    RCLCPP_INFO(get_logger(), "%f - %f - %f", this->x, this->y, this->yaw_degree);
}

BatterySubscriber::BatterySubscriber() : Node("battery_state_subscriber")
{
    subscription_ = this->create_subscription<sensor_msgs::msg::BatteryState>(
        "battery_state",
        10,
        [this](const sensor_msgs::msg::BatteryState::SharedPtr msg)
        {
            processBattery(msg);
        });
}

void BatterySubscriber::processBattery(sensor_msgs::msg::BatteryState::SharedPtr msg)
{
    this->battery = (int) msg->percentage;
    SetBattery(this->battery);
    RCLCPP_INFO(get_logger(), "Battery: %d", this->battery);
}