#ifndef REQUEST_HANDLING
#define REQUEST_HANDLING

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include <iostream>
#include "global.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "navigation.hpp"
#include "turtlebot3_firebase.hpp"

class RequestHandler : public rclcpp::Node, public firebase::database::ValueListener
{
public:
    RequestHandler();
    ~RequestHandler();
private:
    void handlerCallback();
    void getInput();
    void OnValueChanged(const firebase::database::DataSnapshot &snapshot) override;
    void OnCancelled(const firebase::database::Error &error_code, const char *error_message) override;
    firebase::database::DatabaseReference dbref;
    firebase::database::Database *database;
    NavigateToGoal nav;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer;
};

#endif