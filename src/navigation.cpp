#include "navigation.hpp"

NavigateToGoal::NavigateToGoal() : server_timeout_(100)
{
    auto options = rclcpp::NodeOptions().arguments(
        {"--ros-args", "--remap", "__node:=navigation_dialog_action_client", "--"});
    client_node_ = std::make_shared<rclcpp::Node>("_", options);
    navigation_goal_ = nav2_msgs::action::NavigateToPose::Goal();
    navigation_action_client_ =
        rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
            client_node_,
            "navigate_to_pose");
}

void NavigateToGoal::startNavigation(geometry_msgs::msg::PoseStamped pose)
{
    auto is_action_server_ready =
        navigation_action_client_->wait_for_action_server(std::chrono::seconds(5));
    if (!is_action_server_ready)
    {
        RCLCPP_ERROR(
            client_node_->get_logger(),
            "navigate_to_pose action server is not available."
            " Is the initial pose set?");
        return;
    }

    // Send the goal pose
    navigation_goal_.pose = pose;

    RCLCPP_INFO(
        client_node_->get_logger(),
        "NavigateToPose will be called using the BT Navigator's default behavior tree.");

    // Enable result awareness by providing an empty lambda function
    auto send_goal_options =
        rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
    send_goal_options.result_callback = [this](auto)
    {
        navigation_goal_handle_.reset();
    };

    auto future_goal_handle =
        navigation_action_client_->async_send_goal(navigation_goal_, send_goal_options);
    if (rclcpp::spin_until_future_complete(client_node_, future_goal_handle, server_timeout_) !=
        rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(client_node_->get_logger(), "Send goal call failed");
        return;
    }

    // Get the goal handle and save so that we can check on completion in the timer callback
    navigation_goal_handle_ = future_goal_handle.get();
    if (!navigation_goal_handle_)
    {
        RCLCPP_ERROR(client_node_->get_logger(), "Goal was rejected by server");
        return;
    }
}

void NavigateToGoal::cancelNavigation()
{
    if (this->navigation_goal_handle_)
    {
        auto future_cancel = this->navigation_action_client_->async_cancel_goal(this->navigation_goal_handle_);

        if (rclcpp::spin_until_future_complete(client_node_, future_cancel, this->server_timeout_) !=
            rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_ERROR(this->client_node_->get_logger(), "Failed to cancel goal");
        }
        else
        {
            this->navigation_goal_handle_.reset();
        }
    }
}

bool NavigateToGoal::doneNavigate() {
    rclcpp::spin_some(this->client_node_);
    // std::cout << (int) navigation_goal_handle_->get_status() << std::endl;
    return (int) navigation_goal_handle_->get_status() == 4;
    // return status = action_msgs::msg::GoalStatus::STATUS_ACCEPTED ||
    //     status == action_msgs::msg::GoalStatus::STATUS_EXECUTING;
}

// bool NavigateToGoal::isGoalReached() {
//     return this->goal_checker.isGoalReached();
// }