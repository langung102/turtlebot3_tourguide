#include "path_planning_client.hpp"

std::vector<int> travllingSalesmanProblem(std::vector<std::vector<double>> graph, int s)
{
    std::vector<int> vertex;
    for (int i = 0; i < graph.size(); i++)
        if (i != s)
            vertex.push_back(i);

    std::vector<int> min_path_vertices;
    double min_path_weight = std::numeric_limits<double>::max();
    do
    {
        double current_pathweight = 0;
        double k = s;
        std::vector<int> current_path_vertices;
        current_path_vertices.push_back(s);
        for (int i = 0; i < vertex.size(); i++)
        {
            current_pathweight += graph[k][vertex[i]];
            k = vertex[i];
            current_path_vertices.push_back(k);
        }
        current_pathweight += graph[k][s];
        current_path_vertices.push_back(s);

        if (current_pathweight < min_path_weight)
        {
            min_path_weight = current_pathweight;
            min_path_vertices = current_path_vertices;
        }

    } while (std::next_permutation(vertex.begin(), vertex.end()));

    std::cout << "Minimum Path Weight: " << min_path_weight << std::endl;
    return min_path_vertices;
}

PathPlanningClient::PathPlanningClient() : Node("path_planning_client")
{
    compute_path_to_pose_client_ = rclcpp_action::create_client<nav2_msgs::action::ComputePathToPose>(
        this, "compute_path_to_pose");
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    auto pose1 = std::make_shared<geometry_msgs::msg::PoseStamped>();
    pose1->header.frame_id = "map";
    pose1->header.stamp = now();
    pose1->pose.position.x = 1.0;
    pose1->pose.position.y = 0.0;
    pose1->pose.position.z = 0.0;
    pose1->pose.orientation.w = 1.0;
    pose1->pose.orientation.x = 0.0;
    pose1->pose.orientation.y = 0.0;
    pose1->pose.orientation.z = 0.0;

    auto pose2 = std::make_shared<geometry_msgs::msg::PoseStamped>();
    pose2->header.frame_id = "map";
    pose2->header.stamp = now();
    pose2->pose.position.x = 4.0;
    pose2->pose.position.y = 1.0;
    pose2->pose.position.z = 0.0;
    pose2->pose.orientation.w = 1.0;
    pose2->pose.orientation.x = 0.0;
    pose2->pose.orientation.y = 0.0;
    pose2->pose.orientation.z = 0.0;

    auto pose3 = std::make_shared<geometry_msgs::msg::PoseStamped>();
    pose3->header.frame_id = "map";
    pose3->header.stamp = now();
    pose3->pose.position.x = 2.5;
    pose3->pose.position.y = 2.5;
    pose3->pose.position.z = 0.0;
    pose3->pose.orientation.w = 1.0;
    pose3->pose.orientation.x = 0.0;
    pose3->pose.orientation.y = 0.0;
    pose3->pose.orientation.z = 0.0;

    auto pose4 = std::make_shared<geometry_msgs::msg::PoseStamped>();
    pose4->header.frame_id = "map";
    pose4->header.stamp = now();
    pose4->pose.position.x = 2.5;
    pose4->pose.position.y = -1.0;
    pose4->pose.position.z = 0.0;
    pose4->pose.orientation.w = 1.0;
    pose4->pose.orientation.x = 0.0;
    pose4->pose.orientation.y = 0.0;
    pose4->pose.orientation.z = 0.0;

    std::vector<std::shared_ptr<geometry_msgs::msg::PoseStamped>> allposes = {pose1, pose2, pose3, pose4};

    this->requestPath(allposes);
}

void PathPlanningClient::requestPath(const std::vector<std::shared_ptr<geometry_msgs::msg::PoseStamped>> &poses)
{
    std::vector<std::vector<double>> distances(poses.size(), std::vector<double>(poses.size()));
    std::vector<std::vector<nav_msgs::msg::Path::SharedPtr>> paths(poses.size(), std::vector<nav_msgs::msg::Path::SharedPtr>(poses.size()));

    for (size_t i = 0; i < poses.size(); ++i)
    {
        for (size_t j = i + 1; j < poses.size(); ++j)
        {
            // Call getPath with the current pair of poses
            paths[i][j] = getPath(poses[i], poses[j]);
            distances[i][j] = processPath(paths[i][j]);
            distances[j][i] = distances[i][j];
            std::cout << "Path from pose " << poses[i]->pose.position.x << "," << poses[i]->pose.position.y << " to pose " << poses[j]->pose.position.x << "," << poses[j]->pose.position.y << ": " << processPath(paths[i][j]) << std::endl;
        }
    }
    for (size_t i = 0; i < poses.size(); ++i)
    {
        for (size_t j = 0; j < poses.size(); ++j)
        {
            std::cout << distances[i][j] << " ";
        }
        std::cout << std::endl;
    }

    std::vector<int> optimized_path = travllingSalesmanProblem(distances, 0);

    std::cout << "Minimum Path: ";
    for (int vertex : optimized_path)
        std::cout << vertex << " ";
    std::cout << std::endl;

    for (int vertex : optimized_path)
    {
        while (!nav.doneNavigate());
        std::cout << "navigating to: " << poses[vertex]->pose.position.x << " " << poses[vertex]->pose.position.y << std::endl;
        nav.startNavigation(*poses[vertex]);
    }
    std::cout << std::endl;

    // for (const auto& path : allPossiblePaths) {
    //     for (int node : path) {
    //         std::cout << node << " ";
    //     }
    //     std::cout << std::endl;
    // }

    // if (path != nullptr)
    // {
    //     processPath(path);
    // }
    // else
    // {
    //     RCLCPP_ERROR(get_logger(), "Failed to get path.");
    // }
}

double PathPlanningClient::processPath(const nav_msgs::msg::Path::SharedPtr path)
{
    auto prev_pose = std::make_shared<geometry_msgs::msg::PoseStamped>();
    float path_length = 0;
    for (const auto &pose : path->poses)
    {
        if (prev_pose != nullptr)
        {
            float dx = pose.pose.position.x - prev_pose->pose.position.x;
            float dy = pose.pose.position.y - prev_pose->pose.position.y;
            float segment_length = std::sqrt(dx * dx + dy * dy);
            path_length += segment_length;
        }
        prev_pose = std::make_shared<geometry_msgs::msg::PoseStamped>(pose);
    }

    // std::cout << "Length of the global path: " << std::fixed << std::setprecision(2)
    //           << path_length << " units" << std::endl;
    return path_length;
}

rclcpp_action::Client<nav2_msgs::action::ComputePathToPose>::SharedPtr compute_path_to_pose_client_;
std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

std::shared_ptr<nav_msgs::msg::Path> PathPlanningClient::getPath(const std::shared_ptr<geometry_msgs::msg::PoseStamped> &start,
                                                                 const std::shared_ptr<geometry_msgs::msg::PoseStamped> &goal)
{
    while (!compute_path_to_pose_client_->wait_for_action_server(1s))
    {
        RCLCPP_INFO(get_logger(), "Waiting for 'ComputePathToPose' action server...");
    }

    auto goal_msg = nav2_msgs::action::ComputePathToPose::Goal();
    goal_msg.start = *start;
    goal_msg.goal = *goal;
    goal_msg.use_start = true;

    RCLCPP_INFO(get_logger(), "Getting path...");
    auto send_goal_future = compute_path_to_pose_client_->async_send_goal(goal_msg);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), send_goal_future) !=
        rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(get_logger(), "Failed to get path.");
        return nullptr;
    }

    auto goal_handle = send_goal_future.get();
    if (!goal_handle)
    {
        RCLCPP_ERROR(get_logger(), "Goal was rejected.");
        return nullptr;
    }

    auto result_future = compute_path_to_pose_client_->async_get_result(goal_handle);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) !=
        rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(get_logger(), "Failed to get result.");
        return nullptr;
    }

    auto result = result_future.get();
    if (result.code != rclcpp_action::ResultCode::SUCCEEDED)
    {
        RCLCPP_ERROR(get_logger(), "Getting path failed with status code: %d", result.code);
        return nullptr;
    }

    return std::make_shared<nav_msgs::msg::Path>(result.result->path);
}