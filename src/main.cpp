#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executor.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include <iostream>
#include "update_handler.hpp"
#include "request_handler.hpp"
#include "path_planning_client.hpp"
#include "global.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    // rclcpp::shutdown();
    // speak("Starting programs");
    auto node_amcl_pose = std::make_shared<AmclPoseSubscriber>();
    auto node_battery = std::make_shared<BatterySubscriber>();
    // auto path_planning_client = std::make_shared<PathPlanningClient>();
    auto node_pub = std::make_shared<RequestHandler>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node_amcl_pose);
    executor.add_node(node_battery);
    executor.add_node(node_pub);
    // executor.add_node(path_planning_client);
    executor.spin();
    // speak("Program has shut down");
    return 0;
}

// #include <iostream>
// #include <vector>
// #include <algorithm>
// #include <limits.h>

// using namespace std;

// const int n = 4;

// // double dist[n][n] = {
// //     {0, 15.1, 10, 5},
// //     {15.1, 0, 25, 20},
// //     {10, 25, 0, 20},
// //     {5, 20, 20, 0}
// // };

// double dist[n][n] = {
//     {0, 4.6625, 4.41069, 3.31891},
//     {4.6625, 0, 6.34955, 6.92656},
//     {4.41069, 6.34955, 0, 7.04367},
//     {3.31891, 6.92656, 7.04367, 0}
// };

// vector<int> shortestPath;
// double minCost = INT_MAX;
// double memo[n][1 << n];

// double dp(int i, int mask) {
//     if (mask == (1 << n) - 1) { // All vertices have been visited
//         return dist[i][0]; // Return to vertex 1
//     }
//     if (memo[i][mask] != 0) {
//         return memo[i][mask];
//     }

//     double res = INT_MAX;

//     for (int j = 0; j < n; ++j) {
//         if (!(mask & (1 << j))) {
//             res = min(res, dp(j, mask | (1 << j)) + dist[i][j]);
//         }
//     }

//     return memo[i][mask] = res;
// }

// void printPath(int i, int mask) {
//     if (mask == (1 << n) - 1) {
//         shortestPath.push_back(0); // Starting vertex
//         return;
//     }

//     for (int j = 0; j < n; ++j) {
//         if (!(mask & (1 << j))) {
//             double nextCost = dp(j, mask | (1 << j)) + dist[i][j];
//             if (dp(i, mask) == nextCost) {
//                 shortestPath.push_back(j); // Add vertex j to the path
//                 printPath(j, mask | (1 << j)); // Recursive call with vertex j
//                 return;
//             }
//         }
//     }
// }

// int main() {
//     // int ans = dp(0, 1); // Start from vertex 1
//     printPath(0, 1); // Start from vertex 1

//     // cout << "The cost of the most efficient tour = " << ans << endl;
//     cout << "The vertices in the shortest path are: ";
//     for (int vertex : shortestPath) {
//         cout << vertex<< " "; // Add 1 to convert 0-indexed to 1-indexed vertex numbering
//     }
//     cout << endl;
//     return 0;
// }
