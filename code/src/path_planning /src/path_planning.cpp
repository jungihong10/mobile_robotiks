#include "../include/path_planning/path_planning.h"
#include <tf2/utils.h>
#include <tf2/convert.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <color_names/color_names.hpp>
#include <algorithm>
#include <cmath>
#include <math.h>

using std::placeholders::_1;

PathPlanner::PathPlanner(): Node("path_planner")
{
    // BIND planPath to goal subscriber not grid!!! -> change to static
    subMap = this->create_subscription<nav_msgs::msg::OccupancyGrid>("/Grid",
                                                                     10,
                                                                     std::bind(&PathPlanner::planPath, this, _1));
    subGoal = this->create_subscription<geometry_msgs::msg::PoseStamped>("/goal_pose",
                                                                         1,
                                                                         std::bind(&PathPlanner::goalPoseCallback, this, _1));
    subRobotPos = this->create_subscription<nav_msgs::msg::Odometry>("/odom",
                                                                     10,
                                                                     std::bind(&PathPlanner::robotPoseCallback, this, _1));

    pubPath = create_publisher<nav_msgs::msg::Path>("/Path", 10);
    pubGoal= create_publisher<visualization_msgs::msg::Marker>("goalVec", 10);
    pubCostMap = create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);


    declare_parameter("cost_weight", 1.0);
    declare_parameter("heuristic_weight", 1.0);
}

void PathPlanner::goalPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr goal)
{
   // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Received goal pose: (%f, %f, %f)",
                // goal->pose.position.x, goal->pose.position.y, goal->pose.position.z);
    goalX = goal->pose.position.x;
    goalY = goal->pose.position.y;
    return;
}

void PathPlanner::robotPoseCallback(const nav_msgs::msg::Odometry::SharedPtr odom)
{
    robotX = odom->pose.pose.position.x;
    robotY = odom->pose.pose.position.y;
    return;
}


void PathPlanner::publishGoalVector() {
    visualization_msgs::msg::Marker goalVec;
    goalVec.header.frame_id = "base_link";
    goalVec.header.stamp = get_clock()->now();
    goalVec.ns = "goal_vector";
    goalVec.id = 0;
    goalVec.type = visualization_msgs::msg::Marker::ARROW;
    goalVec.action = visualization_msgs::msg::Marker::ADD;

    geometry_msgs::msg::Point start, end;

    start.x = goalX;
    start.y = goalY;
    end.x = goalX +5;
    end.y = goalY +5;

    goalVec.points.push_back(start);
    goalVec.points.push_back(end);

    goalVec.scale.x = 0.05; // Arrow shaft diameter
    goalVec.scale.y = 0.1;  // Arrow head diameter
    goalVec.color.g = 1.0;  // Green color
    goalVec.color.a = .9;

    pubGoal->publish(goalVec);
}


void PathPlanner::planPath(const nav_msgs::msg::OccupancyGrid &grid) {
    // Retrieve grid parameters
    height = grid.info.height;
    width = grid.info.width;
    resolution = grid.info.resolution; // Resolution of the grid
    gridOriginX = grid.info.origin.position.x; // X coordinate of the grid origin
    gridOriginY = grid.info.origin.position.y; // Y coordinate of the grid origin

    // Validate grid data
    if (grid.data.empty()) {
        // RCLCPP_WARN(this->get_logger(), "Received an empty occupancy grid. Cannot plan path.");
        return;
    }
    // RCLCPP_INFO(this->get_logger(), "Received grid with dimensions (%d x %d)", width, height);

    // Convert robot and goal positions from world coordinates to grid indices
    int robotXIdx = static_cast<int>((robotX - gridOriginX) / resolution);
    int robotYIdx = static_cast<int>((robotY - gridOriginY) / resolution);
    int goalXIdx = static_cast<int>((goalX - gridOriginX) / resolution);
    int goalYIdx = static_cast<int>((goalY - gridOriginY) / resolution);

    // Ensure robot and goal positions are within bounds
    if (robotXIdx < 0 || robotXIdx >= width || robotYIdx < 0 || robotYIdx >= height) {
        // RCLCPP_WARN(this->get_logger(), "Robot position (%d, %d) is out of grid bounds.", robotXIdx, robotYIdx);
        return;
    }
    if (goalXIdx < 0 || goalXIdx >= width || goalYIdx < 0 || goalYIdx >= height) {
        // RCLCPP_WARN(this->get_logger(), "Goal position (%d, %d) is out of grid bounds.", goalXIdx, goalYIdx);
        return;
    }


    costmap = costMap(grid);
    // Convert grid indices to linear indices
    int robotGridPos = robotYIdx * width + robotXIdx;
    int goalGridPos = goalYIdx * width + goalXIdx;
    // Call the A* algorithm to plan the path
    nav_msgs::msg::Path path = aStar(grid, costmap, robotGridPos, goalGridPos);
    publishGoalVector();

    // If no valid path was found, log a warning
    if (path.poses.empty()) {
        RCLCPP_WARN(this->get_logger(), "Failed to find a path from (%d, %d) to (%d, %d).", robotXIdx, robotYIdx, goalXIdx, goalYIdx);
    }

    // RCLCPP_INFO(this->get_logger(), "cost weight: %f", get_parameter("cost_weight").as_double());
    // RCLCPP_INFO(this->get_logger(), "heuristic weight: %f", get_parameter("heuristic_weight").as_double());

    if (path.poses.size() > 3){
        pubPath->publish(path);
    }
    pubCostMap->publish(costmap);

}

nav_msgs::msg::Path PathPlanner::aStar(const nav_msgs::msg::OccupancyGrid &grid, const nav_msgs::msg::OccupancyGrid &costmap, const int startPos, const int goal) {
    height = grid.info.height;
    width = grid.info.width;
    resolution = grid.info.resolution;
    gridOriginX = grid.info.origin.position.x;
    gridOriginY = grid.info.origin.position.y;

    auto getIndex = [&](int x, int y) { return y * width + x; };
    auto isValid = [&](int x, int y) {
        int buffer = 2; // Number of cells to inflate obstacles
        for (int dx = -buffer; dx <= buffer; ++dx) {
            for (int dy = -buffer; dy <= buffer; ++dy) {
                int nx = x + dx, ny = y + dy;
                if (nx >= 0 && nx < width && ny >= 0 && ny < height) {
                    if (grid.data[getIndex(nx, ny)] >= 10) return false;
                }
            }
        }
        return true;
    };

    // auto heuristic = [&](int x1, int y1, int x2, int y2) {
    //     return std::abs(x1 - x2) + std::abs(y1 - y2); // Manhattan distance
    // };


    // auto heuristic = [&](int x1, int y1, int x2, int y2) {
    //     float dx = std::abs(x1 - x2);
    //     float dy = std::abs(y1 - y2);
    //     return (dx + dy) + (sqrt(2) - 2) * fmin(dx, dy); // Diagonal/octile distance
    // };


    auto heuristic = [&](int x1, int y1, int x2, int y2) {
        float dx = std::abs(x1 - x2);
        float dy = std::abs(y1 - y2);
        return std::sqrt(std::pow(dx, 2) + std::pow(dy, 2)); // Euclidean distance
    };

    // auto heuristic = [&](int x1, int y1, int x2, int y2) {
    //     return 0.0;
    // };

    int startX = startPos % width, startY = startPos / width;
    int goalX = goal % width, goalY = goal / width;

    std::priority_queue<std::tuple<float, int, int>, std::vector<std::tuple<float, int, int>>, std::greater<>> openSet;
    std::vector<float> gCost(height * width, std::numeric_limits<float>::infinity());
    std::unordered_map<int, int> cameFrom;
    std::vector<bool> closedSet(height * width, false);

    gCost[getIndex(startX, startY)] = 0;
    openSet.emplace(heuristic(startX, startY, goalX, goalY), startX, startY);

    while (!openSet.empty()) {
        auto [currentFCost, currentX, currentY] = openSet.top();
        openSet.pop();

        if (currentX == goalX && currentY == goalY) {
            nav_msgs::msg::Path path;
            path.header.frame_id = grid.header.frame_id;
            path.header.stamp = this->get_clock()->now();

            for (int idx = getIndex(goalX, goalY); idx != getIndex(startX, startY); idx = cameFrom[idx]) {
                double x = idx % width + 0.5, y = idx / width + 0.5;
                geometry_msgs::msg::PoseStamped pose;
                pose.pose.position.x = x * resolution + gridOriginX;
                pose.pose.position.y = y * resolution + gridOriginY;
                pose.pose.orientation.w = 1.0;
                path.poses.push_back(pose);
            }

            geometry_msgs::msg::PoseStamped startPose;
            startPose.pose.position.x = startX * resolution + gridOriginX;
            startPose.pose.position.y = startY * resolution + gridOriginY;
            startPose.pose.orientation.w = 1.0;
            path.poses.push_back(startPose);

            std::reverse(path.poses.begin(), path.poses.end());
            return path;
        }

        closedSet[getIndex(currentX, currentY)] = true;

        std::vector<std::pair<int, int>> neighbors = {
                                                      {currentX - 1, currentY}, {currentX + 1, currentY}, {currentX, currentY - 1}, {currentX, currentY + 1},
                                                      {currentX - 1, currentY - 1}, {currentX - 1, currentY + 1}, {currentX + 1, currentY - 1}, {currentX + 1, currentY + 1}};

        for (auto [nx, ny] : neighbors) {
            if (!isValid(nx, ny)) continue;

            float movementCost = (nx != currentX && ny != currentY) ? std::sqrt(2) : 1;
            // Fetch the costmap value and add its weighted influence
            float costmapValue = costmap.data[getIndex(nx, ny)];
            if (costmapValue == 0 || costmapValue == 100.0) continue; // Skip obstacles

            costWeight = get_parameter("cost_weight").as_double();
            heuristicWeight = get_parameter("heuristic_weight").as_double();

            float tentativeG = gCost[getIndex(currentX, currentY)] + movementCost + (costmapValue * costWeight);
            float fCost = tentativeG + heuristic(nx, ny, goalX, goalY) * heuristicWeight;

            if (tentativeG < gCost[getIndex(nx, ny)]) {
                cameFrom[getIndex(nx, ny)] = getIndex(currentX, currentY);
                gCost[getIndex(nx, ny)] = tentativeG;
                openSet.emplace(fCost, nx, ny);
            }
        }
    }

    // RCLCPP_WARN(this->get_logger(), "Path not found");
    return nav_msgs::msg::Path();
}







nav_msgs::msg::OccupancyGrid PathPlanner::costMap(const nav_msgs::msg::OccupancyGrid &grid) {
    nav_msgs::msg::OccupancyGrid costmap;
    costmap.header.frame_id = "odom";
    costmap.header.stamp = this->get_clock()->now();

    // Copy metadata
    costmap.info = grid.info;
    costmap.data.resize(grid.data.size(), 0);

    int bufferDanger = 4; // Red Zone
    int bufferWarning = 30; // Gradient zone
    float maxCost = 100.0f; // Normalize to 100

    for (int y = 0; y < costmap.info.height; ++y) {
        for (int x = 0; x < costmap.info.width; ++x) {
            int index = y * costmap.info.width + x;
            if (grid.data[index] > 10) {
                costmap.data[index] = -1; // Obstacles explicitly marked
                continue;
            }

            float cost = 0.0f;
            for (int dy = -bufferWarning; dy <= bufferWarning; ++dy) {
                for (int dx = -bufferWarning; dx <= bufferWarning; ++dx) {
                    int nx = x + dx;
                    int ny = y + dy;
                    if (nx >= 0 && nx < costmap.info.width && ny >= 0 && ny < costmap.info.height) {
                        int neighborIndex = ny * costmap.info.width + nx;
                        if (grid.data[neighborIndex] > 50) {
                            float distance = std::sqrt(dx * dx + dy * dy);
                            if (distance <= bufferDanger)
                                cost = maxCost; // Red zone
                            else if (distance <= bufferWarning)
                                cost = fmax(cost, maxCost * std::exp(-(distance - bufferDanger) * .5));
                        }
                    }
                }
            }

            // Assign cost values for specific colors
            if (cost == maxCost) {
                costmap.data[index] = 100;  // Full Red (Danger)
            } else {
                costmap.data[index] = (int8_t) fmax(fmin(cost, 98.0), 5.0);
            }

        }
    }
    return costmap;
}



void PathPlanner::publishCostmapVisualization(const nav_msgs::msg::OccupancyGrid &costmap) {
    if (!pubMarkers) {
        // RCLCPP_ERROR(this->get_logger(), "Publisher not initialized!");
        return;
    }

    if (costmap.data.empty()) {
        // RCLCPP_ERROR(this->get_logger(), "Costmap data is empty!");
        return;
    }

    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = costmap.header.frame_id;
    marker.header.stamp = this->get_clock()->now();
    marker.ns = "costmap";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::POINTS;
    marker.action = visualization_msgs::msg::Marker::ADD;

    // Set the scale of the points
    marker.scale.x = costmap.info.resolution; // Width of each point
    marker.scale.y = costmap.info.resolution; // Height of each point
    marker.scale.z = 0.01; // Thin layer (optional)

    // Iterate through the costmap
    for (int y = 0; y < costmap.info.height; ++y) {
        for (int x = 0; x < costmap.info.width; ++x) {
            int index = y * costmap.info.width + x;

            // Skip invalid or uninitialized indices
            if (index < 0 || index >= costmap.data.size()) {
                continue;
            }

            geometry_msgs::msg::Point point;
            point.x = x * costmap.info.resolution + costmap.info.origin.position.x;
            point.y = y * costmap.info.resolution + costmap.info.origin.position.y;
            point.z = 0.0;

            std_msgs::msg::ColorRGBA color;

            // Assign colors based on costmap values
            if (costmap.data[index] == -1) { // Obstacle
                continue;
            } else if (costmap.data[index] >= 50) { // Danger zone
                color.r = 1.0;
                color.g = 0.0;
                color.b = 0.0;
                color.a = .30;
            } else if (costmap.data[index] >= 25) { // Warning zone
                color.r = 1.0;
                color.g = 1.0;
                color.b = 0.0;
                color.a = .30;
            } else { // Safe zone
                continue;
            }

            // Add the point and its color
            marker.points.push_back(point);
            marker.colors.push_back(color);
        }
    }

    pubMarkers->publish(marker);
}























// #include "../include/path_planning/path_planning.h"
// #include <tf2/utils.h>
// #include <tf2/convert.h>
// #include <tf2/LinearMath/Transform.h>
// #include <tf2/LinearMath/Vector3.h>
// #include <tf2/LinearMath/Quaternion.h>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
// #include <builtin_interfaces/msg/time.hpp>
// #include <geometry_msgs/msg/point.hpp>
// #include <std_msgs/msg/color_rgba.hpp>
// #include <color_names/color_names.hpp>
// #include <algorithm>
// #include <cmath>
// #include <math.h>

// using std::placeholders::_1;

// PathPlanner::PathPlanner(): Node("path_planner")
// {
//     // BIND planPath to goal subscriber not grid!!! -> change to static
//     subMap = this->create_subscription<nav_msgs::msg::OccupancyGrid>("/Grid",
//                                                                      10,
//                                                                      std::bind(&PathPlanner::gridCallback, this, _1));
//     subGoal = this->create_subscription<geometry_msgs::msg::PoseStamped>("/goal_pose",
//                                                                          1,
//                                                                          std::bind(&PathPlanner::planPath, this, _1));
//     subRobotPos = this->create_subscription<nav_msgs::msg::Odometry>("/odom",
//                                                                      10,
//                                                                      std::bind(&PathPlanner::robotPoseCallback, this, _1));

//     pubPath = create_publisher<nav_msgs::msg::Path>("/Path", 10);
//     pubGoal= create_publisher<visualization_msgs::msg::Marker>("goalVec", 10);
//     pubCostMap = create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);


//     declare_parameter("cost_weight", 1.0);
//     declare_parameter("heuristic_weight", 1.0);
// }

// void PathPlanner::gridCallback(const nav_msgs::msg::OccupancyGrid grid)
// {
//     gridMap = grid;
//     costmap = costMap(grid);
//     pubCostMap->publish(costmap);
//     return;
// }

// void PathPlanner::robotPoseCallback(const nav_msgs::msg::Odometry::SharedPtr odom)
// {
//     robotX = odom->pose.pose.position.x;
//     robotY = odom->pose.pose.position.y;
//     return;
// }


// void PathPlanner::publishGoalVector() {
//     visualization_msgs::msg::Marker goalVec;
//     goalVec.header.frame_id = "base_link";
//     goalVec.header.stamp = get_clock()->now();
//     goalVec.ns = "goal_vector";
//     goalVec.id = 0;
//     goalVec.type = visualization_msgs::msg::Marker::ARROW;
//     goalVec.action = visualization_msgs::msg::Marker::ADD;

//     geometry_msgs::msg::Point start, end;

//     start.x = goalX;
//     start.y = goalY;
//     end.x = goalX +5;
//     end.y = goalY +5;

//     goalVec.points.push_back(start);
//     goalVec.points.push_back(end);

//     goalVec.scale.x = 0.05; // Arrow shaft diameter
//     goalVec.scale.y = 0.1;  // Arrow head diameter
//     goalVec.color.g = 1.0;  // Green color
//     goalVec.color.a = .9;

//     pubGoal->publish(goalVec);
// }


// void PathPlanner::planPath(const geometry_msgs::msg::PoseStamped &goal) {
//     RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Received goal pose: (%f, %f, %f)",
//                 goal.pose.position.x, goal.pose.position.y, goal.pose.position.z);
//     goalX = goal.pose.position.x;
//     goalY = goal.pose.position.y;


//     // Retrieve grid parameters
//     height = gridMap.info.height;
//     width = gridMap.info.width;
//     resolution = gridMap.info.resolution; // Resolution of the grid
//     gridOriginX = gridMap.info.origin.position.x; // X coordinate of the grid origin
//     gridOriginY = gridMap.info.origin.position.y; // Y coordinate of the grid origin

//     // Validate grid data
//     if (gridMap.data.empty()) {
//         RCLCPP_WARN(this->get_logger(), "Received an empty occupancy grid. Cannot plan path.");
//         return;
//     }
//     RCLCPP_INFO(this->get_logger(), "Received grid with dimensions (%d x %d)", width, height);

//     // Convert robot and goal positions from world coordinates to grid indices
//     int robotXIdx = static_cast<int>((robotX - gridOriginX) / resolution);
//     int robotYIdx = static_cast<int>((robotY - gridOriginY) / resolution);
//     int goalXIdx = static_cast<int>((goalX - gridOriginX) / resolution);
//     int goalYIdx = static_cast<int>((goalY - gridOriginY) / resolution);

//     // Ensure robot and goal positions are within bounds
//     if (robotXIdx < 0 || robotXIdx >= width || robotYIdx < 0 || robotYIdx >= height) {
//         RCLCPP_WARN(this->get_logger(), "Robot position (%d, %d) is out of grid bounds.", robotXIdx, robotYIdx);
//         return;
//     }
//     if (goalXIdx < 0 || goalXIdx >= width || goalYIdx < 0 || goalYIdx >= height) {
//         RCLCPP_WARN(this->get_logger(), "Goal position (%d, %d) is out of grid bounds.", goalXIdx, goalYIdx);
//         return;
//     }
//     // Convert grid indices to linear indices
//     int robotGridPos = robotYIdx * width + robotXIdx;
//     int goalGridPos = goalYIdx * width + goalXIdx;
//     // Call the A* algorithm to plan the path
//     nav_msgs::msg::Path path = aStar(gridMap, costmap, robotGridPos, goalGridPos);
//     publishGoalVector();

//     // If no valid path was found, log a warning
//     if (path.poses.empty()) {
//         RCLCPP_WARN(this->get_logger(), "Failed to find a path from (%d, %d) to (%d, %d).", robotXIdx, robotYIdx, goalXIdx, goalYIdx);
//     }

//     RCLCPP_INFO(this->get_logger(), "cost weight: %f", get_parameter("cost_weight").as_double());
//     RCLCPP_INFO(this->get_logger(), "heuristic weight: %f", get_parameter("heuristic_weight").as_double());

//     pubPath->publish(path);
// }

// nav_msgs::msg::Path PathPlanner::aStar(const nav_msgs::msg::OccupancyGrid &grid, const nav_msgs::msg::OccupancyGrid &costmap, const int startPos, const int goal) {
//     height = grid.info.height;
//     width = grid.info.width;
//     resolution = grid.info.resolution;
//     gridOriginX = grid.info.origin.position.x;
//     gridOriginY = grid.info.origin.position.y;

//     auto getIndex = [&](int x, int y) { return y * width + x; };
//     auto isValid = [&](int x, int y) {
//         int buffer = 2; // Number of cells to inflate obstacles
//         for (int dx = -buffer; dx <= buffer; ++dx) {
//             for (int dy = -buffer; dy <= buffer; ++dy) {
//                 int nx = x + dx, ny = y + dy;
//                 if (nx >= 0 && nx < width && ny >= 0 && ny < height) {
//                     if (grid.data[getIndex(nx, ny)] >= 10) return false;
//                 }
//             }
//         }
//         return true;
//     };

//     // auto heuristic = [&](int x1, int y1, int x2, int y2) {
//     //     return std::abs(x1 - x2) + std::abs(y1 - y2); // Manhattan distance
//     // };


//     // auto heuristic = [&](int x1, int y1, int x2, int y2) {
//     //     float dx = std::abs(x1 - x2);
//     //     float dy = std::abs(y1 - y2);
//     //     return (dx + dy) + (sqrt(2) - 2) * fmin(dx, dy); // Diagonal/octile distance
//     // };


//     auto heuristic = [&](int x1, int y1, int x2, int y2) {
//         float dx = std::abs(x1 - x2);
//         float dy = std::abs(y1 - y2);
//         return std::sqrt(std::pow(dx, 2) + std::pow(dy, 2)); // Euclidean distance
//     };

//     // auto heuristic = [&](int x1, int y1, int x2, int y2) {
//     //     return 0.0;
//     // };

//     int startX = startPos % width, startY = startPos / width;
//     int goalX = goal % width, goalY = goal / width;

//     std::priority_queue<std::tuple<float, int, int>, std::vector<std::tuple<float, int, int>>, std::greater<>> openSet;
//     std::vector<float> gCost(height * width, std::numeric_limits<float>::infinity());
//     std::unordered_map<int, int> cameFrom;
//     std::vector<bool> closedSet(height * width, false);

//     gCost[getIndex(startX, startY)] = 0;
//     openSet.emplace(heuristic(startX, startY, goalX, goalY), startX, startY);

//     while (!openSet.empty()) {
//         auto [currentFCost, currentX, currentY] = openSet.top();
//         openSet.pop();

//         if (currentX == goalX && currentY == goalY) {
//             nav_msgs::msg::Path path;
//             path.header.frame_id = grid.header.frame_id;
//             path.header.stamp = this->get_clock()->now();

//             for (int idx = getIndex(goalX, goalY); idx != getIndex(startX, startY); idx = cameFrom[idx]) {
//                 double x = idx % width + 0.5, y = idx / width + 0.5;
//                 geometry_msgs::msg::PoseStamped pose;
//                 pose.pose.position.x = x * resolution + gridOriginX;
//                 pose.pose.position.y = y * resolution + gridOriginY;
//                 pose.pose.orientation.w = 1.0;
//                 path.poses.push_back(pose);
//             }

//             geometry_msgs::msg::PoseStamped startPose;
//             startPose.pose.position.x = startX * resolution + gridOriginX;
//             startPose.pose.position.y = startY * resolution + gridOriginY;
//             startPose.pose.orientation.w = 1.0;
//             path.poses.push_back(startPose);

//             std::reverse(path.poses.begin(), path.poses.end());
//             return path;
//         }

//         closedSet[getIndex(currentX, currentY)] = true;

//         std::vector<std::pair<int, int>> neighbors = {
//                                                       {currentX - 1, currentY}, {currentX + 1, currentY}, {currentX, currentY - 1}, {currentX, currentY + 1},
//                                                       {currentX - 1, currentY - 1}, {currentX - 1, currentY + 1}, {currentX + 1, currentY - 1}, {currentX + 1, currentY + 1}};

//         for (auto [nx, ny] : neighbors) {
//             if (!isValid(nx, ny)) continue;

//             float movementCost = (nx != currentX && ny != currentY) ? std::sqrt(2) : 1;
//             // Fetch the costmap value and add its weighted influence
//             float costmapValue = costmap.data[getIndex(nx, ny)];
//             if (costmapValue == 0 || costmapValue == 100) continue; // Skip obstacles

//             costWeight = get_parameter("cost_weight").as_double();
//             heuristicWeight = get_parameter("heuristic_weight").as_double();
//             if (costmapValue == 98) {
//                 costmapValue *= 1000;
//             }
//             float tentativeG = gCost[getIndex(currentX, currentY)] + movementCost + (costmapValue * costWeight);
//             float fCost = tentativeG + heuristic(nx, ny, goalX, goalY) * heuristicWeight;

//             if (tentativeG < gCost[getIndex(nx, ny)]) {
//                 cameFrom[getIndex(nx, ny)] = getIndex(currentX, currentY);
//                 gCost[getIndex(nx, ny)] = tentativeG;
//                 openSet.emplace(fCost, nx, ny);
//             }
//         }
//     }

//     RCLCPP_WARN(this->get_logger(), "Path not found");
//     return nav_msgs::msg::Path();
// }



// nav_msgs::msg::OccupancyGrid PathPlanner::costMap(const nav_msgs::msg::OccupancyGrid &grid) {
//     nav_msgs::msg::OccupancyGrid costmap;
//     costmap.header.frame_id = "odom";
//     costmap.header.stamp = this->get_clock()->now();

//     // Copy metadata
//     costmap.info = grid.info;
//     costmap.data.resize(grid.data.size(), 0);

//     int bufferDoNotEnter = 2;
//     int bufferDanger = 4; // Red Zone
//     int bufferWarning = 30; // Gradient zone
//     float maxCost = 98.0f; // Normalize to 100


//     for (int y = 0; y < costmap.info.height; ++y) {
//         for (int x = 0; x < costmap.info.width; ++x) {
//             int index = y * costmap.info.width + x;
//             if (grid.data[index] > 20) {
//                 costmap.data[index] = -1; // Hindernisse explizit markieren
//                 continue;
//             }

//             float cost = 0.0f;

//             // Zuerst: bufferDoNotEnter prüfen
//             for (int dy = -bufferDoNotEnter; dy <= bufferDoNotEnter; ++dy) {
//                 for (int dx = -bufferDoNotEnter; dx <= bufferDoNotEnter; ++dx) {
//                     int nx = x + dx;
//                     int ny = y + dy;
//                     if (nx >= 0 && nx < costmap.info.width && ny >= 0 && ny < costmap.info.height) {
//                         int neighborIndex = ny * costmap.info.width + nx;
//                         if (grid.data[neighborIndex] > 50) {
//                             costmap.data[index] = 100; // Verbotene Zone
//                             goto next_cell; // Überspringe weitere Berechnungen
//                         }
//                     }
//                 }
//             }

//             // Dann: Buffer für Danger- und Warning-Zonen
//             for (int dy = -bufferWarning; dy <= bufferWarning; ++dy) {
//                 for (int dx = -bufferWarning; dx <= bufferWarning; ++dx) {
//                     int nx = x + dx;
//                     int ny = y + dy;
//                     if (nx >= 0 && nx < costmap.info.width && ny >= 0 && ny < costmap.info.height) {
//                         int neighborIndex = ny * costmap.info.width + nx;
//                         if (grid.data[neighborIndex] > 50) {
//                             float distance = std::sqrt(dx * dx + dy * dy);
//                             if (distance <= bufferDanger)
//                                 cost = maxCost; // Rote Zone
//                             else if (distance <= bufferWarning)
//                                 cost = fmax(cost, maxCost * std::exp(-(distance - bufferDanger) * .05));
//                         }
//                     }
//                 }
//             }

//             // Kosten zuweisen
//             if (cost == maxCost) {
//                 costmap.data[index] = 98;  // Volle Gefahr (Rot)
//             } else {
//                 costmap.data[index] = (int8_t) fmax(fmin(cost, 97.0), 5.0);
//             }

//         next_cell:;
//         }
//     }
//     return costmap;
// }



// void PathPlanner::publishCostmapVisualization(const nav_msgs::msg::OccupancyGrid &costmap) {
//     if (!pubMarkers) {
//         RCLCPP_ERROR(this->get_logger(), "Publisher not initialized!");
//         return;
//     }

//     if (costmap.data.empty()) {
//         RCLCPP_ERROR(this->get_logger(), "Costmap data is empty!");
//         return;
//     }

//     visualization_msgs::msg::Marker marker;
//     marker.header.frame_id = costmap.header.frame_id;
//     marker.header.stamp = this->get_clock()->now();
//     marker.ns = "costmap";
//     marker.id = 0;
//     marker.type = visualization_msgs::msg::Marker::POINTS;
//     marker.action = visualization_msgs::msg::Marker::ADD;

//     // Set the scale of the points
//     marker.scale.x = costmap.info.resolution; // Width of each point
//     marker.scale.y = costmap.info.resolution; // Height of each point
//     marker.scale.z = 0.01; // Thin layer (optional)

//     // Iterate through the costmap
//     for (int y = 0; y < costmap.info.height; ++y) {
//         for (int x = 0; x < costmap.info.width; ++x) {
//             int index = y * costmap.info.width + x;

//             // Skip invalid or uninitialized indices
//             if (index < 0 || index >= costmap.data.size()) {
//                 continue;
//             }

//             geometry_msgs::msg::Point point;
//             point.x = x * costmap.info.resolution + costmap.info.origin.position.x;
//             point.y = y * costmap.info.resolution + costmap.info.origin.position.y;
//             point.z = 0.0;

//             std_msgs::msg::ColorRGBA color;

//             // Assign colors based on costmap values
//             if (costmap.data[index] == -1) { // Obstacle
//                 continue;
//             } else if (costmap.data[index] >= 50) { // Danger zone
//                 color.r = 1.0;
//                 color.g = 0.0;
//                 color.b = 0.0;
//                 color.a = .30;
//             } else if (costmap.data[index] >= 25) { // Warning zone
//                 color.r = 1.0;
//                 color.g = 1.0;
//                 color.b = 0.0;
//                 color.a = .30;
//             } else { // Safe zone
//                 continue;
//             }

//             // Add the point and its color
//             marker.points.push_back(point);
//             marker.colors.push_back(color);
//         }
//     }

//     pubMarkers->publish(marker);
// }









