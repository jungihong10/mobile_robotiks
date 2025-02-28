#ifndef PathPlanner_H
#define PathPlanner_H

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>


#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>
#include <message_filters/subscriber.h>

#include <memory>
#include <mutex>
#include <optional>
#include <vector>


class PathPlanner : public rclcpp::Node {
public:
    PathPlanner();


    struct Point {
        int x, y;
        float gCost, hCost, fCost;
        Point* parent;

        Point(int x, int y, float gCost = 0, float hCost = 0, Point* parent = nullptr)
            : x(x), y(y), gCost(gCost), hCost(hCost), parent(parent) {
            fCost = gCost + hCost;
        }

        bool operator>(const Point& other) const {
            if (fCost == other.fCost) {
                return hCost > other.hCost;
            }
            return fCost > other.fCost;
        }

        bool operator==(const Point& other) const {
            return x == other.x && y == other.y;
        }
        struct Hash {
            size_t operator()(const Point& node) const {
                return std::hash<int>()(node.x) ^ std::hash<int>()(node.y);
            }
        };
    };

private:
    double robotX;
    double robotY;
    double goalX;
    double goalY;

    int height;
    int width;
    float resolution;
    float gridOriginX;
    float gridOriginY;
    float totalPathCost;
    double costWeight;
    double heuristicWeight;
    nav_msgs::msg::Path path;


    void planPath(const nav_msgs::msg::OccupancyGrid &grid);
    void publishGoalVector();
    void goalPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr goal);
    void robotPoseCallback(const nav_msgs::msg::Odometry::SharedPtr odom);
    nav_msgs::msg::OccupancyGrid costMap(const nav_msgs::msg::OccupancyGrid &grid);
    void publishCostmapVisualization(const nav_msgs::msg::OccupancyGrid &costmap);


    nav_msgs::msg::Path aStar(const nav_msgs::msg::OccupancyGrid &grid,const nav_msgs::msg::OccupancyGrid &costmap,
                              const int startPos,
                              const int goal);

    std::mutex mutex;

    nav_msgs::msg::OccupancyGrid costmap;

    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pubGoal;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pubCostMap;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pubMarkers;

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubPath;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr subMap;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subGoal;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subRobotPos;
    std::priority_queue<Point, std::vector<Point>, std::greater<Point>> openList;



};

#endif


// #ifndef PathPlanner_H
// #define PathPlanner_H

// #include <rclcpp/rclcpp.hpp>
// #include <sensor_msgs/msg/laser_scan.hpp>
// #include <sensor_msgs/msg/joy.hpp>
// #include <visualization_msgs/msg/marker.hpp>
// #include <visualization_msgs/msg/marker_array.hpp>
// #include <nav_msgs/msg/occupancy_grid.hpp>
// #include <nav_msgs/msg/path.hpp>
// #include <nav_msgs/msg/odometry.hpp>


// #include <tf2_ros/buffer.h>
// #include <tf2_ros/create_timer_ros.h>
// #include <tf2_ros/message_filter.h>
// #include <tf2_ros/transform_listener.h>
// #include <message_filters/subscriber.h>

// #include <memory>
// #include <mutex>
// #include <optional>
// #include <vector>


// class PathPlanner : public rclcpp::Node {
//     public:
//         PathPlanner();


//         struct Point {
//             int x, y;
//             float gCost, hCost, fCost;
//             Point* parent;

//             Point(int x, int y, float gCost = 0, float hCost = 0, Point* parent = nullptr)
//                 : x(x), y(y), gCost(gCost), hCost(hCost), parent(parent) {
//                 fCost = gCost + hCost;
//             }

//             bool operator>(const Point& other) const {
//                 if (fCost == other.fCost) {
//                     return hCost > other.hCost;
//                 }
//                 return fCost > other.fCost;
//             }

//             bool operator==(const Point& other) const {
//                 return x == other.x && y == other.y;
//             }
//             struct Hash {
//                 size_t operator()(const Point& node) const {
//                     return std::hash<int>()(node.x) ^ std::hash<int>()(node.y);
//                 }
//             };
//         };

//     private:
//         double robotX;
//         double robotY;
//         double goalX;
//         double goalY;

//         int height;
//         int width;
//         float resolution;
//         float gridOriginX;
//         float gridOriginY;
//         float totalPathCost;
//         double costWeight;
//         double heuristicWeight;
//         nav_msgs::msg::Path path;
//         nav_msgs::msg::OccupancyGrid gridMap;
//         nav_msgs::msg::OccupancyGrid costmap;



//         void planPath(const geometry_msgs::msg::PoseStamped &goal);
//         void publishGoalVector();
//         void gridCallback(const nav_msgs::msg::OccupancyGrid grid);
//         void robotPoseCallback(const nav_msgs::msg::Odometry::SharedPtr odom);
//         nav_msgs::msg::OccupancyGrid costMap(const nav_msgs::msg::OccupancyGrid &grid);
//         void publishCostmapVisualization(const nav_msgs::msg::OccupancyGrid &costmap);


//         nav_msgs::msg::Path aStar(const nav_msgs::msg::OccupancyGrid &grid,const nav_msgs::msg::OccupancyGrid &costmap,
//                                   const int startPos,
//                                   const int goal);

//         std::mutex mutex;


//         rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pubGoal;
//         rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pubCostMap;
//         rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pubMarkers;

//         rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubPath;
//         rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr subMap;
//         rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subGoal;
//         rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subRobotPos;
//         std::priority_queue<Point, std::vector<Point>, std::greater<Point>> openList;



// };

// #endif
