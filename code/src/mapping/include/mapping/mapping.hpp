#ifndef MAPPING_H
#define MAPPING_H

#include "gridtraversal.hpp"
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2/transform_datatypes.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <message_filters/subscriber.h>

#include <vector>
#include <memory>
#include <mutex>
#include "vec2.h"


/// The Mapping class provides functions for mapping with an Occupancy Grid.
class Mapping : public rclcpp::Node{
public:
  Mapping();


private:

    std::mutex mutex;
    bool odomInit;
    bool laserInit;
    bool obstacleDetected = false;
    Vec2f robotPos;
    double robotYaw;
    double resolution;
    double noiseCheckWidth;
    double noiseCheckLength;


  /// Callback function for incoming laser messages
    void extracted(Vec2f &laser, double &dangerZoneWidth,
                   double &dangerZoneLength, bool &obstacleDetected);
    void laserCallback(const sensor_msgs::msg::LaserScan &scan);
    /// Updates the Occupancy Grid Map
    void update(const Vec2f &robotPos);
    void decayMapCells();

    int mapWidth;
    int mapHeight;
    std::vector<Vec2f> laserPoints;
    std::vector<Vec2f> prevLaserPoints;
    std::vector<Vec2f> prevPrevLaserPoints;
    std::vector<bool> flagPoints;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pubCmdVel;

    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pubGrid;
    // rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subLaser;

    nav_msgs::msg::OccupancyGrid map;
  std::shared_ptr<tf2_ros::Buffer> tf2Buffer;
  std::shared_ptr<tf2_ros::TransformListener> tf2Listener;
  message_filters::Subscriber<sensor_msgs::msg::LaserScan> subLaser;
  std::shared_ptr<tf2_ros::MessageFilter<sensor_msgs::msg::LaserScan>> tf2MessageFilter;
};

#endif // MAPPING_H
