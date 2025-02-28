#ifndef OBSTACLE_AVOIDANCE_H
#define OBSTACLE_AVOIDANCE_H

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/bool.hpp>

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
#include "potential_field.h"

class ObstacleAvoidance: public rclcpp::Node {
    public:
        ObstacleAvoidance();

    private:
        void odomCallback(const nav_msgs::msg::Odometry &odom);
        void laserCallback(const sensor_msgs::msg::LaserScan &scan);
        void goalCallback(const geometry_msgs::msg::PoseStamped goalMsg);
        void process();
        void visualizeMarkers();
        void publishGoalVector(Vec2f force);
        void publishTotalForce(Vec2f force);
        Vec2f publishObjectVector(const std::vector<Vec2f>& laserPoints);

        std::mutex mutex;
        bool odomInit;
        bool laserInit;

        Vec2f robotPos;
        double robotYaw;
        Vec2f goalPos;
        double dangerZoneWidth;
        double dangerZoneLength;
        bool obstacleDetected;
        double maxSpeed;
        std::vector<Vec2f> laserPoints;
        PotentialField potentialField;

        sensor_msgs::msg::LaserScan lastScan;

        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pubCmdVel;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pubGoal;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pubDangerZone;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pubTotalForce;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pubMarkerArray;



        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subOdom;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subGoal;

        std::shared_ptr<tf2_ros::Buffer> tf2Buffer;
        std::shared_ptr<tf2_ros::TransformListener> tf2Listener;
        message_filters::Subscriber<sensor_msgs::msg::LaserScan> subLaser;
        std::shared_ptr<tf2_ros::MessageFilter<sensor_msgs::msg::LaserScan>> tf2MessageFilter;
};

#endif
