#ifndef PATH_FOLLOWING_H
#define PATH_FOLLOWING_H

#include <rclcpp/rclcpp.hpp>
#include "vec2.h"
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include "pid_controller.h"
#include <message_filters/subscriber.h>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <kobuki_ros_interfaces/msg/led.hpp>
#include <kobuki_ros_interfaces/msg/sound.hpp>
#include <kobuki_ros_interfaces/msg/bumper_event.hpp>

class PathFollowing: public rclcpp::Node {
    public:
        PathFollowing();

    private:

        Vec2f robotPos;
        std::optional<Vec2f> prevRobotPos;
        PIDController pid;

        rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr subPath;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubSmoothPath;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubRobotPath;

        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subOdom;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subObstAvoid;
        rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr subCostmap;

        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pubCmdVel;
        rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pubProj;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pubPath1;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pubPath2;
        message_filters::Subscriber<sensor_msgs::msg::LaserScan> subLaser;

        rclcpp::Publisher<kobuki_ros_interfaces::msg::Led>::SharedPtr pubLed1;
        rclcpp::Publisher<kobuki_ros_interfaces::msg::Led>::SharedPtr pubLed2;
        rclcpp::Publisher<kobuki_ros_interfaces::msg::Sound>::SharedPtr pubSound;
        rclcpp::Subscription<kobuki_ros_interfaces::msg::BumperEvent>::SharedPtr subBumper;

        bool bumperPressed[3] = {false, false, false};
        bool anyBumperPressed = false;
        bool backwards = false;

        std::vector<Vec2f> laserPoints;

        sensor_msgs::msg::LaserScan lastScan;
        bool obstacleDetected = false;
        bool isPathBlocked = false;

        geometry_msgs::msg::Twist avoidMessage;

        double maxSpeed;
        bool playedStartSound = false;
        bool playedStartSound2 = false;
        double startX = 0.0;  // Startposition für das Zurückfahren
        bool isReversing = false;  // Flag für Rückwärtsfahren
        int reverseCounter = 0;  // Zählt die aufeinanderfolgenden Rückwärtsbewegungen
        const int maxReverseAttempts = 2;  // Maximale Anzahl an Rückwärtsbewegungen



        void smoothPath(const nav_msgs::msg::Path &path);
        void setLed();
        void pathBlocked();
        void costmapCallBack(const nav_msgs::msg::OccupancyGrid &costmap);
        void laserCallback(const sensor_msgs::msg::LaserScan &scan);
        void bumperCallback(const kobuki_ros_interfaces::msg::BumperEvent &bumperMsg);
        void playSounds();
        void playSounds2();
        void goingBackwards();

        std::shared_ptr<tf2_ros::Buffer> tf2Buffer;
        std::shared_ptr<tf2_ros::TransformListener> tf2Listener;
        std::shared_ptr<tf2_ros::MessageFilter<sensor_msgs::msg::LaserScan>> tf2MessageFilter;


        std::vector<geometry_msgs::msg::PoseStamped> smoothedPath;

        nav_msgs::msg::Path followedPathTotal;

        nav_msgs::msg::Path followedPathFiltered;

        nav_msgs::msg::OccupancyGrid costmap;

        double findDesiredAngle(const geometry_msgs::msg::Pose &robotPose);

        int computeSign(const geometry_msgs::msg::Pose &robotPose,
                                       const geometry_msgs::msg::Pose &projectionPose,
                                       const geometry_msgs::msg::Pose &pathTangent);

        double computePhiC(double distance);

        void robotPoseCallback(const geometry_msgs::msg::Pose &robotPose);

        double euclDistance(const geometry_msgs::msg::Pose robotPose, const geometry_msgs::msg::PoseStamped pathPose);

        void odomCallback(const nav_msgs::msg::Odometry &odom);

        void constructFollowedPath();

        void sendControlCommand(double speed, double omega, double phiC);


};

#endif
