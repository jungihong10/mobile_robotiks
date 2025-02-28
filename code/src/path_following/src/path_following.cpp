#include "path_following.h"
#include "path_processing.h"
#include "write_plot_data.hpp"
#include <tf2/utils.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>
#include <chrono>
#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <message_filters/subscriber.h>

#include <unistd.h>


using std::placeholders::_1;
using namespace std::chrono;

PathFollowing::PathFollowing(): Node("path_following"), pid(1.0, 0.0, 0.0) { // Kp, Ki, Kd
    declare_parameter("Kp", 1.0);
    declare_parameter("Ki", 0.0);
    declare_parameter("Kd", 0.0);

   // RCLCPP_INFO(this->get_logger(), "Kp %.2f, Ki %.2f, Kd %.2f", pid.getKp(), pid.getKi(), pid.getKd());

    subPath = this->create_subscription<nav_msgs::msg::Path>("/Path",
                                                             10,
                                                             std::bind(&PathFollowing::smoothPath, this, _1));

    pubSmoothPath = create_publisher<nav_msgs::msg::Path>("/smoothPath", 10);
    pubPath1 = create_publisher<geometry_msgs::msg::PoseStamped>("/pathPoint1", 10);
    pubPath2 = create_publisher<geometry_msgs::msg::PoseStamped>("/pathPoint2", 10);

    subOdom = create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10,
        std::bind(&PathFollowing::odomCallback, this, _1));

    pubCmdVel = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    subCostmap = create_subscription<nav_msgs::msg::OccupancyGrid>("/costmap", 10, std::bind(&PathFollowing::costmapCallBack, this, _1));

    pubRobotPath = create_publisher<nav_msgs::msg::Path>("/followedPath", 10);

    pubLed1 = create_publisher<kobuki_ros_interfaces::msg::Led>("/commands/led1", 10);
    pubLed2 = create_publisher<kobuki_ros_interfaces::msg::Led>("/commands/led2", 10);
    pubSound = create_publisher<kobuki_ros_interfaces::msg::Sound>("/commands/sound", 10);
    subBumper = create_subscription<kobuki_ros_interfaces::msg::BumperEvent>("/events/bumper", 10, std::bind(&PathFollowing::bumperCallback, this, _1));

    tf2Buffer = std::make_shared<tf2_ros::Buffer>(get_clock());
    std::shared_ptr<tf2_ros::CreateTimerROS> timer_interface =
        std::make_shared<tf2_ros::CreateTimerROS>(get_node_base_interface(),
                                                  get_node_timers_interface());
    tf2Buffer->setCreateTimerInterface(timer_interface);
    tf2Listener = std::make_shared<tf2_ros::TransformListener>(*tf2Buffer);

    subLaser.subscribe(this, "/scan");
    tf2MessageFilter =
        std::make_shared<tf2_ros::MessageFilter<sensor_msgs::msg::LaserScan>>(
            subLaser, *tf2Buffer, "odom", 3, get_node_logging_interface(),
            get_node_clock_interface(), 500ms);
    tf2MessageFilter->setTolerance(100ms);
    tf2MessageFilter->registerCallback(&PathFollowing::laserCallback, this);
}


void PathFollowing::setLed() { // kobuki_ros_interfaces::msg::Led::RED
    kobuki_ros_interfaces::msg::Led ledMessage;
    ledMessage.value = kobuki_ros_interfaces::msg::Led::RED;
    if (obstacleDetected) {
        ledMessage.value = kobuki_ros_interfaces::msg::Led::RED;
    } else {
        ledMessage.value = kobuki_ros_interfaces::msg::Led::BLACK;
    }

    pubLed1->publish(ledMessage);
}

void PathFollowing::playSounds(){
    kobuki_ros_interfaces::msg::Sound soundMessage;
    soundMessage.value = kobuki_ros_interfaces::msg::Sound::CLEANINGEND;
    if (!smoothedPath.empty() && !playedStartSound) {
        pubSound->publish(soundMessage);
        playedStartSound = true;
    }
}


void PathFollowing::costmapCallBack(const nav_msgs::msg::OccupancyGrid &costMap){
    std::lock_guard<std::mutex> guard(std::mutex);
    // RCLCPP_INFO(this->get_logger(), "receiving costmap of size %zu", costMap.data.size());
    costmap = costMap;
}


void PathFollowing::laserCallback(const sensor_msgs::msg::LaserScan &scan) {
    std::lock_guard<std::mutex> guard(std::mutex);
    double dangerZoneWidth = .4;
    double dangerZoneLength = .25;
    lastScan = scan;
    laserPoints.clear();

    bool detected = false;

    for (unsigned int i = 0; i < scan.ranges.size(); i++) {
        float angle = scan.angle_min + i * scan.angle_increment;

        if (std::isnan(scan.ranges[i]) || std::isinf(scan.ranges[i]) ||
            scan.ranges[i] < scan.range_min || scan.range_max < scan.ranges[i]) {
            continue;
        }

        Vec2f laser = scan.ranges[i] * Vec2f::fromAngle(angle);
        std_msgs::msg::Header header = scan.header;
        header.stamp = header.stamp + rclcpp::Duration::from_seconds(i * scan.time_increment);
        Vec2f pOdom = tf2Buffer->transform(laser.toGeometryMsgPointStamped(header), "odom");

        laserPoints.push_back(pOdom);

        if (std::abs(laser.y) < dangerZoneWidth / 2 && laser.x > 0.10 && laser.x < dangerZoneLength) {
            detected = true;
            if (!isReversing) {
                startX = robotPos.x;
                isReversing = true;
            }
        }
    }

    obstacleDetected = detected;
}


void PathFollowing::bumperCallback(const kobuki_ros_interfaces::msg::BumperEvent &bumperMsg) {
    std::lock_guard<std::mutex> guard(std::mutex);

    if (bumperMsg.bumper < 0 || bumperMsg.bumper >= std::size(bumperPressed)) {
        RCLCPP_ERROR(this->get_logger(), "Invalid bumper message");
        return;
    }

    bumperPressed[bumperMsg.bumper] = bumperMsg.state;
    anyBumperPressed = bumperPressed[0] || bumperPressed[1] || bumperPressed[2];

    if(anyBumperPressed){
        startX = robotPos.x;
        backwards = true;
        isReversing = true;
    }

    /*if (anyBumperPressed) {
        RCLCPP_INFO(this->get_logger(), "Bumper pressed:");
        if (bumperPressed[0]) {
            RCLCPP_INFO(this->get_logger(), "0");
        }
        if (bumperPressed[1]) {
            RCLCPP_INFO(this->get_logger(), "1");
        }
        if (bumperPressed[2]) {
            RCLCPP_INFO(this->get_logger(), "2");
        }
    }*/
}


void PathFollowing::pathBlocked() {

    if (!costmap.data.empty()) {

        int costThreshold = 98;
        double resolution = costmap.info.resolution; // Resolution of the grid
        int width = costmap.info.width;
        int gridOriginX = costmap.info.origin.position.x; // X coordinate of the grid origin
        int gridOriginY = costmap.info.origin.position.y; // Y coordinate of the grid origin
        // RCLCPP_INFO(this->get_logger(), "1");

        int robotXIndex = static_cast<int>((robotPos.x - gridOriginX) / resolution);
        int robotYIndex = static_cast<int>((robotPos.y - gridOriginY) / resolution);

        int robotGridPos = robotYIndex * width + robotXIndex;
        int robotCellCost = costmap.data[robotGridPos];

        int pathCellCost = 0;
        if (smoothedPath.size() > 51) {
            int pathXIndex = static_cast<int>((smoothedPath[50].pose.position.x - gridOriginX) / resolution);
            int pathYIndex = static_cast<int>((smoothedPath[50].pose.position.y - gridOriginY) / resolution);

            int pathGridPos = pathYIndex * width + pathXIndex;
            pathCellCost = costmap.data[pathGridPos];
        } else {
            int pathXIndex = static_cast<int>((smoothedPath.back().pose.position.x - gridOriginX) / resolution);
            int pathYIndex = static_cast<int>((smoothedPath.back().pose.position.y - gridOriginY) / resolution);

            int pathGridPos = pathYIndex * width + pathXIndex;
            pathCellCost = costmap.data[pathGridPos];
        }

        // RCLCPP_INFO(this->get_logger(), "2");

        // RCLCPP_INFO(this->get_logger(), "%d ::::: %zu", robotGridPos, costmap.data.size());

        // RCLCPP_INFO(this->get_logger(), "3");

        if (robotCellCost >= costThreshold || pathCellCost >= costThreshold) {
            isPathBlocked = true;
            // RCLCPP_INFO(this->get_logger(), "path is blocked, stopping");
        } else {
            // RCLCPP_INFO(this->get_logger(), "3.no");
            isPathBlocked = false;
        }
    }
}

void PathFollowing::odomCallback(const nav_msgs::msg::Odometry &odom){
    std::lock_guard<std::mutex> guard(std::mutex);
    robotPos = odom.pose.pose.position;

    // RCLCPP_INFO(this->get_logger(), "1");

    // Pass it to the existing path-following logic
    robotPoseCallback(odom.pose.pose);
    // RCLCPP_INFO(this->get_logger(), "2");
    // Keep to max 10 poses in followed path, only if robot is moving
    if (prevRobotPos && (robotPos - *prevRobotPos).norm() > 0.0001) {
        // RCLCPP_INFO(this->get_logger(), "len of path %zu", followedPathTotal.poses.size());
        if (followedPathTotal.poses.size() >= 6) {
            constructFollowedPath();
            // RCLCPP_INFO(this->get_logger(), "len of path %zu", followedPathFiltered.poses.size());
            followedPathTotal.poses.clear();
        }

        geometry_msgs::msg::PoseStamped poseStamped;
        poseStamped.header = odom.header;  // Keep the same timestamp and frame
        poseStamped.header.stamp = get_clock()->now();
        poseStamped.pose = odom.pose.pose;  // Extract the pose
        followedPathTotal.poses.push_back(poseStamped);
        followedPathTotal.header.stamp = poseStamped.header.stamp;
        followedPathTotal.header.frame_id = odom.header.frame_id;

    }
    // RCLCPP_INFO(this->get_logger(), "3");

    prevRobotPos = robotPos;
}


void PathFollowing::constructFollowedPath(){
    followedPathFiltered.header = followedPathTotal.header;
    followedPathFiltered.poses.push_back(followedPathTotal.poses[5]);
    pubRobotPath->publish(followedPathFiltered);
}


void PathFollowing::smoothPath(const nav_msgs::msg::Path &path){
    std::lock_guard<std::mutex> guard(std::mutex);
    // if (!pathReceived) {
    //RCLCPP_INFO(this->get_logger(), "smoothing path of size %zu", path.poses.size());

    nav_msgs::msg::Path processedPath = processPath(path);

    smoothedPath = processedPath.poses;

    pubSmoothPath->publish(processedPath);
    // pathReceived = true;
    // }
}


double PathFollowing::euclDistance(const geometry_msgs::msg::Pose robotPose, const geometry_msgs::msg::PoseStamped pathPose){
    auto robotPosition = robotPose.position;
    auto pathPosition = pathPose.pose.position;

    return sqrt(pow(robotPosition.x - pathPosition.x, 2) + pow(robotPosition.y - pathPosition.y, 2));
}


double PathFollowing::findDesiredAngle(const geometry_msgs::msg::Pose &robotPose) {
    geometry_msgs::msg::PoseStamped nearestT;
    double minDist = std::numeric_limits<double>::max();
    int minIndex;

    for (size_t i = 0; i < smoothedPath.size() - 5; i++) {

        geometry_msgs::msg::PoseStamped pathPose = smoothedPath[i];
        double dist = euclDistance(robotPose, pathPose);
        if (dist < minDist){
            minDist = dist;
            minIndex = i;
        }
    }


    Vec2f p1 = smoothedPath[minIndex].pose.position;
    Vec2f p2 = smoothedPath[minIndex + 1].pose.position;
    Vec2f p5 = smoothedPath[minIndex + 5].pose.position;

    Vec2f Xt = (p2 - p1).normalized();

    Vec2f Xn = p1 - Vec2f(robotPose.position);

    double xn = perpDotProduct(Xn, Xt);

    double k = 3.0;

    double phiC = std::atan(- k * xn);

    pubPath1->publish(smoothedPath[minIndex]);
    pubPath2->publish(smoothedPath[minIndex + 1]);

    return phiC + (p5 - p2).angle(); //Xt.angle();
}



void PathFollowing::sendControlCommand(double speed, double omega, double error) {
    geometry_msgs::msg::Twist cmdVelMsg;
    // RCLCPP_INFO(this->get_logger(), "error: %.2f", error);
    //RCLCPP_INFO(this->get_logger(), "following path");
    // RCLCPP_INFO(this->get_logger(), "obstacle detected: %b", obstacleDetected);
    if (!(obstacleDetected || isPathBlocked)) {
        cmdVelMsg.linear.x = speed * std::max(1 - std::abs(error), 0.0) / M_PI;
        cmdVelMsg.angular.z = omega;  // Use PID output for angular velocity
    } else if (backwards){
        if (isReversing) {
            // Prüfe, ob mindestens 20 cm zurückgefahren wurde
            if (abs(startX - robotPos.x) >= 0.05) {
                isReversing = false;  // Rückwärtsbewegung beenden
                backwards = false;
                cmdVelMsg.linear.x = 0.0;
            } else {
                cmdVelMsg.linear.x = -0.3;  // Weiter rückwärts fahren
            }
        }
        //cmdVelMsg.angular.z = omega;
    } else if (obstacleDetected){
        cmdVelMsg.linear.x = 0.0;
        cmdVelMsg.angular.z = omega;
    }

    pubCmdVel->publish(cmdVelMsg);
}


void PathFollowing::robotPoseCallback(const geometry_msgs::msg::Pose &robotPose){
    write_plot_data::PlotDataWriter dataWriter = write_plot_data::PlotDataWriter("/home/praktikum7/prmr_group7_ws/src/path_following/src/irl_data.txt");
    // RCLCPP_INFO(this->get_logger(), "1.0");
    maxSpeed = 1.25;
    if (!smoothedPath.empty()) {
        double desiredAngle = findDesiredAngle(robotPose);
        // RCLCPP_INFO(this->get_logger(), "1.1");

        double robotYaw = tf2::getYaw(robotPose.orientation);
        // RCLCPP_INFO(this->get_logger(), "1.2");

        // Compute rotational velocity using PID
        static rclcpp::Time last_time = this->now();
        rclcpp::Time current_time = this->now();
        double dt = (current_time - last_time).seconds();
        last_time = current_time;

        pid.setKp(get_parameter("Kp").as_double());
        pid.setKi(get_parameter("Ki").as_double());
        pid.setKd(get_parameter("Kd").as_double());

        // RCLCPP_INFO(this->get_logger(), "Kp %.2f, Ki %.2f, Kd %.2f", pid.getKp(), pid.getKi(), pid.getKd());

        double error = std::remainder(desiredAngle - robotYaw, 2 * std::numbers::pi);
        double omega = pid.compute(error, dt);

        omega = std::clamp(omega, -M_PI / 6, M_PI / 6);
        // RCLCPP_INFO(this->get_logger(), "1.3");


        // RCLCPP_INFO(this->get_logger(), "desired angle: %.4f rad (%.2f°), omega: %.4f",
        //             desiredAngle, desiredAngle * (180.0 / M_PI), omega);

        // slow robot if close to goal, stop if reached goal


        if (euclDistance(robotPose, smoothedPath.back()) < 1.0) {
            sendControlCommand(maxSpeed * 0.3, omega, error);
            if (euclDistance(robotPose, smoothedPath.back()) < 0.1) {
                sendControlCommand(0.0, 0.0, 0.0);
                playedStartSound = false;
                playSounds();
               // RCLCPP_INFO(this->get_logger(), "END OF PATH REACHED");
                rclcpp::shutdown();
            }
        } else {
            sendControlCommand(maxSpeed, omega, error);  // Send computed velocity to the robot
        }
        // RCLCPP_INFO(this->get_logger(), "1.4");

        // RCLCPP_INFO(this->get_logger(), "robot yaw: %.2f", robotYaw);
        // RCLCPP_INFO(this->get_logger(), "desired yaw: %.2f", desiredAngle);

        dataWriter.write(robotYaw, desiredAngle, desiredAngle - robotYaw);
        // RCLCPP_INFO(this->get_logger(), "1.5");
        playSounds();
        setLed();

    }

}



