#include <builtin_interfaces/msg/time.hpp>
#include <color_names/color_names.hpp>
#include <functional>
#include "../include/obstacle_avoidance/obstacle_avoidance.h"
#include <std_msgs/msg/color_rgba.hpp>
#include <tf2/utils.h>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <cmath>
#include <algorithm>

using namespace std::chrono_literals;

ObstacleAvoidance::ObstacleAvoidance()
    : Node("obstacle_avoidance"), odomInit(false), laserInit(false),
    goalPos(5, 4), dangerZoneWidth(0.3), dangerZoneLength(0.2), potentialField(1.0, 0.2, 3) // Set initial parameters
{
    pubCmdVel = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel2", 10);
    pubDangerZone = create_publisher<visualization_msgs::msg::Marker>("dangerZone", 10);
    pubGoal = create_publisher<visualization_msgs::msg::Marker>("goalVec", 10);
    pubTotalForce = create_publisher<visualization_msgs::msg::Marker>("totalVec", 10);
    pubMarkerArray = create_publisher<visualization_msgs::msg::MarkerArray>("repArray", 10);

    subGoal = this->create_subscription<geometry_msgs::msg::PoseStamped>("/goal_pose",
                                                                         1,
                                                                         std::bind(&ObstacleAvoidance::goalCallback, this, std::placeholders::_1));

    subOdom = create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 1,
        std::bind(&ObstacleAvoidance::odomCallback, this, std::placeholders::_1));

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
    tf2MessageFilter->registerCallback(&ObstacleAvoidance::laserCallback, this);
}

void ObstacleAvoidance::goalCallback(const geometry_msgs::msg::PoseStamped goalMsg){
    goalPos = Vec2f(goalMsg.pose.position.x, goalMsg.pose.position.y);
}

void ObstacleAvoidance::odomCallback(const nav_msgs::msg::Odometry &odom) {
    std::lock_guard<std::mutex> guard(mutex);
    RCLCPP_INFO(this->get_logger(), "running odom");

    robotPos = Vec2f(odom.pose.pose.position.x, odom.pose.pose.position.y);
    robotYaw = tf2::getYaw(odom.pose.pose.orientation);

    odomInit = true;
    if (odomInit && laserInit) {
        process();
    }

}

void ObstacleAvoidance::laserCallback(const sensor_msgs::msg::LaserScan &scan) {
    std::lock_guard<std::mutex> guard(mutex);
    lastScan = scan;
    laserPoints.clear();
    obstacleDetected = false;

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

        if (std::abs(laser.y) < dangerZoneWidth / 2 && laser.x > 0.15 && laser.x < dangerZoneLength) {
            obstacleDetected = true;
        }
    }

    laserInit = true;
    if (odomInit && laserInit) {
        process();
    }
}


void ObstacleAvoidance::process() {
    geometry_msgs::msg::Twist twistMsg;
    Vec2f goalForce = potentialField.calculateAttractiveForce(goalPos, robotPos);

    visualizeMarkers();
    publishGoalVector(goalForce);
    Vec2f repForce = publishObjectVector(laserPoints);
    Vec2f totalForce = goalForce + repForce;
    publishTotalForce(totalForce);

    double angleDiff = totalForce.angle() - robotYaw;

    maxSpeed = 0.3;
    if (obstacleDetected) {
    RCLCPP_INFO(this->get_logger(), "obstacle detected, avoiding!");

    double escapeAngle = totalForce.angle();

    if (backupCounter < 20) {
        // ① 후진 (stuck 탈출)
        twistMsg.linear.x = -0.1;
        twistMsg.angular.z = 0.0;
        backupCounter++;
    } else {
        // ② 회전하며 우회
        twistMsg.linear.x = 0.0;
        twistMsg.angular.z = std::clamp(escapeAngle, -std::numbers::pi / 8, std::numbers::pi / 8);

        // 장애물이 사라지면 상태 초기화
        if (!obstacleDetected) {
            backupCounter = 0;
        }
    }
}

    pubCmdVel->publish(twistMsg);
}


void ObstacleAvoidance::visualizeMarkers() {
    visualization_msgs::msg::Marker dangerZone;
    dangerZone.header.frame_id = "base_link";
    dangerZone.header.stamp = this->get_clock()->now();
    dangerZone.ns = "my_namespace";
    dangerZone.id = 0;
    dangerZone.type = visualization_msgs::msg::Marker::CUBE;
    dangerZone.action = visualization_msgs::msg::Marker::ADD;

    // Set the pose of the marker
    dangerZone.pose.position.x = dangerZoneLength / 2;  // 1 meter in front of the robot
    dangerZone.pose.position.y = 0.0;
    dangerZone.pose.position.z = 0.0;
    dangerZone.pose.orientation.x = 0.0;
    dangerZone.pose.orientation.y = 0.0;
    dangerZone.pose.orientation.z = 0.0;
    dangerZone.pose.orientation.w = 1.0;

    // Define the rectangle's dimensions
    dangerZone.scale.x = dangerZoneLength;  // Length
    dangerZone.scale.y = dangerZoneWidth;  // Width
    dangerZone.scale.z = 0.1;  // Height (thin rectangle)

    // Set the color
    dangerZone.color.r = 0.0f;
    dangerZone.color.g = 1.0f;
    dangerZone.color.b = 0.0f;
    dangerZone.color.a = 0.5;  // Make it slightly transparent

    pubDangerZone->publish(dangerZone);

}

void ObstacleAvoidance::publishGoalVector(Vec2f force) {
    visualization_msgs::msg::Marker goalVec;
    goalVec.header.frame_id = "odom";
    goalVec.header.stamp = get_clock()->now();
    goalVec.ns = "goal_vector";
    goalVec.id = 0;
    goalVec.type = visualization_msgs::msg::Marker::ARROW;
    goalVec.action = visualization_msgs::msg::Marker::ADD;

    geometry_msgs::msg::Point start, end;
    start.x = robotPos.x;
    start.y = robotPos.y;
    end.x = robotPos.x + force.x;
    end.y = robotPos.y + force.y;

    goalVec.points.push_back(start);
    goalVec.points.push_back(end);

    goalVec.scale.x = 0.05; // Arrow shaft diameter
    goalVec.scale.y = 0.1;  // Arrow head diameter
    goalVec.color.g = 1.0;  // Green color
    goalVec.color.a = .9;

    pubGoal->publish(goalVec);
}

void ObstacleAvoidance::publishTotalForce(Vec2f force) {
    visualization_msgs::msg::Marker resultantVec;
    resultantVec.header.frame_id = "odom";
    resultantVec.header.stamp = get_clock()->now();
    resultantVec.ns = "resultant_force";
    resultantVec.id = 0;
    resultantVec.type = visualization_msgs::msg::Marker::ARROW;
    resultantVec.action = visualization_msgs::msg::Marker::ADD;

    geometry_msgs::msg::Point start, end;
    start.x = robotPos.x;
    start.y = robotPos.y;
    end.x = robotPos.x + force.x;
    end.y = robotPos.y + force.y;

    resultantVec.points.push_back(start);
    resultantVec.points.push_back(end);

    resultantVec.scale.x = 0.05; // Arrow shaft diameter
    resultantVec.scale.y = 0.1;  // Arrow head diameter
    resultantVec.color.r = 0.0;  // Blue color for visualization
    resultantVec.color.b = 1.0;
    resultantVec.color.a = .9;

    pubTotalForce->publish(resultantVec); // Using pubMarkerArray for consistency
}

Vec2f ObstacleAvoidance::publishObjectVector(const std::vector<Vec2f>& laserPoints) {
    const int numSegments = 10;  // Anzahl der Segmente (kann abhängig von der Winkelteilung angepasst werden)
    std::vector<std::optional<Vec2f>> closestInSegment(numSegments);
    std::vector<std::optional<Vec2f>> segmentRepForces;
    const int segmentSize = laserPoints.size() / numSegments;
    const float MAX_RANGE = 4.1;

    for (int segmentIndexMarker = 0; segmentIndexMarker < numSegments; ++segmentIndexMarker) {
        float closestPointDistance = 100000.0;
        int closestPointID = -1;

        for (int j = segmentIndexMarker * segmentSize; j < (segmentIndexMarker + 1) * segmentSize; ++j) {
            Vec2f pointCoords = laserPoints[j];
            Vec2f botCoords = robotPos;

            float distanceFromBot = (pointCoords - botCoords).norm();

            if (distanceFromBot < closestPointDistance && distanceFromBot > .3 && distanceFromBot < MAX_RANGE){
                closestPointDistance = distanceFromBot;
                closestPointID = j;
            }
        }

        if (closestPointID != -1) {
            closestInSegment[segmentIndexMarker] = laserPoints[closestPointID];
        } else {
            closestInSegment[segmentIndexMarker] = std::nullopt;
        }
    }
    for (const auto& point : closestInSegment) {
        segmentRepForces.push_back(point? std::optional<Vec2f>(potentialField.calculateRepulsiveForce(*point, robotPos)) : std::nullopt);
    }

    visualization_msgs::msg::MarkerArray repArray;

    for (unsigned long i = 0; i < segmentRepForces.size(); i++) {
        if ( !segmentRepForces[i].has_value() || !closestInSegment[i].has_value()) {
            continue;
        }
        visualization_msgs::msg::Marker objectVec;
        objectVec.header.frame_id = "odom";
        objectVec.header.stamp = get_clock()->now();
        objectVec.id = i;
        objectVec.type = visualization_msgs::msg::Marker::ARROW;
        objectVec.action = visualization_msgs::msg::Marker::ADD;
        geometry_msgs::msg::Point start, end;
        start = closestInSegment[i]->toGeometryMsgPoint();
        end = (robotPos + *segmentRepForces[i]).toGeometryMsgPoint();

        objectVec.scale.x = .05 * segmentRepForces[i]->norm();
        objectVec.scale.y = .1 * segmentRepForces[i]->norm();
        objectVec.scale.z = .2 * segmentRepForces[i]->norm();
        objectVec.color.r = 1.0;
        objectVec.color.a = 0.5;

        objectVec.points = {start, end};

        repArray.markers.push_back(objectVec);
    }

    pubMarkerArray->publish(repArray);

    Vec2f repForce(0, 0);
    for (auto force : segmentRepForces) {
        if (force) {
            repForce += *force;
        }
    }

    return repForce;
}



// void ObstacleAvoidance::publishObjectVector(const sensor_msgs::msg::LaserScan &scan, const std::vector<Vec2f>& laserPoints) {
//     std::vector<Vec2f> segment;
//     std::vector<Vec2f> closestInSegment;
//     int numSegments = 16;  // Anzahl der Segmente (kann abhängig von der Winkelteilung angepasst werden)
//     int segmentSize = scan.ranges.size() / numSegments;

//     for (int segmentIndexMarker = 0; segmentIndexMarker < numSegments; ++segmentIndexMarker) {
//         segment.clear();
//         closestInSegment.clear();

//         //&& j < scan.ranges.size()
//         for (int j = segmentIndexMarker * segmentSize; j < (segmentIndexMarker + 1) * segmentSize; ++j) {
//             Vec2f pointCoords = laserPoints[j];
//             Vec2f botCoords = robotPos;

//             float distanceFromBot = std::sqrt(std::pow(botCoords.x - pointCoords.x, 2) + std::pow(botCoords.y - pointCoords.y, 2));

//             if (distanceFromBot > .15){
//                 segment.push_back(pointCoords);
//             }
//         }
//         RCLCPP_INFO(this->get_logger(), "num segments: %d", segment.size());

//         auto minIt = std::min_element(segment.begin(), segment.end(),
//                                           [](const Vec2f& a, const Vec2f& b) { return a.norm() < b.norm(); });
//         Vec2f nearestPoint = *minIt;

//         closestInSegment.push_back(nearestPoint);
//         Vec2f repulsiveForce = potentialField.calculateRepulsiveForce(closestInSegment, robotPos);

//         visualization_msgs::msg::Marker objectVec;
//             objectVec.header.frame_id = "odom";
//             objectVec.header.stamp = get_clock()->now();
//             objectVec.ns = "goal_vector";
//             objectVec.id = segmentIndexMarker;
//             objectVec.type = visualization_msgs::msg::Marker::ARROW;
//             objectVec.action = visualization_msgs::msg::Marker::ADD;

//             geometry_msgs::msg::Point start, end;
//             start.x = robotPos.x;
//             start.y = robotPos.y;
//             end.x = robotPos.x + nearestPoint.x;
//             end.y = robotPos.y + nearestPoint.y;

//             objectVec.points.push_back(start);
//             objectVec.points.push_back(end);

//             objectVec.scale.x = .05;
//             objectVec.scale.y = .1;
//             objectVec.color.r = 1.0;
//             objectVec.color.a = 0.7;


//             pubMarker->publish(objectVec);

//     }
// }

