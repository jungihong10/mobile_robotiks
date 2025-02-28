#include "../include/mapping/mapping.hpp"
#include <geometry_msgs/msg/point_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <algorithm>
#include <cmath>

using namespace std::chrono_literals;

Mapping::Mapping()
    : Node("grid_occupancy") // Set node name to "grid_occupancy"
{
    pubGrid = create_publisher<nav_msgs::msg::OccupancyGrid>("/Grid", 10);

    // Map initialization
    int mapWidthInM = 30;
    int mapHeightInM = 30;
    double mapResolution = .05;

    map.info.width = mapWidthInM * (1/mapResolution);
    map.info.height = mapHeightInM * (1/mapResolution);
    map.header.frame_id = "odom";  // Reference frame for the map
    map.info.resolution = mapResolution;   // Resolution (each cell represents 0.25 meters)

    map.info.origin.position.x = - (.5 * mapWidthInM);
    map.info.origin.position.y = - (.5 * mapHeightInM);
    map.info.origin.position.z = 0.0;
    map.info.origin.orientation.w = 1.0;  // Identity rotation (no rotation)

    // Initialize all cells to 50 (unknown)
    map.data.resize(map.info.width * map.info.height, 50);

    // Initialize TF2 buffer and listener
    tf2Buffer = std::make_shared<tf2_ros::Buffer>(get_clock());
    std::shared_ptr<tf2_ros::CreateTimerROS> timer_interface =
        std::make_shared<tf2_ros::CreateTimerROS>(get_node_base_interface(),
                                                  get_node_timers_interface());
    tf2Buffer->setCreateTimerInterface(timer_interface);
    tf2Listener = std::make_shared<tf2_ros::TransformListener>(*tf2Buffer);

    // Subscribe to laser scan topic
    subLaser.subscribe(this, "/scan");
    tf2MessageFilter =
        std::make_shared<tf2_ros::MessageFilter<sensor_msgs::msg::LaserScan>>(
            subLaser, *tf2Buffer, "odom", 100, get_node_logging_interface(),
            get_node_clock_interface(), 500ms);
    tf2MessageFilter->setTolerance(100ms);
    tf2MessageFilter->registerCallback(&Mapping::laserCallback, this);
}


void Mapping::laserCallback(const sensor_msgs::msg::LaserScan &scan) {
    std::lock_guard<std::mutex> guard(mutex);
    prevPrevLaserPoints = prevLaserPoints;
    prevLaserPoints = laserPoints;
    laserPoints.clear();

    // Assuming robot's position is available in the "base_link" frame or another reference frame
    geometry_msgs::msg::PoseStamped robotPoseStamped;
    robotPoseStamped.header.stamp = scan.header.stamp;  // Sync with laser scan timestamp
    robotPoseStamped.header.frame_id = "base_link";  // Or the frame where robot position is available

    // Transform robot's position to "odom" frame
    geometry_msgs::msg::PoseStamped robotPoseInOdom;
    try {
        tf2Buffer->transform(robotPoseStamped, robotPoseInOdom, "odom", 500ms);  // Transform to "odom" frame
    } catch (const tf2::TransformException &ex) {
        // RCLCPP_WARN(this->get_logger(), "Could not transform robot pose to odom frame: %s", ex.what());
        return;
    }

    // Convert robot position to Vec2f (in the odom frame)
    Vec2f robotPos(robotPoseInOdom.pose.position.x, robotPoseInOdom.pose.position.y);

    for (unsigned int i = 0; i < scan.ranges.size(); i++) {
        float angle = scan.angle_min + i * scan.angle_increment;
        float angleDeg = angle * 180 / M_PI;
        Vec2f laser;

        // Ignore invalid ranges (NaN, inf, out of bounds)
        if ((std::isnan(scan.ranges[i]) || scan.ranges[i] > 4.0 || std::isinf(scan.ranges[i])))
        {
            // RCLCPP_INFO(this->get_logger(), "angle in rad: %f and in degrees: %f", angle, angle * 180 / M_PI);
            if (angleDeg < 45 && angleDeg > -45) {
                laser = 3.8 * Vec2f::fromAngle(angle);
                flagPoints.push_back(false);
                // RCLCPP_INFO(this->get_logger(), "angle in rad: %f and in degrees: %f", angle, angle * 180 / M_PI);
            } else {
                continue;
            }
        } else if (scan.ranges[i] < .3) {
            continue;
        } else {
            // Convert laser scan to Cartesian coordinates (relative to robot)
            flagPoints.push_back(true);
            laser = scan.ranges[i] * Vec2f::fromAngle(angle);
        }

        // Transform laser point to "odom" frame
        std_msgs::msg::Header header = scan.header;
        header.stamp = header.stamp + rclcpp::Duration::from_seconds(i * scan.time_increment);

        // Transform laser point to "odom" frame
        Vec2f pOdom = tf2Buffer->transform(laser.toGeometryMsgPointStamped(header), "odom");

        laserPoints.push_back(pOdom);
    }

    // Call update to process the laser points and update the map
    update(robotPos);
    flagPoints.clear();
}

void Mapping::update(const Vec2f &robotPos) {
    // Define distance function
    auto dist = [](const Vec2f& a, const Vec2f& b) {
        return std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2));
    };

    // Parameters
    float pOcc = 0.9f;  // Probability of occupancy
    float pFree = 0.1f; // Probability of free space
    float res = map.info.resolution; // Resolution of the map
    float mapOriginX = map.info.origin.position.x; // X coordinate of the grid origin
    float mapOriginY = map.info.origin.position.y; // Y coordinate of the grid origin

    // Convert robot position to grid cell coordinates
    int cellX = static_cast<int>((robotPos.x - mapOriginX) / res);
    int cellY = static_cast<int>((robotPos.y - mapOriginY) / res);
    // Mark robot's cell as free (0)
    int robotMapPos = cellY * map.info.width + cellX;

    if ((abs(cellX) < map.info.width) & ((abs(cellY) < map.info.height))){
        map.data[robotMapPos] = static_cast<int8_t>(.01 * 100);
    }

    for (unsigned int i = 0; i < laserPoints.size(); i++){
        // Calculate distance from robot to laser point
        auto& point = laserPoints[i];
        bool flagI = flagPoints[i];

        float distanceZ = dist(robotPos, point);

        // Check if likely noise or notz
      // if (!(std::abs(point.y) < noiseCheckWidth / 2 && point.x < noiseCheckLength)) {
            if (prevPrevLaserPoints.size() > 0) {
                unsigned long i = find(laserPoints.begin(), laserPoints.end(), point) - laserPoints.begin();
                // RCLCPP_INFO(this->get_logger(), "prev and current number of points: %zu, %zu", prevPrevLaserPoints.size(), laserPoints.size());
                unsigned long j = i - 5;
                bool isNoise = true;
                while (i > 5 && j < i + 5 && i < laserPoints.size() && j < prevPrevLaserPoints.size()){
                    float distance = dist(point, prevPrevLaserPoints[j]);
                    if (distance < .1){
                        isNoise = false;
                    }
                    j += 1;
                }
                if (isNoise) {
                    continue;
                }
            }
       //}

        // Process if laser point is within valid range
        if (distanceZ > 0.3 && distanceZ < 4.1) {
            // Convert laser point to grid cell coordinates
            int laserCellX = static_cast<int>((point.x - mapOriginX) / res);
            int laserCellY = static_cast<int>((point.y - mapOriginY) / res);
            int laserMapPos = laserCellY * map.info.width + laserCellX;

            // Update laser point cell towards occupied if conditions are met
            if (laserMapPos >= 0 && static_cast<size_t>(laserMapPos) < map.data.size() && flagI) {
                float priorProb = static_cast<float>(map.data[laserMapPos]) / 100;

                // Calculate the distance from the robot to the center of the current grid cell (m_l)
                float cellCenterX = laserCellX * res + mapOriginX + res / 2;
                float cellCenterY = laserCellY * res + mapOriginY + res / 2;
                float distToCell = dist(robotPos, Vec2f(cellCenterX, cellCenterY));

                // Compare the measured distance (z_t) to the distance to the grid cell center (dist(x_t, m_l))
                if (std::abs(distanceZ - distToCell) < res / 2) {
                    // Update towards occupied
                    float newProb = (pOcc * priorProb) / (pOcc * priorProb + pFree * (1 - priorProb));
                    map.data[laserMapPos] = static_cast<int8_t>(newProb * 100);
                } else {
                    // Retain prior probability for cells not covered by z_t
                    map.data[laserMapPos] = static_cast<int8_t>(priorProb * 100);
                }
            }

            // Traverse the grid and mark cells between robot and laser point as free
            GridTraversal gridTraversal(cellX, cellY, laserCellX, laserCellY);
            int i, j;
            while (gridTraversal.next(i, j)) {
                int mapPos = j * map.info.width + i;

                if (mapPos >= 0 && static_cast<size_t>(mapPos) < map.data.size()) {
                    float priorProb = static_cast<float>(map.data[mapPos]) / 100;
                    float cellDist = dist(robotPos, Vec2f(
                                                        i * res + mapOriginX,
                                                        j * res + mapOriginY
                                                        ));

                    if (distanceZ >= cellDist) {
                        // Mark as free
                        if (map.data[mapPos] > 50){
                            float newProb = fmax(fmax((pFree * priorProb) / (pFree * priorProb + pOcc * (1 - priorProb)), priorProb - 0.01), 0.01);
                            map.data[mapPos] = static_cast<int8_t>(newProb * 100);
                        } else {
                            float newProb = fmax((pFree * priorProb) / (pFree * priorProb + pOcc * (1 - priorProb)), 0.01);
                            map.data[mapPos] = static_cast<int8_t>(newProb * 100);
                        }
                    } else {
                        // Retain prior probability
                        map.data[mapPos] = static_cast<int8_t>(priorProb * 100);
                    }
                }
            }
        }
    }

    // Publish the updated map
    map.header.stamp = this->get_clock()->now();
    pubGrid->publish(map);

    // RCLCPP_INFO(this->get_logger(), "Publishing map with frame_id: %s", map.header.frame_id.c_str());
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto mapping_node = std::make_shared<Mapping>();

    rclcpp::Rate loop_rate(30);
    while (rclcpp::ok()) {
        rclcpp::spin_some(mapping_node);
        loop_rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}
