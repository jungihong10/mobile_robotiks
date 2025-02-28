// #include <cstdio>
// #include <chrono>
// #include <string>
// #include <geometry_msgs/msg/twist.hpp>
// #include <std_msgs/msg/string.hpp>
// #include <rclcpp/rclcpp.hpp>
// #include <sensor_msgs/msg/laser_scan.hpp>
// #include <cmath>
// #include <iterator>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "../include/simple_laser/online_statistics.h"


class SimpleScan : public rclcpp::Node
{
public:
    SimpleScan() : Node("simple_scan")
    {
        subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&SimpleScan::scan_callback, this, std::placeholders::_1));

    }
private:

    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        int num_measurements = msg->ranges.size();
        float angle_increment = msg->angle_increment;
        float first_ray_angle = msg->angle_min;
        float last_ray_angle = msg->angle_max;

        RCLCPP_INFO(this->get_logger(), "Number of distance measurements per scan: %d", num_measurements);
        RCLCPP_INFO(this->get_logger(), "Angle difference btw two subsequent rays: %f", angle_increment);
        RCLCPP_INFO(this->get_logger(), "Orientation of the first ray: %f", first_ray_angle);
        RCLCPP_INFO(this->get_logger(), "Orientation of the last ray: %f", last_ray_angle);

    }


    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
};

// class SimpleScan : public rclcpp::Node
// {
// public:
//     SimpleScan() : Node("simple_scan"), forward_ray_index_(0)
//     {
//         subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
//             "/scan", 10, std::bind(&SimpleScan::scan_callback, this, std::placeholders::_1));
//     }

// private:
//     void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
//     {
//         float range_min = msg->range_min;
//         float range_max = msg->range_max;

//         // float first_ray_angle = msg->angle_min;
//         // float last_ray_angle = msg->angle_max;
//         // float angle_increment = msg->angle_increment;

//         // RCLCPP_INFO(this->get_logger(), "first Ray angle: %.5f rad", first_ray_angle);
//         // RCLCPP_INFO(this->get_logger(), "last Ray angle: %.5f rad", last_ray_angle);
//         // RCLCPP_INFO(this->get_logger(), "angle increment: %.5f rad", angle_increment);

//         // first Ray angle: -2.35619 rad
//         // last Ray angle: 2.09235 rad
//         // angle increment: 0.00614 rad

//         // range min: 0.0200
//         // range max: 5.6000

//         forward_ray_index_ = 405;

//         double forward_ray_distance = msg->ranges[forward_ray_index_];

//         if (std::isnan(forward_ray_distance) || std::isinf(forward_ray_distance) || forward_ray_distance < range_min || forward_ray_distance > range_max) {
//             RCLCPP_WARN(this->get_logger(), "Invalid distance measurement: %.4f", forward_ray_distance);
//             return;
//         }
//         statistics_.update(forward_ray_distance);

//         RCLCPP_INFO(this->get_logger(), "forward Ray Distance: %.5f m", forward_ray_distance);
//         RCLCPP_INFO(this->get_logger(), "mean Distance: %.5f m", statistics_.getMean());
//         RCLCPP_INFO(this->get_logger(), "standard Deviation: %.5f m", statistics_.getStandardDeviation());
//     }

//     rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
//     OnlineStatistics statistics_;
//     size_t forward_ray_index_;
// };




int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimpleScan>());
    rclcpp::shutdown();
    return 0;
}



