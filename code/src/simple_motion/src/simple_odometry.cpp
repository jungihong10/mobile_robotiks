#include <cstdio>
#include <chrono>
#include <string>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/string.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <cmath>

using namespace std::chrono_literals;
using std::placeholders::_1;

class simple_odometry : public rclcpp::Node
{
public:
    simple_odometry()
        : Node("simple_odometry"), start_pos_x(0.0), start_pos_y(0.0), moving_forward(true), initial_position_set(false), speed(0.2)
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&simple_odometry::odom_callback, this, _1));
    }

private:
    double start_pos_x;
    double start_pos_y;
    double speed;
    bool moving_forward;
    bool initial_position_set;

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        double current_pos_x = msg->pose.pose.position.x;
        double current_pos_y = msg->pose.pose.position.y;


        // ignore all the stuff with angular velocity; we noticed a drift leftwards
        // while driving straight ahead and tried to do something to correct it
        // double z_rotation = msg->twist.twist.angular.z;

        // set current x and y position as start position on first iteration
        if (!initial_position_set) {
            start_pos_x = current_pos_x;
            start_pos_y = current_pos_y;
            initial_position_set = true;
        }

        // calculate euclidean distance in 2d space (we assume the robot traveled in a straight line)
        double distance = std::sqrt(std::pow((current_pos_x - start_pos_x), 2) + std::pow((current_pos_y - start_pos_y), 2));
        // double counter_rotation = -1 * z_rotation;
        // double new_rotation;

        auto message = geometry_msgs::msg::Twist();

        // if distance of 2m reached, stop movement, invert boolean moving_forward
        // and set new start position
        if (distance >= 2.0)
        {
            message.linear.x = 0.0;
            moving_forward = !moving_forward;
            start_pos_x = current_pos_x;
            start_pos_y = current_pos_y;
            RCLCPP_INFO(this->get_logger(), "Traveled %.2f meters, changing direction", distance);

        }

        if (moving_forward)
        {
            message.linear.x = speed;
            // message.angular.z = counter_rotation;
            // new_rotation = msg->twist.twist.angular.z;
            RCLCPP_INFO(this->get_logger(), "Moving forward, distance is %.2f meters", distance);
            // RCLCPP_INFO(this->get_logger(), "Rotation is %.4f", new_rotation);

        }
        else
        {
            message.linear.x = -speed;
            // message.angular.z = counter_rotation;
            // new_rotation = msg->twist.twist.angular.z;
            RCLCPP_INFO(this->get_logger(), "Moving backwards, distance is %.2f meters", distance);
            // RCLCPP_INFO(this->get_logger(), "Rotation is %.4f", new_rotation);

        }

        publisher_->publish(message);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<simple_odometry>());
    rclcpp::shutdown();
    return 0;
}
