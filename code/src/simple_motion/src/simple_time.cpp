#include <cstdio>
#include <chrono>
#include <string>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/string.hpp>
#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;


class simple_time : public rclcpp::Node
{
public:
    simple_time()
        : Node("simple_time"), count_(0)
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

        // we use count_ to switch between publishing a forward and backward velocity
        auto timer_callback =
            [this]() -> void {
            auto message = geometry_msgs::msg::Twist();
            // move forwards while the count is even, else move backwards
            if (count_ % 2 == 0) {
                RCLCPP_INFO(this->get_logger(), "forward");
                message.linear.set__x(.1);
            } else {
                RCLCPP_INFO(this->get_logger(), "backwards");
                message.linear.set__x(-.1);
            }
            this->publisher_->publish(message);
            // iterate count_ after the loop
            ++count_;
            // std::chrono::seconds dura(2);
            // std::this_thread::sleep_for( dura );

        };
        timer_ = this->create_wall_timer(2000ms, timer_callback);
    }

private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    size_t count_;
};



class simple_odometry : public rclcpp::Node
{
public:
    simple_odometry()
        : Node("simple_odometry"), count_(0)
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        auto timer_callback =
            [this]() -> void {
            auto message = geometry_msgs::msg::Twist();
            if (count_ % 2 == 0) {
                RCLCPP_INFO(this->get_logger(), "forward");
                message.linear.set__x(.1);
            } else {
                RCLCPP_INFO(this->get_logger(), "backwards");
                message.linear.set__x(-.1);
            }
            //message.data = "Hello, world! " + std::to_string(this->count_++);
            RCLCPP_INFO(this->get_logger(), "Publishing");
            this->publisher_->publish(message);
            ++count_;
        };
        timer_ = this->create_wall_timer(2000ms, timer_callback);
    }

private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    size_t count_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<simple_time>());
    rclcpp::shutdown();
    return 0;
}



