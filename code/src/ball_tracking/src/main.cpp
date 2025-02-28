#include <rclcpp/rclcpp.hpp>
#include <memory>
#include "ball_tracker.h"

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    std::shared_ptr<BallTracker> node = std::make_shared<BallTracker>();
    //node->setRobotSpeed(ROBOT_SPEED);
    //node->setGoalPosition(GOAL_POSITION_X, GOAL_POSITION_Y);

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
