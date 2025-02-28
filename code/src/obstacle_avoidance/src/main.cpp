#include <memory>
#include "obstacle_avoidance/obstacle_avoidance.h"
#include <rclcpp/rclcpp.hpp>


#define GOAL_POSITION_X 20.0
#define GOAL_POSITION_Y -20.0
#define ROBOT_SPEED 0.3

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  std::shared_ptr<ObstacleAvoidance> node =
      std::make_shared<ObstacleAvoidance>();
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
