#include <rclcpp/rclcpp.hpp>
#include <memory>
#include "../include/path_planning/path_planning.h"

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    std::shared_ptr<PathPlanner> node = std::make_shared<PathPlanner>();

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
