#include <rclcpp/rclcpp.hpp>
#include <memory>
#include "path_following.h"

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    std::shared_ptr<PathFollowing> node = std::make_shared<PathFollowing>();

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
