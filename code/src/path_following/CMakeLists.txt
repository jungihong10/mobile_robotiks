cmake_minimum_required(VERSION 3.8)
project(path_following)

set(CMAKE_CXX_STANDARD 23)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
  #add_compile_options(-fsanitize=address)
  #add_link_options(-fsanitize=address)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(kobuki_ros_interfaces REQUIRED)
find_package(ALGLIB REQUIRED)

include_directories(include/${PROJECT_NAME})

add_executable(path_following
  src/main.cpp
  src/path_following.cpp
  src/path_processing.cpp
  src/plot_data.cpp
  src/pid_controller.cpp
)

# install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_target_dependencies(path_following rclcpp std_msgs geometry_msgs nav_msgs sensor_msgs tf2_ros tf2_geometry_msgs message_filters kobuki_ros_interfaces ALGLIB)
target_link_libraries(path_following alglib)

install(TARGETS path_following
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
