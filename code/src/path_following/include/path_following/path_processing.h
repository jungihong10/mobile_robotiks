#ifndef PATH_PROCESSING_H
#define PATH_PROCESSING_H

#include <nav_msgs/msg/path.hpp>

//#include <eigen3/Eigen/Core>
#include <interpolation.h>

nav_msgs::msg::Path processPath(const nav_msgs::msg::Path &path);

#endif
