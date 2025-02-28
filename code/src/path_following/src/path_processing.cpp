#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <vector>
#include <cassert>
#include "path_processing.h"

nav_msgs::msg::Path processPath(const nav_msgs::msg::Path &path) {
    std::vector<double> p_;
    std::vector<double> q_;
    std::vector<double> p_prim_;
    std::vector<double> q_prim_;

    nav_msgs::msg::Path path_interp_;

    int N = path.poses.size();
    std::cout << "The original path has " << N << " poses." << std::endl;

    if (N < 2)
        return path_interp_;

    // first, copy the path waypoints to X_arr and Y_arr, and calculate the distance vector l_arr
    double X_arr[N], Y_arr[N], l_arr[N], l_arr_unif[N];
    double L = 0;
    geometry_msgs::msg::PoseStamped first_wp = path.poses.at(0);

    X_arr[0] = first_wp.pose.position.x;
    Y_arr[0] = first_wp.pose.position.y;
    l_arr[0] = 0;

    int insert_index = 1;
    for (std::size_t i = 1; i < N; i++) {
        geometry_msgs::msg::PoseStamped wp = path.poses.at(i);

        double dist = hypot(wp.pose.position.x - X_arr[insert_index-1], wp.pose.position.y - Y_arr[insert_index-1]);
        if (dist >= 1e-3) {
            X_arr[insert_index] = wp.pose.position.x;
            Y_arr[insert_index] = wp.pose.position.y;

            L += dist;
            l_arr[insert_index] = L;

            ++insert_index;
        } else {
            // two points were to close...
            std::cout << "dropping point (" << wp.pose.position.x << " / " << wp.pose.position.y <<
                ") because it is too close to the last point (" << X_arr[insert_index-1] << " / " << Y_arr[insert_index-1] << ")" << std::endl;
        }
    }
    // RCLCPP_INFO(rclcpp::get_logger("path_processing"), "Length of the unsmoothed path: %lf m", L); // -------------- print unsmoothed path length

    // calculate the arclength vector
    double f = std::max(0.001, L / (double) (N-1));
    for (std::size_t i = 0; i < N; i++)
        l_arr_unif[i] = i * f;

    // copy the X_arr, Y_arr, l_arr and l_arr_unif to alglib arrays
    alglib::real_1d_array X_alg, Y_alg, l_alg, l_alg_unif;
    alglib::real_1d_array x_s, y_s, x_s_prim, y_s_prim, x_s_sek, y_s_sek;

    X_alg.setcontent(N, X_arr);
    Y_alg.setcontent(N, Y_arr);
    l_alg.setcontent(N, l_arr);
    l_alg_unif.setcontent(N, l_arr_unif);

    // interpolate the path and find the derivatives
    // l_alg is the old base, and l_alg_unif the new one (uniform arclength)
    try {
        alglib::spline1dconvdiff2cubic(l_alg, X_alg, l_alg_unif, x_s, x_s_prim, x_s_sek);
        alglib::spline1dconvdiff2cubic(l_alg, Y_alg, l_alg_unif, y_s, y_s_prim, y_s_sek);
    } catch(const alglib::ap_error& error) {
        RCLCPP_FATAL_STREAM(rclcpp::get_logger("path_processing"), "alglib error: " << error.msg);
        throw error;
    }

    // smoothen the path
    alglib::spline1dinterpolant s1, s2;
    alglib::spline1dfitreport rep1, rep2;
    double rho = 3.0;
    alglib::ae_int_t info1, info2;
    alglib::real_1d_array x_sm, y_sm, l_alg_sm;

    // initialize the vectors x_sm and y_sm
    double h_arr[N], l_arr_sm[N];
    for (std::size_t i = 0; i < N; i++)
        h_arr[i] = 0.0;

    x_sm.setcontent(N, h_arr);
    y_sm.setcontent(N, h_arr);

    // calculate the smoothed spline interpolants s1 and s2
    try {
        alglib::spline1dfitpenalized(l_alg_unif, x_s, N, rho, info1, s1, rep1);
        alglib::spline1dfitpenalized(l_alg_unif, y_s, N, rho, info2, s2, rep2);
    } catch(const alglib::ap_error& error) {
        RCLCPP_FATAL_STREAM(rclcpp::get_logger("path_processing"), "alglib error: " << error.msg);
        throw error;
    }

    // use the interpolants and the base of the interpolated path l_alg_unif to calculate
    // the smooth values x_sm and y_sm
    try {
        x_sm[0] = alglib::spline1dcalc(s1, l_alg_unif[0]);
        y_sm[0] = alglib::spline1dcalc(s2, l_alg_unif[0]);
        double L_sm = 0.0;

        for (std::size_t i = 1; i < N; i++) {
            x_sm[i] = alglib::spline1dcalc(s1, l_alg_unif[i]);
            y_sm[i] = alglib::spline1dcalc(s2, l_alg_unif[i]);

            double dist = hypot(x_sm[i] - x_sm[i-1], y_sm[i] - y_sm[i-1]);

            L_sm += dist;
            l_arr_sm[i] = L_sm;
        }
        // RCLCPP_INFO(rclcpp::get_logger("path_processing"), "Length of the smoothed path: %lf m", L_sm);           //------------ print smooth path length
    } catch(const alglib::ap_error& error) {
        RCLCPP_FATAL_STREAM(rclcpp::get_logger("path_processing"), "alglib error: " << error.msg);
        throw error;
    }

    l_alg_sm.setcontent(N, l_arr_sm);

    // interpolate over a new grid l_arr_unif_sm[M]
    // this new grid is the arclength vector of the smoothed path, but with a different number of points (M)
    int M = 3*N;
    std::cout << "The interpolated path has " << M << " poses." << std::endl;

    double l_arr_unif_sm[M];
    double h = std::max(0.001, L / (double) (M-1));

    for (std::size_t i = 0; i < M; i++)
        l_arr_unif_sm[i] = i * h;

    alglib::real_1d_array l_alg_unif_sm;
    l_alg_unif_sm.setcontent(M, l_arr_unif_sm);

    try {
        alglib::spline1dconvdiff2cubic(l_alg_unif, x_sm, l_alg_unif_sm, x_s, x_s_prim, x_s_sek);
        alglib::spline1dconvdiff2cubic(l_alg_unif, y_sm, l_alg_unif_sm, y_s, y_s_prim, y_s_sek);
    } catch(const alglib::ap_error& error) {
        RCLCPP_FATAL_STREAM(rclcpp::get_logger("path_processing"), "alglib error: " << error.msg);
        throw error;
    }

    // define path components, its derivatives, and curvilinear abscissa, then calculate the path curvature
    for (uint i = 0; i < M; ++i) {
        p_.push_back(x_s[i]);
        q_.push_back(y_s[i]);

        geometry_msgs::msg::PoseStamped pose;
        pose.pose.position.x = x_s[i];
        pose.pose.position.y = y_s[i];
        pose.header = path.header;
        //pose.header.seq = i;

        path_interp_.poses.push_back(pose);
        path_interp_.header = path.header;

        p_prim_.push_back(x_s_prim[i]);
        q_prim_.push_back(y_s_prim[i]);
    }

    assert(p_prim_.size() == M);
    assert(q_prim_.size() == M);
    assert(p_.size() == M);
    assert(q_.size() == M);

    return path_interp_;
}



// #include "path_following.h"
// #include "path_processing.h"
// #include "write_plot_data.hpp"
// #include <tf2/utils.h>
// #include <tf2/LinearMath/Quaternion.h>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
// #include <cmath>
// #include <chrono>

// using std::placeholders::_1;
// using namespace std::chrono;

// PathFollowing::PathFollowing(): Node("path_following"), pid(1.0, 0.0, 0.0) { // Kp, Ki, Kd
//     declare_parameter("Kp", 1.0);
//     declare_parameter("Ki", 0.0);
//     declare_parameter("Kd", 0.0);

//     RCLCPP_INFO(this->get_logger(), "Kp %.2f, Ki %.2f, Kd %.2f", pid.getKp(), pid.getKi(), pid.getKd());

//     subPath = this->create_subscription<nav_msgs::msg::Path>("/Path",
//                                                              10,
//                                                              std::bind(&PathFollowing::smoothPath, this, _1));

//     pubSmoothPath = create_publisher<nav_msgs::msg::Path>("/smoothPath", 10);
//     pubPath1 = create_publisher<geometry_msgs::msg::PoseStamped>("/pathPoint1", 10);
//     pubPath2 = create_publisher<geometry_msgs::msg::PoseStamped>("/pathPoint2", 10);

//     subOdom = create_subscription<nav_msgs::msg::Odometry>(
//         "/odom", 10,
//         std::bind(&PathFollowing::odomCallback, this, _1));

//     pubCmdVel = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

//     subObstAvoid = create_subscription<geometry_msgs::msg::Twist>("/cmd_vel2", 10, std::bind(&PathFollowing::avoidOrNo, this, _1));
//     subCostmap = create_subscription<nav_msgs::msg::OccupancyGrid>("/costmap", 10, std::bind(&PathFollowing::costmapCallBack, this, _1));

//     pubRobotPath = create_publisher<nav_msgs::msg::Path>("/followedPath", 10);
// }


// void PathFollowing::costmapCallBack(const nav_msgs::msg::OccupancyGrid &costMap){
//     RCLCPP_INFO(this->get_logger(), "receiving costmap");
//     costmap = costMap;
// }

// void PathFollowing::avoidOrNo(const geometry_msgs::msg::Twist &message) {
//     RCLCPP_INFO(this->get_logger(), "avoid? %b", avoid);
//     int costThreshold = 60;
//     double resolution = costmap.info.resolution; // Resolution of the grid
//     int width = costmap.info.width;
//     int gridOriginX = costmap.info.origin.position.x; // X coordinate of the grid origin
//     int gridOriginY = costmap.info.origin.position.y; // Y coordinate of the grid origin

//     int robotXIndex = static_cast<int>((robotPos.x - gridOriginX) / resolution);
//     int robotYIndex = static_cast<int>((robotPos.y - gridOriginY) / resolution);

//     int robotGridPos = robotYIndex * width + robotXIndex;

//     int robotCellCost = costmap.data[robotGridPos];

//     if (robotCellCost > costThreshold) {
//         avoid = true;
//         avoidMessage = message;
//     } else {
//         avoid = false;
//     }
// }

// void PathFollowing::odomCallback(const nav_msgs::msg::Odometry &odom){
//     robotPos = odom.pose.pose.position;

//     RCLCPP_INFO(this->get_logger(), "1");

//     // Pass it to the existing path-following logic
//     robotPoseCallback(odom.pose.pose);
//     RCLCPP_INFO(this->get_logger(), "2");
//     // Keep to max 10 poses in followed path, only if robot is moving
//     if (prevRobotPos && (robotPos - *prevRobotPos).norm() > 0.0001) {
//         RCLCPP_INFO(this->get_logger(), "len of path %zu", followedPathTotal.poses.size());
//         if (followedPathTotal.poses.size() >= 6) {
//             constructFollowedPath();
//             RCLCPP_INFO(this->get_logger(), "len of path %zu", followedPathFiltered.poses.size());
//             followedPathTotal.poses.clear();
//         }

//         geometry_msgs::msg::PoseStamped poseStamped;
//         poseStamped.header = odom.header;  // Keep the same timestamp and frame
//         poseStamped.header.stamp = get_clock()->now();
//         poseStamped.pose = odom.pose.pose;  // Extract the pose
//         followedPathTotal.poses.push_back(poseStamped);
//         followedPathTotal.header.stamp = poseStamped.header.stamp;
//         followedPathTotal.header.frame_id = odom.header.frame_id;

//     }
//     RCLCPP_INFO(this->get_logger(), "3");

//     prevRobotPos = robotPos;
// }


// void PathFollowing::constructFollowedPath(){
//     followedPathFiltered.header = followedPathTotal.header;
//     followedPathFiltered.poses.push_back(followedPathTotal.poses[5]);
//     pubRobotPath->publish(followedPathFiltered);
// }


// void PathFollowing::smoothPath(const nav_msgs::msg::Path &path){

//     // if (!pathReceived) {
//     // RCLCPP_INFO(this->get_logger(), "smoothing path");

//     nav_msgs::msg::Path processedPath = processPath(path);

//     smoothedPath = processedPath.poses;

//     pubSmoothPath->publish(processedPath);
//     // pathReceived = true;
//     // }
// }


// double PathFollowing::euclDistance(const geometry_msgs::msg::Pose robotPose, const geometry_msgs::msg::PoseStamped pathPose){
//     auto robotPosition = robotPose.position;
//     auto pathPosition = pathPose.pose.position;

//     return sqrt(pow(robotPosition.x - pathPosition.x, 2) + pow(robotPosition.y - pathPosition.y, 2));
// }


// double PathFollowing::findDesiredAngle(const geometry_msgs::msg::Pose &robotPose) {
//     geometry_msgs::msg::PoseStamped nearestT;
//     double minDist = std::numeric_limits<double>::max();
//     int minIndex;

//     for (size_t i = 0; i < smoothedPath.size() - 5; i++) {

//         geometry_msgs::msg::PoseStamped pathPose = smoothedPath[i];
//         double dist = euclDistance(robotPose, pathPose);
//         if (dist < minDist){
//             minDist = dist;
//             minIndex = i;
//         }
//     }


//     Vec2f p1 = smoothedPath[minIndex].pose.position;
//     Vec2f p2 = smoothedPath[minIndex + 1].pose.position;
//     Vec2f p5 = smoothedPath[minIndex + 5].pose.position;

//     Vec2f Xt = (p2 - p1).normalized();

//     Vec2f Xn = p1 - Vec2f(robotPose.position);

//     double xn = perpDotProduct(Xn, Xt);

//     double k = 3.0;

//     double phiC = std::atan(- k * xn);

//     pubPath1->publish(smoothedPath[minIndex]);
//     pubPath2->publish(smoothedPath[minIndex + 1]);

//     return phiC + (p5 - p2).angle(); //Xt.angle();
// }


// void PathFollowing::sendControlCommand(double speed, double omega, double error) {
//     geometry_msgs::msg::Twist cmdVelMsg;
//     // RCLCPP_INFO(this->get_logger(), "error: %.2f", error);
//     RCLCPP_INFO(this->get_logger(), "following path");
//     cmdVelMsg.linear.x = speed * std::max(1 - std::abs(error), 0.0) / M_PI;
//     cmdVelMsg.angular.z = omega;  // Use PID output for angular velocity

//     pubCmdVel->publish(cmdVelMsg);
// }


// void PathFollowing::robotPoseCallback(const geometry_msgs::msg::Pose &robotPose){
//     write_plot_data::PlotDataWriter dataWriter = write_plot_data::PlotDataWriter("/home/praktikum7/prmr_group7_ws/src/path_following/src/irl_data.txt");
//     RCLCPP_INFO(this->get_logger(), "1.0");
//     maxSpeed = 1.25;
//     if (!smoothedPath.empty()) {


//         double desiredAngle = findDesiredAngle(robotPose);
//         RCLCPP_INFO(this->get_logger(), "1.1");

//         double robotYaw = tf2::getYaw(robotPose.orientation);
//         RCLCPP_INFO(this->get_logger(), "1.2");

//         // Compute rotational velocity using PID
//         static rclcpp::Time last_time = this->now();
//         rclcpp::Time current_time = this->now();
//         double dt = (current_time - last_time).seconds();
//         last_time = current_time;

//         pid.setKp(get_parameter("Kp").as_double());
//         pid.setKi(get_parameter("Ki").as_double());
//         pid.setKd(get_parameter("Kd").as_double());

//         // RCLCPP_INFO(this->get_logger(), "Kp %.2f, Ki %.2f, Kd %.2f", pid.getKp(), pid.getKi(), pid.getKd());

//         double error = std::remainder(desiredAngle - robotYaw, 2 * std::numbers::pi);
//         double omega = pid.compute(error, dt);

//         omega = std::clamp(omega, -M_PI / 6, M_PI / 6);
//         RCLCPP_INFO(this->get_logger(), "1.3");


//         // RCLCPP_INFO(this->get_logger(), "desired angle: %.4f rad (%.2f°), omega: %.4f",
//         //             desiredAngle, desiredAngle * (180.0 / M_PI), omega);

//         // slow robot if close to goal, stop if reached goal

//         if (avoid) {
//             RCLCPP_INFO(this->get_logger(), "avoiding obstacle");
//             pubCmdVel->publish(avoidMessage);
//         } else {
//             if (euclDistance(robotPose, smoothedPath.back()) < 1.0) {
//                 sendControlCommand(maxSpeed * 0.3, omega, error);
//                 if (euclDistance(robotPose, smoothedPath.back()) < 0.1) {
//                     sendControlCommand(0.0, 0.0, 0.0);
//                     RCLCPP_INFO(this->get_logger(), "END OF PATH REACHED");
//                     rclcpp::shutdown();
//                 }
//             } else {
//                 sendControlCommand(maxSpeed, omega, error);  // Send computed velocity to the robot
//             }
//         }
//         RCLCPP_INFO(this->get_logger(), "1.4");

//         // RCLCPP_INFO(this->get_logger(), "robot yaw: %.2f", robotYaw);
//         // RCLCPP_INFO(this->get_logger(), "desired yaw: %.2f", desiredAngle);

//         dataWriter.write(robotYaw, desiredAngle, desiredAngle - robotYaw);
//         RCLCPP_INFO(this->get_logger(), "1.5");
//     }

// }



