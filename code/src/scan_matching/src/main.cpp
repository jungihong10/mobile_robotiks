#include <rclcpp/rclcpp.hpp>
#include <memory>
#include "scan_matcher.h"

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    std::shared_ptr<ScanMatcher> node = std::make_shared<ScanMatcher>();

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}

// SteppableTask ScanMatcher::scanMatchTask(const std::vector<Vec2f> refPoints, const std::vector<Vec2f> newPoints, const Vec2f origin) {
//     RCLCPP_INFO(get_logger(), "Starting matching");
//     Transform transform;
//     double Bw_0 = M_PI / 6;
//     double Bw;
//     double distanceThreshold;
//     double translationThreshold = .05;
//     double angleThreshold = .01;
//     double old_w = 0.0;
//     double old_t = 0.0;
//     double w_difference = 0.0;
//     double t_difference = 0.0;
//     double mean_t = 10.0;
//     double mean_w = 10.0;
//     std::deque<double> old_ts(1, 0.0);
//     std::deque<double> old_ws(1, 0.0);

//     co_await std::suspend_always();

//     for (int i = 0; i < 100; i++) {
//         RCLCPP_INFO(get_logger(), "Iteration %d", i);
//         Bw = std::max(Bw_0 * exp(-.05*i), .05);
//         distanceThreshold = std::max(.5 * exp(-.1*i), .05);

//         // Run ICP and MRP
//         std::vector<Match> matchesClosest = findMatchesClosest(refPoints, newPoints, transform, distanceThreshold);
//         std::vector<Match> matchesMRP = findMatchesMRP(refPoints, newPoints, origin, Bw, transform);
//         publishVisualization(refPoints, newPoints, matchesClosest, transform);


//         Transform transform_1 = findTransformationLSQ(matchesClosest);
//         Transform transform_2 = findTransformationLSQ(matchesMRP);

//         // Combine translation from ICP and rotation from MRP into one transformation
//         transform = Transform(transform_1.t, transform_2.w);
//         w_difference = abs(transform.w) - abs(old_w);
//         t_difference =  transform_1.t.norm() - old_t;
//         old_ws.push_back(w_difference);
//         old_ts.push_back(t_difference);

//         if(old_ts.size() > 5) {
//             old_ws.pop_front();
//             old_ts.pop_front();
//             mean_t = static_cast<double>(std::accumulate(old_ts.begin(), old_ts.end(), 0.0) / old_ts.size());
//             mean_w = static_cast<double>(std::accumulate(old_ws.begin(), old_ws.end(), 0.0) / old_ws.size());
//             for(int i = 0; i < (int) old_ts.size(); i++) {
//                 RCLCPP_INFO(get_logger(), "%d-tes t: %f", i, old_ts[i]);
//                 RCLCPP_INFO(get_logger(), "%d-tes w: %f", i, old_ws[i]);
//             }
//         }
//         old_t = transform.t.norm();
//         old_w = transform.w;

//         RCLCPP_INFO(get_logger(), "mean t: %f", mean_t);
//         RCLCPP_INFO(get_logger(), "mean w: %f", mean_w);
//         // take difference to last one instead
//         if (mean_w < angleThreshold && mean_t < translationThreshold) {
//             RCLCPP_INFO(get_logger(), "Converged in %d iterations", i);
//             break;
//         }

//         co_await std::suspend_always();
//     }
// }
