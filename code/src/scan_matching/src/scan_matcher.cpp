#include "scan_matcher.h"
#include <tf2/utils.h>
#include <tf2/convert.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <color_names/color_names.hpp>
#include <chrono>
#include <algorithm>
#include <ranges>
#include <cmath>
#include <numbers>

using namespace std::chrono_literals;

ScanMatcher::ScanMatcher(): Node("scan_matcher") {
    subJoy = create_subscription<sensor_msgs::msg::Joy>("/joy", 10, std::bind(&ScanMatcher::joyCallback, this, std::placeholders::_1));
    pubVisualization = create_publisher<visualization_msgs::msg::MarkerArray>("/matches", 1);

    tf2Buffer = std::make_shared<tf2_ros::Buffer>(get_clock(), 3s);
    std::shared_ptr<tf2_ros::CreateTimerROS> timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(get_node_base_interface(), get_node_timers_interface());
    tf2Buffer->setCreateTimerInterface(timer_interface);
    tf2Listener = std::make_shared<tf2_ros::TransformListener>(*tf2Buffer);

    subLaser.subscribe(this, "/scan");
    tf2MessageFilter = std::make_shared<tf2_ros::MessageFilter<sensor_msgs::msg::LaserScan>>(subLaser, *tf2Buffer, "odom", 3, get_node_logging_interface(), get_node_clock_interface(), 500ms);
    tf2MessageFilter->setTolerance(100ms);
    tf2MessageFilter->registerCallback(&ScanMatcher::laserCallback, this);
}

void ScanMatcher::joyCallback(const sensor_msgs::msg::Joy &joy) {
    std::lock_guard<std::mutex> guard(mutex);

    enum class Button {
        CROSS      = 0,
        CIRCLE     = 1,
        TRIANGLE   = 2,
        SQUARE     = 3,

        TRIGGER_L1 = 4,
        TRIGGER_R1 = 5,
        TRIGGER_L2 = 6,
        TRIGGER_R2 = 7,

        SELECT     = 8,
        START      = 9,
        PS         = 10,

        ANALOG_L   = 11,
        ANALOG_R   = 12,

        UP         = 13,
        DOWN       = 14,
        LEFT       = 15,
        RIGHT      = 16
    };

    if (prevJoy) {
        auto justPressed = [&prevJoy = *this->prevJoy, &joy] (Button b) {
            return prevJoy.buttons[(int)b] == 0 && joy.buttons[(int)b] == 1;
        };

        if (justPressed(Button::SQUARE)) {
            if (!lastScan) {
                RCLCPP_INFO(get_logger(), "Cannot save reference scan, no scan available");
            } else if (newScan) {
                RCLCPP_INFO(get_logger(), "Cannot save reference scan, delete new scan first");
            } else {
                RCLCPP_INFO(get_logger(), "Saving reference scan");

                referenceScan = lastScan;
                lastScan.reset();
                task.reset();
            }
        }

        if (justPressed(Button::TRIANGLE)) {
            if (!lastScan) {
                RCLCPP_INFO(get_logger(), "Cannot save reference scan, no scan available");
            } else if (!referenceScan) {
                RCLCPP_INFO(get_logger(), "Cannot save new scan, save reference scan first");
            } else {
                RCLCPP_INFO(get_logger(), "Saving new scan");

                newScan = lastScan;
                lastScan.reset();
                task.reset();
            }
        }

        if (justPressed(Button::CROSS)) {
            RCLCPP_INFO(get_logger(), "Reset");
            referenceScan.reset();
            newScan.reset();
            task.reset();
        }

        if (justPressed(Button::CIRCLE)) {
            if (!task && referenceScan && newScan) {
                RCLCPP_INFO(get_logger(), "Launching match task");

                std::vector<Vec2f> refPoints = transformLaserScan(*referenceScan);

                // deliberately use an outdated transformation for the new scan to test the scan matching
                ScanWithTransform newScanWithStaleTransform = {newScan->scan, referenceScan->startTransform, referenceScan->endTransform};
                std::vector<Vec2f> newPoints = transformLaserScan(newScanWithStaleTransform);

                tf2::Transform midTransform = slerpTransforms(referenceScan->startTransform, referenceScan->endTransform, 0.5);
                Vec2f origin = midTransform(Vec2f(0, 0).toTf2Vector3());

                task = scanMatchTask(refPoints, newPoints, origin);
            }

            if (task) {
                task->step();
            }
        }
    }

    prevJoy = joy;
}

void ScanMatcher::laserCallback(const sensor_msgs::msg::LaserScan &scan) {
    std::lock_guard<std::mutex> guard(mutex);

    tf2::Transform startTransform;
    tf2::Transform endTransform;
    tf2::convert(tf2Buffer->lookupTransform("odom", scan.header.frame_id, scan.header.stamp).transform, startTransform);
    tf2::convert(tf2Buffer->lookupTransform("odom", scan.header.frame_id, scan.header.stamp + rclcpp::Duration::from_seconds((scan.ranges.size() - 1) * scan.time_increment)).transform, endTransform);

    lastScan = {scan, startTransform, endTransform};
}

// SteppableTask ScanMatcher::scanMatchTask(const std::vector<Vec2f> refPoints, const std::vector<Vec2f> newPoints, const Vec2f origin) {
//     RCLCPP_INFO(get_logger(), "Starting matching");
//     Transform transform;
//     double Bw_0 = M_PI / 6;
//     double Bw;
//     double distanceThreshold;
//     double translationThreshold = .001;
//     double angleThreshold = .001;
//     double old_w = 0.0;
//     double old_t = 0.0;
//     double w_difference = 0.0;
//     double t_difference = 0.0;

//     co_await std::suspend_always();

//     for (int i = 0; i < 100; i++) {
//         RCLCPP_INFO(get_logger(), "Iteration %d", i);
//         Bw = std::max(Bw_0 * exp(-.05*i), .05);
//         distanceThreshold = std::max(.5 * exp(-.1*i), .02);

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

//         // take difference to last one instead
//         if (w_difference < angleThreshold && t_difference < translationThreshold) {
//             RCLCPP_INFO(get_logger(), "Converged in %d iterations", i);
//             break;
//         }
//         old_t = transform.t.norm();
//         old_w = transform.w;

//         co_await std::suspend_always();
//     }
// }


SteppableTask ScanMatcher::scanMatchTask(const std::vector<Vec2f> refPoints, const std::vector<Vec2f> newPoints, const Vec2f origin) {
    RCLCPP_INFO(get_logger(), "Starting matching");
    Transform transform;
    double Bw_0 = M_PI / 6;
    double Bw;
    double distanceThreshold;
    double translationThreshold = .0001;
    double angleThreshold = .0001;
    double old_w = 0.0;
    double old_t = 0.0;
    double w_difference = 0.0;
    double t_difference = 0.0;
    double mean_t = 10.0;
    double mean_w = 10.0;
    std::deque<double> old_ts(1, 0.0);
    std::deque<double> old_ws(1, 0.0);

    co_await std::suspend_always();

    for (int i = 0; i < 100; i++) {
        RCLCPP_INFO(get_logger(), "Iteration %d", i);
        Bw = std::max(Bw_0 * exp(-.05*i), .05);
        distanceThreshold = std::max(.5 * exp(-.1*i), .05);

        // Run ICP and MRP
        std::vector<Match> matchesClosest = findMatchesClosest(refPoints, newPoints, transform, distanceThreshold);
        std::vector<Match> matchesMRP = findMatchesMRP(refPoints, newPoints, origin, Bw, transform);
        publishVisualization(refPoints, newPoints, matchesClosest, transform);


        Transform transform_1 = findTransformationLSQ(matchesClosest);
        Transform transform_2 = findTransformationLSQ(matchesMRP);

        // Combine translation from ICP and rotation from MRP into one transformation
        transform = Transform(transform_1.t, transform_2.w);
        w_difference = abs(transform.w) - abs(old_w);
        t_difference =  transform_1.t.norm() - old_t;
        old_ws.push_back(w_difference);
        old_ts.push_back(t_difference);

        if(old_ts.size() > 5) {
            old_ws.pop_front();
            old_ts.pop_front();
            mean_t = static_cast<double>(std::accumulate(old_ts.begin(), old_ts.end(), 0.0) / old_ts.size());
            mean_w = static_cast<double>(std::accumulate(old_ws.begin(), old_ws.end(), 0.0) / old_ws.size());
            for(int i = 0; i < (int) old_ts.size(); i++) {
                RCLCPP_INFO(get_logger(), "%d-tes t: %f", i, old_ts[i]);
                RCLCPP_INFO(get_logger(), "%d-tes w: %f", i, old_ws[i]);
            }
        }
        old_t = transform.t.norm();
        old_w = transform.w;

        RCLCPP_INFO(get_logger(), "mean t: %f", mean_t);
        RCLCPP_INFO(get_logger(), "mean w: %f", mean_w);
        // take difference to last one instead
        if (mean_w < angleThreshold && mean_t < translationThreshold && Bw < .075) {
            RCLCPP_INFO(get_logger(), "Converged in %d iterations", i);
            break;
        }

        co_await std::suspend_always();
    }
}


std::vector<ScanMatcher::Match> ScanMatcher::findMatchesClosest(const std::vector<Vec2f> &refPoints, const std::vector<Vec2f> &newPoints, const Transform &transform, double distanceThreshold) {
    std::vector<ScanMatcher::Match> matches;

    for (const Vec2f &newPoint : newPoints) {

        Vec2f transformedPoint = newPoint.rotated(transform.w) + transform.t;

        float minDistance = std::numeric_limits<float>::max();
        const Vec2f *closestRefPoint = nullptr;

        for (const Vec2f &refPoint : refPoints) {
            float eucl_distance = sqrt(pow((refPoint.x - transformedPoint.x), 2) + pow((refPoint.y - transformedPoint.y), 2));

            if (eucl_distance < minDistance){
                minDistance = eucl_distance;
                closestRefPoint = &refPoint;
            }
        }
        if (closestRefPoint == nullptr || minDistance > distanceThreshold){

            continue;}

        matches.push_back({newPoint, *closestRefPoint});
    }

    return matches;
}



/*
 * Match points of this scan to points of the reference scan according to the MRP rule given the rotation limit Bw.
 *
 * transform is the current estimation of the transformation from the new scan to the reference scan,
 * origin is the position of the laser scanner in reference scan.
 *
 * Return a list of all match pairs.
 */
std::vector<ScanMatcher::Match> ScanMatcher::findMatchesMRP(
    const std::vector<Vec2f> &refPoints,
    const std::vector<Vec2f> &newPoints,
    Vec2f origin,
    double Bw,
    const Transform &transform)
{
    std::vector<ScanMatcher::Match> matches;

    for (const Vec2f &newPoint : newPoints) {
        // Transform the new point using the current transform
        Vec2f transformedPoint = newPoint.rotated(transform.w) + transform.t;

        // Find the closest reference point using the MRP rule
        double minLengthDifference = std::numeric_limits<double>::max();
        const Vec2f *bestMatch = nullptr;
        for (const Vec2f &refPoint : refPoints) {
            // Compute vectors from the origin
            Vec2f vectorNew = transformedPoint - origin;
            Vec2f vectorRef = refPoint - origin;

            double lengthDifference = std::abs(vectorNew.norm() - vectorRef.norm());

            // Calculate the angular difference
            double angleNew = std::atan2(vectorNew.y, vectorNew.x);
            double angleRef = std::atan2(vectorRef.y, vectorRef.x);
            double angularDifference = std::abs(std::remainder(angleRef - angleNew, 2 * std::numbers::pi));

            // Check if the angular difference is within the limit
            if (angularDifference < Bw) {
                if (lengthDifference < minLengthDifference && lengthDifference < .15) {
                    minLengthDifference = lengthDifference;
                    bestMatch = &refPoint;
                }
            }
        }

        if (bestMatch == nullptr ){

            continue;}

        // If a match is found, add it to the list
        matches.push_back({newPoint, *bestMatch});

    }

    return matches;
}


/*
 * Find the translation and rotation that minimize the total distance of matches in a least squares sense.
 */

ScanMatcher::Transform ScanMatcher::findTransformationLSQ(const std::vector<Match> &matches) {
    if (matches.empty()) {
        return Transform(); // Return identity transform if no matches exist.
    }

    // Step 1: Calculate centroids of the point sets.
    Vec2f centroidRef(0, 0);
    Vec2f centroidNew(0, 0);

    for (const auto &match : matches) {
        centroidRef += match.pRef;
        centroidNew += match.pNew;
    }

    centroidRef /= static_cast<float>(matches.size());
    centroidNew /= static_cast<float>(matches.size());

    // Step 2: Center the points around their centroids.
    std::vector<Vec2f> centeredRef;
    std::vector<Vec2f> centeredNew;

    for (const auto &match : matches) {
        centeredRef.push_back(match.pRef - centroidRef);
        centeredNew.push_back(match.pNew - centroidNew);
    }

    // Step 3: Compute the cross-covariance matrix.
    float Sxx = 0, Sxy = 0, Syx = 0, Syy = 0;

    for (size_t i = 0; i < matches.size(); ++i) {
        Sxx += centeredNew[i].x * centeredRef[i].x;
        Sxy += centeredNew[i].x * centeredRef[i].y;
        Syx += centeredNew[i].y * centeredRef[i].x;
        Syy += centeredNew[i].y * centeredRef[i].y;
    }

    // Step 4: Compute the rotation angle using atan2.
    float rotationAngle = atan2(Sxy - Syx, Sxx + Syy);

    // Step 5: Compute the translation.
    Vec2f translation = centroidRef - centroidNew.rotated(rotationAngle);

    // Return the computed transformation.
    return Transform(translation, rotationAngle);
}

void ScanMatcher::publishVisualization(const std::vector<Vec2f> &refPoints, const std::vector<Vec2f> &newPoints, const std::vector<Match> &matches, const Transform &transform) {
    visualization_msgs::msg::MarkerArray markerArray;

    visualization_msgs::msg::Marker deleteAllMarker;
    //deleteAllMarker.ns = "delete_all";
    deleteAllMarker.action = visualization_msgs::msg::Marker::DELETEALL;
    markerArray.markers.push_back(deleteAllMarker);

    markerArray.markers.push_back(pointsToMarker(refPoints, color_names::makeColorMsg("lime"), "reference_points"));

    markerArray.markers.push_back(pointsToMarker(newPoints, color_names::makeColorMsg("red"), "new_points", transform));

    std::vector<visualization_msgs::msg::Marker> arrows = matchesToArrows(matches, transform);
    markerArray.markers.insert(markerArray.markers.end(), arrows.begin(), arrows.end());

    pubVisualization->publish(markerArray);
}

visualization_msgs::msg::Marker ScanMatcher::pointsToMarker(const std::vector<Vec2f> &points, const std_msgs::msg::ColorRGBA color, std::string ns, const Transform &transform) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id  = "odom";
    marker.type = visualization_msgs::msg::Marker::POINTS;
    marker.color = color;
    marker.ns = ns;
    marker.scale.x = 0.01;
    marker.scale.y = 0.01;
    marker.scale.z = 0.01;

    for (Vec2f p: points) {
        marker.points.push_back((p.rotated(transform.w) + transform.t).toGeometryMsgPoint());
    }

    return marker;
}

std::vector<visualization_msgs::msg::Marker> ScanMatcher::matchesToArrows(const std::vector<Match> &matches, const Transform &transform) {
    std::vector<visualization_msgs::msg::Marker> markers;

    visualization_msgs::msg::Marker marker;
    marker.header.frame_id  = "odom";
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.color = color_names::makeColorMsg("blue");
    marker.ns = "matches";
    marker.scale.x = 0.005;
    marker.scale.y = 0.01;
    marker.scale.z = 0.01;

    for (const Match &match: matches) {
        marker.points = {(match.pNew.rotated(transform.w) + transform.t).toGeometryMsgPoint(), match.pRef.toGeometryMsgPoint()};
        markers.push_back(marker);

        marker.id++;
    }

    return markers;
}

std::vector<Vec2f> ScanMatcher::transformLaserScan(const ScanWithTransform &scanWithTransform) {
    const sensor_msgs::msg::LaserScan &scan = scanWithTransform.scan;

    std::vector<Vec2f> laserPoints;
    for (unsigned int i = 0; i < scan.ranges.size(); i++) {
        double angle = scan.angle_min + scan.angle_increment * i;

        if (std::abs(angle) > 1.6) {
            continue;
        }

        if (std::isnan(scan.ranges[i]) || std::isinf(scan.ranges[i]) || scan.ranges[i] < scan.range_min || scan.ranges[i] > scan.range_max) {
            continue;
        }

        Vec2f pLaser = scan.ranges[i] * Vec2f::fromAngle(scan.angle_min + scan.angle_increment * i);

        tf2::Transform interpolatedTransform = slerpTransforms(scanWithTransform.startTransform, scanWithTransform.endTransform, i / (scan.ranges.size() - 1.0));
        Vec2f pOdom = interpolatedTransform(pLaser.toTf2Vector3());

        laserPoints.push_back(pOdom);
    }

    return laserPoints;
}

// spherical linear interpolation, https://en.wikipedia.org/wiki/Slerp
tf2::Transform ScanMatcher::slerpTransforms(const tf2::Transform &a, const tf2::Transform &b, double ratio) {
    tf2::Transform slerpedTransform;
    tf2::Vector3 translation;
    translation.setInterpolate3(a.getOrigin(), b.getOrigin(), ratio);
    slerpedTransform.setOrigin(translation);
    slerpedTransform.setRotation(tf2::slerp(a.getRotation(), b.getRotation(), ratio));

    return slerpedTransform;
}
