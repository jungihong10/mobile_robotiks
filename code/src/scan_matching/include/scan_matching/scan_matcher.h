#ifndef LASER_TRANSFORMER_H
#define LASER_TRANSFORMER_H

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>
#include <message_filters/subscriber.h>

#include <memory>
#include <mutex>
#include <optional>

#include "vec2.h"
#include "steppable_task.h"

class ScanMatcher: public rclcpp::Node {
    public:
        ScanMatcher();

    private:
        struct ScanWithTransform {
            sensor_msgs::msg::LaserScan scan;
            tf2::Transform startTransform;
            tf2::Transform endTransform;
        };

        struct Match {
            Vec2f pNew;
            Vec2f pRef;
        };

        struct Transform {
            Transform(): w(0) {};
            Transform(Vec2f t, double w): t(t), w(w) {};

            Vec2f t;
            double w;
        };

        void joyCallback(const sensor_msgs::msg::Joy &joy);
        void laserCallback(const sensor_msgs::msg::LaserScan &scan);
        SteppableTask scanMatchTask(const std::vector<Vec2f> refPoints, const std::vector<Vec2f> newPoints, const Vec2f origin);
        std::vector<ScanMatcher::Match> findMatchesClosest(const std::vector<Vec2f> &refPoints, const std::vector<Vec2f> &newPoints, const Transform &transform, double distanceThreshold);
        std::vector<ScanMatcher::Match> findMatchesMRP(const std::vector<Vec2f> &refPoints, const std::vector<Vec2f> &newPoints, Vec2f origin, double Bw, const Transform &transform);
        ScanMatcher::Transform findTransformationLSQ(const std::vector<Match> &matches);
        void findTransformationLSQ(const std::vector<Match> &matches, Vec2f &t, double &w);
        void publishVisualization(const std::vector<Vec2f> &refPoints, const std::vector<Vec2f> &newPoints, const std::vector<Match> &matches, const Transform &transform);
        visualization_msgs::msg::Marker pointsToMarker(const std::vector<Vec2f> &points, const std_msgs::msg::ColorRGBA color, std::string ns, const Transform &transform = Transform());
        std::vector<visualization_msgs::msg::Marker> matchesToArrows(const std::vector<Match> &matches, const Transform &transform);
        std::vector<Vec2f> transformLaserScan(const ScanWithTransform &scanWithTransform);
        tf2::Transform slerpTransforms(const tf2::Transform &a, const tf2::Transform &b, double ratio);
        ScanMatcher::Transform findTransformationIDC(const std::vector<Vec2f> &refPoints, const std::vector<Vec2f> &newPoints, Vec2f origin, double Bw, const Transform &transform);

        std::mutex mutex;

        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subJoy;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pubVisualization;

        std::shared_ptr<tf2_ros::Buffer> tf2Buffer;
        std::shared_ptr<tf2_ros::TransformListener> tf2Listener;
        message_filters::Subscriber<sensor_msgs::msg::LaserScan> subLaser;
        std::shared_ptr<tf2_ros::MessageFilter<sensor_msgs::msg::LaserScan>> tf2MessageFilter;

        std::optional<sensor_msgs::msg::Joy> prevJoy;

        std::optional<ScanWithTransform> lastScan;

        std::optional<ScanWithTransform> referenceScan;
        std::optional<ScanWithTransform> newScan;

        std::optional<SteppableTask> task;

        std::vector<Vec2f> newPoint;
};

#endif
