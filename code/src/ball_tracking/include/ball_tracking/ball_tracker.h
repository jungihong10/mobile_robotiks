#ifndef BALL_TRACKER_H
#define BALL_TRACKER_H

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/bool.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <message_filters/subscriber.h>

#include <memory>
#include <mutex>

#include <pcl/pcl_base.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/sac_model_plane.h>

class BallTracker: public rclcpp::Node {
    public:
        BallTracker();

        typedef pcl::PointXYZRGB PointT;
        typedef pcl::PointCloud<PointT> PointCloud;
        typedef pcl::PointCloud<pcl::Normal> NormalCloud;

    private:
        void pointsCallback(const sensor_msgs::msg::PointCloud2 &pointCloud);
        bool findAndExtractFloor(PointCloud::Ptr cloud, NormalCloud::Ptr normals, Eigen::Vector4f &plane_model);
        bool findAndExtractWalls(PointCloud::Ptr cloud, NormalCloud::Ptr normals, Eigen::Vector4f &plane_model);
        void findAndExtractBalls(PointCloud::Ptr cloud, NormalCloud::Ptr normals, const Eigen::Vector4f &plane_model, PointCloud &balls);
        void extractBall(pcl::PointXYZ center, PointCloud &points, PointCloud &balls);
        void processBalls(PointCloud &balls);
        void removeNans(PointCloud::Ptr cloud);

        void tick();
        void sendBallPoseAsGoalPose(const PointT &ball);

        bool isYellow(const PointT &ball);

        void estimateNormals(PointCloud::ConstPtr cloud, NormalCloud::Ptr normals);
        PointCloud pointCloud2ToPclPointCloud(const sensor_msgs::msg::PointCloud2 &pointCloud2);
        sensor_msgs::msg::PointCloud2 pclPointCloudTopointCloud2(const PointCloud &pclPointCloud);

        std::mutex mutex;

        std::shared_ptr<tf2_ros::Buffer> tf2Buffer;
        std::shared_ptr<tf2_ros::TransformListener> tf2Listener;
        message_filters::Subscriber<sensor_msgs::msg::PointCloud2> subPoints;
        std::shared_ptr<tf2_ros::MessageFilter<sensor_msgs::msg::PointCloud2>> tf2MessageFilter;

        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubFloor;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubWalls;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubBalls;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubRest;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pubGoal;
        rclcpp::TimerBase::SharedPtr timer;

        bool send_goal;
        PointT last_goal_ball;
};

#endif
