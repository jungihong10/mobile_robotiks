#include "ball_tracker.h"
#include <tf2/utils.h>
#include <builtin_interfaces/msg/time.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <color_names/color_names.hpp>
#include <chrono>
#include <functional>
#include <pcl_ros/transforms.hpp>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

/// To get an understanding of the PCL API, take a quick look at these websites:
///   https://pcl.readthedocs.io/projects/tutorials/en/latest/planar_segmentation.html#planar-segmentation
///   https://pcl.readthedocs.io/projects/tutorials/en/latest/extract_indices.html?highlight=extract%20indices
///   https://pcl.readthedocs.io/projects/tutorials/en/latest/random_sample_consensus.html#random-sample-consensus

using namespace std::chrono_literals;

BallTracker::BallTracker(): Node("ball_tracker"), send_goal(false) {
    declare_parameter("ransac_plane", 0); // RANSAC method (plane)
    declare_parameter("ransac_sphere", 0); // RANSAC method (sphere)

    declare_parameter("sphere_radius_min", 0.06); // sphere radius (min)
    declare_parameter("sphere_radius_max", 0.20); // sphere radius (max)

    declare_parameter("iterations_plane", 50); // RANSAC iterations (plane)
    declare_parameter("iterations_sphere", 700); // RANSAC iterations (sphere)

    declare_parameter("sphere_min_points", 50); // min no of point per sphere

    declare_parameter("sphere_distance_threshold", 0.06);
    declare_parameter("sphere_normal_distance_weight", 0.1);

    declare_parameter("floor", 1); // no of floors
    declare_parameter("walls", 3); // no of walls
    declare_parameter("spheres", 3); // no of spheres

    pubFloor = create_publisher<sensor_msgs::msg::PointCloud2>("/floor", 1);
    pubWalls = create_publisher<sensor_msgs::msg::PointCloud2>("/walls", 1);
    pubBalls = create_publisher<sensor_msgs::msg::PointCloud2>("/balls", 1);
    pubRest = create_publisher<sensor_msgs::msg::PointCloud2>("/rest", 1);
    pubGoal = create_publisher<geometry_msgs::msg::PoseStamped>("/goal", 1);

    timer = create_wall_timer(50ms, std::bind(&BallTracker::tick, this));

    tf2Buffer = std::make_shared<tf2_ros::Buffer>(get_clock());
    // Create the timer interface before call to waitForTransform, to avoid a tf2_ros::CreateTimerInterfaceException exception
    std::shared_ptr<tf2_ros::CreateTimerROS> timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(get_node_base_interface(), get_node_timers_interface());
    tf2Buffer->setCreateTimerInterface(timer_interface);
    tf2Listener = std::make_shared<tf2_ros::TransformListener>(*tf2Buffer);

    subPoints.subscribe(this, "/camera/depth_registered/points");
    tf2MessageFilter = std::make_shared<tf2_ros::MessageFilter<sensor_msgs::msg::PointCloud2>>(subPoints, *tf2Buffer, "odom", 3, get_node_logging_interface(), get_node_clock_interface(), 500ms);
    tf2MessageFilter->setTolerance(100ms);
    // Register a callback with tf2_rpublishos::MessageFilter to be called when transforms are available
    tf2MessageFilter->registerCallback(&BallTracker::pointsCallback, this);
}

void BallTracker::pointsCallback(const sensor_msgs::msg::PointCloud2 &pointCloud) {
    std::lock_guard<std::mutex> guard(mutex);

    RCLCPP_INFO(get_logger(), "Point cloud received");

    sensor_msgs::msg::PointCloud2 pointCloudOdom;
    pcl_ros::transformPointCloud("odom", pointCloud, pointCloudOdom, *tf2Buffer);

    PointCloud::Ptr pclPointCloud = std::make_shared<PointCloud>();
    *pclPointCloud = pointCloud2ToPclPointCloud(pointCloudOdom);

    // removeNans(pclPointCloud);
    std::vector<int> removed_indices;
    pcl::removeNaNFromPointCloud(*pclPointCloud, *pclPointCloud, removed_indices);

    NormalCloud::Ptr normals = std::make_shared<NormalCloud>();
    estimateNormals(pclPointCloud, normals);

    // find the ground plane and extract it
    Eigen::Vector4f plane_model;
    if (!findAndExtractFloor(pclPointCloud, normals, plane_model)) {
        RCLCPP_INFO(get_logger(), "cannot find the plane");
    }
    if (!findAndExtractWalls(pclPointCloud, normals, plane_model)) {
        RCLCPP_INFO(get_logger(), "cannot find the plane");
    }

    pubRest->publish(pclPointCloudTopointCloud2(*pclPointCloud));
    //return;

    // find all balls
    PointCloud balls;
    findAndExtractBalls(pclPointCloud, normals, plane_model, balls);

}


void BallTracker::removeNans(PointCloud::Ptr cloud){
    PointCloud::Ptr cloud_clean(new PointCloud);

    int i = 0;
    for (const PointT &point: cloud->points){
        if(!std::isnan(point.x) && !std::isnan(point.y) && !std::isnan(point.z)){
            cloud_clean ->points.push_back(point);
        }
    }
    cloud->points.swap(cloud_clean->points);
}


bool BallTracker::findAndExtractFloor(PointCloud::Ptr cloud, NormalCloud::Ptr normals, Eigen::Vector4f &plane_model) {
    // 1. find a plane
    // 2. remove all point of that plane
    // 3. publish the extracted points

    // TAKE A LOOK AT THE LINKS PROVIDED IN THE FIRST LINES OF THIS FILE!

    int no_of_planes_to_extract = get_parameter("floor").as_int();
    for (int i = 0; i < no_of_planes_to_extract; i++) {
        // find segmentation
        // Use segmenter_plane_ to segment the cloud into a plane (you need to set the input cloud and input normals)
        pcl::PointIndices::Ptr inliers (new pcl::PointIndices());
        pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients);
        pcl::SACSegmentationFromNormals<pcl::PointXYZRGB, pcl::Normal> seg;
        RCLCPP_INFO(this->get_logger(), "Input PointCloud size: %zu", cloud->size());

        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_NORMAL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(0.01);
        seg.setInputCloud(cloud);
        seg.setInputNormals(normals);
        seg.setMaxIterations(get_parameter("iterations_plane").as_int());
        seg.segment(*inliers, *coefficients_plane);


        // Set a minimum number of points to be used as a plane
        if (inliers->indices.size() <= 1000) {
            PCL_ERROR ("Could not estimate a planar model for the given dataset.\n");
            return false;
        }

        // extract all inliers and put them into 'extracted_plane'
        PointCloud::Ptr extracted_plane(new PointCloud);
        extracted_plane->header = cloud->header;

        pcl::ExtractIndices<pcl::PointXYZRGB> extract;

        extract.setInputCloud(cloud);
        extract.setIndices (inliers);
        extract.setNegative (false);
        extract.filter(*extracted_plane);

        RCLCPP_INFO(this->get_logger(), "Size of floor %d: %zu", i, extracted_plane->size());

        // remove the points from the point cloud
        PointCloud::Ptr cloud_remains (new PointCloud);
        extract.setNegative(true);
        extract.filter(*cloud_remains);
        *cloud = *cloud_remains;

        // remove the normals from the normal cloud
        pcl::ExtractIndices<pcl::Normal> extract_normals;
        NormalCloud::Ptr normal_remains (new NormalCloud);
        extract_normals.setInputCloud(normals);
        extract_normals.setIndices (inliers);
        extract_normals.setNegative (true);
        extract_normals.filter(*normal_remains);
        *normals = *normal_remains;


        // update plane model
        // plane_model[0] = coefficients_plane->values[0];
        // plane_model[1] = coefficients_plane->values[1];
        // plane_model[2] = coefficients_plane->values[2];
        // plane_model[3] = coefficients_plane->values[3];

        // publish the plane
        pubFloor->publish(pclPointCloudTopointCloud2(*extracted_plane));

        return true;
    }

    return false;
}

bool BallTracker::findAndExtractWalls(PointCloud::Ptr cloud, NormalCloud::Ptr normals, Eigen::Vector4f &plane_model) {
    // 1. find a plane
    // 2. remove all point of that plane
    // 3. publish the extracted points

    // TAKE A LOOK AT THE LINKS PROVIDED IN THE FIRST LINES OF THIS FILE!

    int no_of_planes_to_extract = get_parameter("walls").as_int();
    for (int i = 0; i < no_of_planes_to_extract; i++) {
        // find segmentation
        // Use segmenter_plane_ to segment the cloud into a plane (you need to set the input cloud and input normals)
        pcl::PointIndices::Ptr inliers (new pcl::PointIndices());
        pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients);
        pcl::SACSegmentationFromNormals<pcl::PointXYZRGB, pcl::Normal> seg;
        RCLCPP_INFO(this->get_logger(), "Input PointCloud size: %zu", cloud->size());

        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_NORMAL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(0.05);
        seg.setNormalDistanceWeight(.1);
        seg.setInputCloud(cloud);
        seg.setInputNormals(normals);
        seg.setMaxIterations(get_parameter("iterations_plane").as_int());
        seg.segment(*inliers, *coefficients_plane);


        // Set a minimum number of points to be used as a plane
        if (inliers->indices.size() <= 1000) {
            PCL_ERROR ("Could not estimate a planar model for the given dataset.\n");
            return false;
        }

        // extract all inliers and put them into 'extracted_plane'
        PointCloud::Ptr extracted_plane(new PointCloud);
        extracted_plane->header = cloud->header;

        pcl::ExtractIndices<pcl::PointXYZRGB> extract;

        extract.setInputCloud(cloud);
        extract.setIndices (inliers);
        extract.setNegative (false);
        extract.filter(*extracted_plane);

        RCLCPP_INFO(this->get_logger(), "Size of wall %d: %zu", i, extracted_plane->size());

        // remove the points from the point cloud
        PointCloud::Ptr cloud_remains (new PointCloud);
        extract.setNegative(true);
        extract.filter(*cloud_remains);
        *cloud = *cloud_remains;

        // remove the normals from the normal cloud
        pcl::ExtractIndices<pcl::Normal> extract_normals;
        NormalCloud::Ptr normal_remains (new NormalCloud);
        extract_normals.setInputCloud(normals);
        extract_normals.setIndices (inliers);
        extract_normals.setNegative (true);
        extract_normals.filter(*normal_remains);
        *normals = *normal_remains;


        // update plane model
        // plane_model[0] = coefficients_plane->values[0];
        // plane_model[1] = coefficients_plane->values[1];
        // plane_model[2] = coefficients_plane->values[2];
        // plane_model[3] = coefficients_plane->values[3];

        // publish the plane
        pubWalls->publish(pclPointCloudTopointCloud2(*extracted_plane));

        return true;
    }

    return false;
}

void BallTracker::findAndExtractBalls(PointCloud::Ptr cloud, NormalCloud::Ptr normals, const Eigen::Vector4f &plane_model, PointCloud &balls) {
    pcl::ModelCoefficients::Ptr coefficients_sphere = std::make_shared<pcl::ModelCoefficients>();

    balls.header = cloud->header;

    int no_of_spheres_to_extract = get_parameter("spheres").as_int();
    for (int i = 0; i < no_of_spheres_to_extract; i++) {
        // find segmentation

        pcl::PointIndices::Ptr inliers (new pcl::PointIndices());
        pcl::ModelCoefficients::Ptr coefficients_sphere (new pcl::ModelCoefficients);
        pcl::SACSegmentationFromNormals<pcl::PointXYZRGB, pcl::Normal> seg;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_NORMAL_SPHERE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(get_parameter("sphere_distance_threshold").as_double());
        seg.setInputCloud(cloud);
        seg.setInputNormals(normals);
        seg.setMaxIterations(get_parameter("iterations_sphere").as_int());
        seg.setRadiusLimits(get_parameter("sphere_radius_min").as_double(), get_parameter("sphere_radius_max").as_double());
        seg.segment(*inliers, *coefficients_sphere);

        if (inliers->indices.size() <= 50) {
            PCL_ERROR ("No more spheres found.\n");
            continue;
        }

        // calculate sphere position
        pcl::PointXYZ sphere_center;//(coefficients_sphere)
        sphere_center.x = coefficients_sphere->values[0];
        sphere_center.y = coefficients_sphere->values[1];
        sphere_center.z = coefficients_sphere->values[2];

        // update the sphere model
        // bonus: use plane model to limit ball positions to the ground
        // TODO...

        // extract all inliers
        PointCloud::Ptr extracted_sphere_points(new PointCloud);
        pcl::ExtractIndices<pcl::PointXYZRGB> extract;
        extract.setInputCloud(cloud);
        extract.setIndices (inliers);
        extract.setNegative (false);
        extract.filter(*extracted_sphere_points);

        if (extracted_sphere_points->size() < 500){
            RCLCPP_INFO(this->get_logger(), "No balls!");
            break;
        }

        RCLCPP_INFO(this->get_logger(), "Found ball %d of size: %zu", i, extracted_sphere_points->size());

        // analyze the ball
        extractBall(sphere_center, *extracted_sphere_points, balls);
        // remove the points from the point cloud
        PointCloud::Ptr cloud_remains (new PointCloud);
        extract.setNegative(true);
        extract.filter(*cloud_remains);
        *cloud = *cloud_remains;

        // remove the normals from the normal cloud
        pcl::ExtractIndices<pcl::Normal> extract_normals;
        NormalCloud::Ptr normal_remains (new NormalCloud);
        extract_normals.setInputCloud(normals);
        extract_normals.setIndices (inliers);
        extract_normals.setNegative (true);
        extract_normals.filter(*normal_remains);
        *normals = *normal_remains;
    }

    // publish the spheres
    pubBalls->publish(pclPointCloudTopointCloud2(balls));

    processBalls(balls);
}

void BallTracker::extractBall(pcl::PointXYZ center, PointCloud &points, PointCloud &balls) {
    // iterate all points in 'points' and calculate the mean color
    PointT ball;
    // ball has fields: ['x', 'y', 'z', 'r', 'g', 'b']

    ball.x = center.x;
    ball.y = center.y;
    ball.z = center.z;

    float sum_r = 0.0, sum_g = 0.0, sum_b=0.0;

    size_t point_count = points.points.size();

    for(const PointT &p : points.points){
        sum_r += p.r;
        sum_g += p.g;
        sum_b += p.b;
    }

    float avg_r = (sum_r / point_count);
    float avg_g = (sum_g / point_count);
    float avg_b = (sum_b / point_count);


    ball.r = static_cast<uint8_t>(avg_r);
    ball.g = static_cast<uint8_t>(avg_g);
    ball.b = static_cast<uint8_t>(avg_b);

    balls.points.push_back(ball);

    for(PointT &p : points.points){
        p.r = static_cast<uint8_t>(avg_r);
        p.g = static_cast<uint8_t>(avg_g);
        p.b = static_cast<uint8_t>(avg_b);
        balls.points.push_back(p);
    }
}

bool BallTracker::isYellow(const PointT &ball){
    return (ball.r > 200 && ball.g < 200 && ball.g > 100 && ball.b < 100);
}


void BallTracker::processBalls(PointCloud &balls) {
    send_goal = false;
    for (const PointT &ball: balls.points) {
        if (isYellow(ball)) {
            RCLCPP_INFO(this->get_logger(), "Found yellow ball with r %d, g %d, b %d", ball.r, ball.g, ball.b);
            send_goal = true;
            last_goal_ball = ball;
            sendBallPoseAsGoalPose(last_goal_ball);
            break;
        }
    }
}

void BallTracker::tick() {
    std::lock_guard<std::mutex> guard(mutex);

    // this code runs at a higher frequency than the cloud callback (~30Hz)
    if (send_goal) {
        sendBallPoseAsGoalPose(last_goal_ball);
    }
}

void BallTracker::sendBallPoseAsGoalPose(const PointT &ball) {
    // publish the position of the ball as the goal pose
    geometry_msgs::msg::PoseStamped goal;
    goal.header.frame_id = "/odom";
    goal.header.stamp = this->now();
    goal.pose.position.x = ball.x;
    goal.pose.position.y = ball.y;
    goal.pose.position.z = ball.z;
    goal.pose.orientation.w = 1.0;
    RCLCPP_INFO(this->get_logger(), "Publishing goal at x %f, y %f, z %f", ball.x, ball.y, ball.z);


    pubGoal->publish(goal);
}

void BallTracker::estimateNormals(PointCloud::ConstPtr cloud, NormalCloud::Ptr normals) {
    pcl::NormalEstimation<PointT, pcl::Normal> normal_estimation;
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);

    normal_estimation.setSearchMethod(tree);
    normal_estimation.setInputCloud(cloud);
    normal_estimation.setKSearch(50);
    normal_estimation.compute(*normals);
}

BallTracker::PointCloud BallTracker::pointCloud2ToPclPointCloud(const sensor_msgs::msg::PointCloud2 &pointCloud2) {
    pcl::PCLPointCloud2 pclPointCloud2;
    pcl_conversions::toPCL(pointCloud2, pclPointCloud2);

    PointCloud pclPointCloud;
    pcl::fromPCLPointCloud2(pclPointCloud2, pclPointCloud);

    return pclPointCloud;
}

sensor_msgs::msg::PointCloud2 BallTracker::pclPointCloudTopointCloud2(const PointCloud &pclPointCloud) {
    pcl::PCLPointCloud2 pclPointCloud2;
    pcl::toPCLPointCloud2(pclPointCloud, pclPointCloud2);

    sensor_msgs::msg::PointCloud2 pointCloud2;
    pcl_conversions::fromPCL(pclPointCloud2, pointCloud2);

    return pointCloud2;
}
