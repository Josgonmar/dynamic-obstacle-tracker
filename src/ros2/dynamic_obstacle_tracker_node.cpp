#include "dynamic_obstacle_tracker/ros2/dynamic_obstacle_tracker_node.hpp"

#include <pcl_conversions/pcl_conversions.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <exception>
#include <functional>
#include <memory>

#include "dynamic_obstacle_tracker/common/config_loader.hpp"

namespace dynamic_obstacle_tracker {

DynamicObstacleTrackerNode::DynamicObstacleTrackerNode() : Node("dynamic_obstacle_tracker_node")
{
    const std::string configured_path = declare_parameter<std::string>("config_file", "");
    input_cloud_topic_                = declare_parameter<std::string>("input_cloud_topic", "dynamic_cloud");

    std::string                  config_path = configured_path;
    DynamicObstacleTrackerConfig config;
    try {
        if (config_path.empty()) {
                config_path = ament_index_cpp::get_package_share_directory("dynamic_obstacle_tracker")
                            + "/config/default.yaml";
        }
        config = ConfigLoader(config_path).load();
    } catch (const std::exception& exception) {
        RCLCPP_FATAL(get_logger(), "Could not load tracker config '%s': %s", config_path.c_str(), exception.what());
        throw;
    }

    predicted_obstacle_topic_ = config.predicted_obstacle_topic;
    bbox_marker_topic_        = config.bbox_marker_topic;
    prediction_marker_topic_  = config.prediction_marker_topic;
    output_frame_             = config.output_frame;
    tracker_                  = std::make_unique<ObstacleTracker>(config.tracker_params, get_logger());

    RCLCPP_INFO(get_logger(), "Loaded tracker config from '%s'", config_path.c_str());

    predicted_obstacle_pub_ = create_publisher<msg::DynamicObstacleTrajectory>(
            predicted_obstacle_topic_, rclcpp::QoS(5).reliable());
    bbox_marker_pub_
            = create_publisher<visualization_msgs::msg::MarkerArray>(bbox_marker_topic_, rclcpp::SensorDataQoS());
    prediction_marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
            prediction_marker_topic_, rclcpp::SensorDataQoS());

    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            input_cloud_topic_,
            rclcpp::SensorDataQoS(),
            std::bind(&DynamicObstacleTrackerNode::cloudCallback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "Listening for dynamic points on '%s'", input_cloud_topic_.c_str());
}

void DynamicObstacleTrackerNode::cloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg)
{
    auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    try {
        pcl::fromROSMsg(*msg, *cloud);
    } catch (const std::exception& exception) {
        RCLCPP_WARN(get_logger(), "Could not convert input point cloud: %s", exception.what());
        return;
    }

    const std::string frame_id = output_frame_.empty() ? msg->header.frame_id : output_frame_;
    if (frame_id.empty()) {
        RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 5000, "Input point cloud has no frame_id and output_frame is empty");
        return;
    }

    tracker_->setFrameId(frame_id);
    const auto result = tracker_->update(cloud, now().seconds());

    rclcpp::Time stamp(msg->header.stamp);
    if (stamp.nanoseconds() == 0)
        stamp = now();

    for (auto trajectory : result.trajectories) {
        trajectory.header.stamp = stamp;
        predicted_obstacle_pub_->publish(trajectory);
    }

    if (!result.bbox_markers.markers.empty()) {
        auto markers = result.bbox_markers;
        for (auto& marker : markers.markers) marker.header.stamp = stamp;
        bbox_marker_pub_->publish(markers);
    }

    if (!result.prediction_markers.markers.empty()) {
        auto markers = result.prediction_markers;
        for (auto& marker : markers.markers) marker.header.stamp = stamp;
        prediction_marker_pub_->publish(markers);
    }
}

} // namespace dynamic_obstacle_tracker
