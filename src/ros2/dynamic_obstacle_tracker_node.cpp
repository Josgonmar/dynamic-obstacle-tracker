#include "dynamic_obstacle_tracker/ros2/dynamic_obstacle_tracker_node.hpp"

#include <pcl_conversions/pcl_conversions.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <exception>
#include <functional>
#include <memory>
#include <rclcpp_components/register_node_macro.hpp>
#include <stdexcept>

#include "dynamic_obstacle_tracker/common/config_loader.hpp"

namespace dynamic_obstacle_tracker {
namespace {

std::string normalizeFrame(std::string frame)
{
    while (!frame.empty() && frame.front() == '/') frame.erase(frame.begin());
    return frame;
}

} // namespace

DynamicObstacleTrackerNode::DynamicObstacleTrackerNode(const rclcpp::NodeOptions& options) :
        Node("dynamic_obstacle_tracker_node", options)
{
    const std::string configured_path = declare_parameter<std::string>("config_file", "");
    const std::string topic_override  = declare_parameter<std::string>("input_cloud_topic", "");

    std::string                  config_path = configured_path;
    DynamicObstacleTrackerConfig config;
    try {
        if (config_path.empty()) {
            config_path
                    = ament_index_cpp::get_package_share_directory("dynamic_obstacle_tracker") + "/config/default.yaml";
        }
        config = ConfigLoader(config_path).load();
    } catch (const std::exception& exception) {
        RCLCPP_FATAL(get_logger(), "Could not load tracker config '%s': %s", config_path.c_str(), exception.what());
        throw;
    }

    input_cloud_topic_        = topic_override.empty() ? config.dynamic_cloud_topic : topic_override;
    predicted_obstacle_topic_ = config.predicted_obstacle_topic;
    bbox_marker_topic_        = config.bbox_marker_topic;
    prediction_marker_topic_  = config.prediction_marker_topic;
    tracking_frame_           = normalizeFrame(config.tracking_frame);
    if (tracking_frame_.empty())
        throw std::invalid_argument("frame.tracking_frame must not be empty");

    config.tracker_params.frame_id = tracking_frame_;
    tracker_                       = std::make_unique<ObstacleTracker>(config.tracker_params, get_logger());

    RCLCPP_INFO(get_logger(), "Loaded tracker config from '%s'", config_path.c_str());

    predicted_obstacle_pub_
            = create_publisher<msg::DynamicObstacleTrajectory>(predicted_obstacle_topic_, rclcpp::QoS(5).reliable());
    bbox_marker_pub_
            = create_publisher<visualization_msgs::msg::MarkerArray>(bbox_marker_topic_, rclcpp::SensorDataQoS());
    prediction_marker_pub_
            = create_publisher<visualization_msgs::msg::MarkerArray>(prediction_marker_topic_, rclcpp::SensorDataQoS());

    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            input_cloud_topic_,
            rclcpp::SensorDataQoS(),
            std::bind(&DynamicObstacleTrackerNode::cloudCallback, this, std::placeholders::_1));

    RCLCPP_INFO(
            get_logger(),
            "Listening for dynamic points on '%s' in tracking frame '%s'",
            input_cloud_topic_.c_str(),
            tracking_frame_.c_str());
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

    const std::string frame_id = normalizeFrame(msg->header.frame_id);
    if (frame_id.empty()) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Input dynamic point cloud has no frame_id");
        return;
    }
    if (frame_id != tracking_frame_) {
        RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                5000,
                "Rejecting dynamic cloud in frame '%s'; tracker requires '%s'",
                frame_id.c_str(),
                tracking_frame_.c_str());
        return;
    }

    const rclcpp::Time stamp(msg->header.stamp);
    if (stamp.nanoseconds() <= 0) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Rejecting dynamic cloud with a zero timestamp");
        return;
    }
    if (last_cloud_stamp_.nanoseconds() > 0 && stamp <= last_cloud_stamp_) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Rejecting non-monotonic dynamic cloud");
        return;
    }

    const auto result = tracker_->update(cloud, stamp.seconds());
    last_cloud_stamp_ = stamp;

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

RCLCPP_COMPONENTS_REGISTER_NODE(dynamic_obstacle_tracker::DynamicObstacleTrackerNode)
