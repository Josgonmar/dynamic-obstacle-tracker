#include "dynamic_obstacle_tracker/ros2/dynamic_obstacle_tracker_node.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <chrono>
#include <exception>
#include <functional>
#include <memory>
#include <rclcpp_components/register_node_macro.hpp>
#include <stdexcept>

#include "dynamic_obstacle_tracker/common/config_loader.hpp"
#include "dynamic_obstacle_tracker/common/utils.hpp"

namespace dynamic_obstacle_tracker {
namespace {

using SteadyClock = std::chrono::steady_clock;

double elapsedMilliseconds(const SteadyClock::time_point& start, const SteadyClock::time_point& end)
{
    return std::chrono::duration<double, std::milli>(end - start).count();
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

    input_cloud_topic_ = topic_override.empty() ? config.dynamic_cloud_topic : topic_override;
    tracking_frame_    = normalizeFrame(config.tracking_frame);
    debug_             = config.debug;
    if (tracking_frame_.empty())
        throw std::invalid_argument("frame.tracking_frame must not be empty");

    config.tracker_params.frame_id = tracking_frame_;
    tracker_                       = std::make_unique<ObstacleTracker>(config.tracker_params, get_logger());

    RCLCPP_INFO(get_logger(), "Loaded tracker config from '%s'", config_path.c_str());

    predicted_obstacle_pub_
            = create_publisher<msg::DynamicObstacleTrajectory>("obstacle_predicted_traj", rclcpp::QoS(5).reliable());
    bbox_marker_pub_
            = create_publisher<visualization_msgs::msg::MarkerArray>("obstacle_bbox_marker", rclcpp::QoS(5).reliable());
    prediction_marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
            "obstacle_prediction_marker", rclcpp::QoS(5).reliable());

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
    const auto callback_start = SteadyClock::now();

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

    const auto             conversion_start = SteadyClock::now();
    cilantro::PointCloud3f cloud;
    try {
        cloud = pointCloud2ToCilantro(*msg);
    } catch (const std::exception& exception) {
        RCLCPP_WARN(get_logger(), "Could not convert tracker input cloud: %s", exception.what());
        return;
    }
    const auto conversion_end = SteadyClock::now();

    const auto tracker_start = SteadyClock::now();
    const auto result        = tracker_->update(cloud, stamp.seconds());
    const auto tracker_end   = SteadyClock::now();
    last_cloud_stamp_        = stamp;

    const auto publication_start = SteadyClock::now();
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
    const auto publication_end = SteadyClock::now();

    if (debug_) {
        RCLCPP_INFO(
                get_logger(),
                "Tracker scan: input=%zu finite=%zu candidate_clusters=%zu accepted_clusters=%zu active_tracks=%zu "
                "predictions=%zu",
                result.input_point_count,
                result.finite_point_count,
                result.candidate_cluster_count,
                result.accepted_cluster_count,
                result.active_track_count,
                result.trajectories.size());
        RCLCPP_INFO(
                get_logger(),
                "Tracker timing [ms]: callback=%.2f ros_to_cilantro=%.2f core=%.2f publication=%.2f",
                elapsedMilliseconds(callback_start, publication_end),
                elapsedMilliseconds(conversion_start, conversion_end),
                elapsedMilliseconds(tracker_start, tracker_end),
                elapsedMilliseconds(publication_start, publication_end));
        RCLCPP_INFO(
                get_logger(),
                "Tracker core timing [ms]: cleanup=%.2f finite_filter=%.2f clustering=%.2f state_update=%.2f "
                "bbox_markers=%.2f prediction=%.2f total=%.2f",
                result.timings.state_cleanup_ms,
                result.timings.finite_filter_ms,
                result.timings.clustering_ms,
                result.timings.state_update_ms,
                result.timings.bbox_markers_ms,
                result.timings.prediction_ms,
                result.timings.total_ms);
    }
}

} // namespace dynamic_obstacle_tracker

RCLCPP_COMPONENTS_REGISTER_NODE(dynamic_obstacle_tracker::DynamicObstacleTrackerNode)
