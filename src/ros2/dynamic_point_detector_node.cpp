#include "dynamic_obstacle_tracker/ros2/dynamic_point_detector_node.hpp"

#include <pcl_conversions/pcl_conversions.h>
#include <tf2/exceptions.h>

#include <Eigen/Geometry>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <chrono>
#include <exception>
#include <functional>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <memory>
#include <rclcpp_components/register_node_macro.hpp>
#include <stdexcept>
#include <utility>

#include "dynamic_obstacle_tracker/common/config_loader.hpp"

namespace dynamic_obstacle_tracker {
namespace {

using SteadyClock = std::chrono::steady_clock;

double elapsedMilliseconds(const SteadyClock::time_point& start, const SteadyClock::time_point& end)
{
    return std::chrono::duration<double, std::milli>(end - start).count();
}

std::string normalizeFrame(std::string frame)
{
    while (!frame.empty() && frame.front() == '/') frame.erase(frame.begin());
    return frame;
}

bool transformToEigen(
        const geometry_msgs::msg::TransformStamped& transform,
        Eigen::Quaterniond&                         rotation,
        Eigen::Vector3d&                            translation)
{
    const auto& quaternion = transform.transform.rotation;
    rotation               = Eigen::Quaterniond(quaternion.w, quaternion.x, quaternion.y, quaternion.z);
    translation            = Eigen::Vector3d(
            transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z);
    if (!rotation.coeffs().allFinite() || !translation.allFinite() || rotation.norm() < 1e-12)
        return false;
    rotation.normalize();
    return true;
}

} // namespace

DynamicPointDetectorNode::DynamicPointDetectorNode(const rclcpp::NodeOptions& options) :
        Node("dynamic_point_detector_node", options)
{
    const std::string configured_path = declare_parameter<std::string>("config_file", "");
    const std::string topic_override  = declare_parameter<std::string>("input_cloud_topic", "");

    std::string                config_path = configured_path;
    DynamicPointDetectorConfig config;
    try {
        if (config_path.empty()) {
            config_path
                    = ament_index_cpp::get_package_share_directory("dynamic_obstacle_tracker") + "/config/default.yaml";
        }
        config = DetectorConfigLoader(config_path).load();
    } catch (const std::exception& exception) {
        RCLCPP_FATAL(get_logger(), "Could not load detector config '%s': %s", config_path.c_str(), exception.what());
        throw;
    }

    input_cloud_topic_    = topic_override.empty() ? config.input_cloud_topic : topic_override;
    tracking_frame_       = normalizeFrame(config.tracking_frame);
    sensor_frame_         = normalizeFrame(config.sensor_frame);
    debug_                = config.debug;
    publish_static_cloud_ = config.publish_static_cloud;
    tf_timeout_           = config.tf_timeout;

    if (tracking_frame_.empty())
        throw std::invalid_argument("frame.tracking_frame must not be empty");
    if (sensor_frame_.empty())
        throw std::invalid_argument("frame.sensor_frame must not be empty");
    if (tf_timeout_ <= 0.0)
        throw std::invalid_argument("detector.tf_timeout must be positive");

    detector_    = std::make_unique<DynamicPointDetector>(config.detector_params);
    tf_buffer_   = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    dynamic_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("dynamic_points", rclcpp::SensorDataQoS());
    if (publish_static_cloud_) {
        static_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("static_points", rclcpp::SensorDataQoS());
    }

    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            input_cloud_topic_,
            rclcpp::SensorDataQoS(),
            std::bind(&DynamicPointDetectorNode::cloudCallback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "Loaded detector config from '%s'", config_path.c_str());
    RCLCPP_INFO(
            get_logger(),
            "Detecting dynamic points from '%s' in tracking frame '%s' using sensor frame '%s'",
            input_cloud_topic_.c_str(),
            tracking_frame_.c_str(),
            sensor_frame_.c_str());
}

void DynamicPointDetectorNode::cloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg)
{
    const auto callback_start = SteadyClock::now();

    rclcpp::Time stamp(msg->header.stamp);
    if (stamp.nanoseconds() <= 0) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Dropping point cloud with a zero timestamp");
        return;
    }
    if (last_cloud_stamp_.nanoseconds() > 0 && stamp <= last_cloud_stamp_) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Dropping non-monotonic point cloud");
        return;
    }

    const std::string cloud_frame = normalizeFrame(msg->header.frame_id);
    if (cloud_frame.empty()) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Dropping point cloud without a frame_id");
        return;
    }

    const auto                     conversion_start = SteadyClock::now();
    pcl::PointCloud<pcl::PointXYZ> input_cloud;
    try {
        pcl::fromROSMsg(*msg, input_cloud);
    } catch (const std::exception& exception) {
        RCLCPP_WARN(get_logger(), "Could not convert detector input cloud: %s", exception.what());
        return;
    }
    const auto conversion_end = SteadyClock::now();

    const auto                     cloud_transform_start = SteadyClock::now();
    pcl::PointCloud<pcl::PointXYZ> tracking_cloud;
    if (cloud_frame == tracking_frame_) {
        tracking_cloud = std::move(input_cloud);
    } else {
        try {
            const auto transform = tf_buffer_->lookupTransform(
                    tracking_frame_, cloud_frame, stamp, rclcpp::Duration::from_seconds(tf_timeout_));
            Eigen::Quaterniond rotation;
            Eigen::Vector3d    translation;
            if (!transformToEigen(transform, rotation, translation)) {
                RCLCPP_WARN(get_logger(), "Cloud transform contains invalid values");
                return;
            }

            tracking_cloud.reserve(input_cloud.size());
            for (const auto& point : input_cloud.points) {
                const Eigen::Vector3d transformed = rotation * Eigen::Vector3d(point.x, point.y, point.z) + translation;
                tracking_cloud.push_back(pcl::PointXYZ(
                        static_cast<float>(transformed.x()),
                        static_cast<float>(transformed.y()),
                        static_cast<float>(transformed.z())));
            }
        } catch (const tf2::TransformException& exception) {
            RCLCPP_WARN_THROTTLE(
                    get_logger(),
                    *get_clock(),
                    2000,
                    "Could not transform cloud from '%s' to tracking frame '%s': %s",
                    cloud_frame.c_str(),
                    tracking_frame_.c_str(),
                    exception.what());
            return;
        }
    }
    const auto cloud_transform_end = SteadyClock::now();

    const auto      sensor_tf_start = SteadyClock::now();
    Eigen::Vector3d sensor_origin;
    try {
        const auto transform = tf_buffer_->lookupTransform(
                tracking_frame_, sensor_frame_, stamp, rclcpp::Duration::from_seconds(tf_timeout_));
        sensor_origin = Eigen::Vector3d(
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z);
    } catch (const tf2::TransformException& exception) {
        RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                2000,
                "Could not resolve sensor frame '%s' in '%s': %s",
                sensor_frame_.c_str(),
                tracking_frame_.c_str(),
                exception.what());
        return;
    }
    const auto sensor_tf_end = SteadyClock::now();

    const bool collect_static_points = static_cloud_pub_ && static_cloud_pub_->get_subscription_count() > 0;

    DynamicPointDetectionResult result;
    const auto                  detector_start = SteadyClock::now();
    try {
        result = detector_->update(tracking_cloud, sensor_origin, stamp.seconds(), collect_static_points);
    } catch (const std::exception& exception) {
        RCLCPP_ERROR(get_logger(), "Dynamic-point detector rejected cloud: %s", exception.what());
        return;
    }
    const auto detector_end = SteadyClock::now();

    const auto ros_output_start = SteadyClock::now();
    auto       dynamic_msg      = std::make_unique<sensor_msgs::msg::PointCloud2>();
    pcl::toROSMsg(result.dynamic_points, *dynamic_msg);
    dynamic_msg->header.stamp    = msg->header.stamp;
    dynamic_msg->header.frame_id = tracking_frame_;
    const auto output_header     = dynamic_msg->header;
    dynamic_cloud_pub_->publish(std::move(dynamic_msg));

    if (collect_static_points) {
        sensor_msgs::msg::PointCloud2 static_msg;
        pcl::toROSMsg(result.static_points, static_msg);
        static_msg.header = output_header;
        static_cloud_pub_->publish(static_msg);
    }
    const auto ros_output_end = SteadyClock::now();

    last_cloud_stamp_ = stamp;
    if (debug_) {
        const auto& map_timings = result.timings.voxel_map;
        RCLCPP_INFO(
                get_logger(),
                "Detector scan: input=%zu accepted=%zu state_changes=%zu dynamic_voxels=%zu dynamic_points=%zu "
                "otsu=%.2f blocks=%zu removed=%zu",
                result.input_point_count,
                result.accepted_point_count,
                result.state_change_voxel_count,
                result.dynamic_voxel_count,
                result.dynamic_points.size(),
                result.otsu_threshold,
                result.allocated_block_count,
                result.removed_block_count);
        RCLCPP_INFO(
                get_logger(),
                "Detector timing [ms]: callback=%.2f ros_to_pcl=%.2f cloud_tf=%.2f sensor_tf=%.2f core=%.2f "
                "ros_output=%.2f",
                elapsedMilliseconds(callback_start, ros_output_end),
                elapsedMilliseconds(conversion_start, conversion_end),
                elapsedMilliseconds(cloud_transform_start, cloud_transform_end),
                elapsedMilliseconds(sensor_tf_start, sensor_tf_end),
                elapsedMilliseconds(detector_start, detector_end),
                elapsedMilliseconds(ros_output_start, ros_output_end));
        RCLCPP_INFO(
                get_logger(),
                "Detector core timing [ms]: preprocessing=%.2f map=%.2f output=%.2f total=%.2f | ray_casting=%.2f "
                "edf=%.2f hmm=%.2f spatial_conv=%.2f temporal_conv=%.2f segmentation=%.2f garbage_collection=%.2f "
                "scan_cleanup=%.2f map_total=%.2f",
                result.timings.preprocessing_ms,
                result.timings.map_update_ms,
                result.timings.output_assembly_ms,
                result.timings.total_ms,
                map_timings.ray_casting_ms,
                map_timings.edf_construction_ms,
                map_timings.hmm_update_ms,
                map_timings.spatial_convolution_ms,
                map_timings.temporal_convolution_ms,
                map_timings.segmentation_ms,
                map_timings.garbage_collection_ms,
                map_timings.scan_cleanup_ms,
                map_timings.total_ms);
    }
}

} // namespace dynamic_obstacle_tracker

RCLCPP_COMPONENTS_REGISTER_NODE(dynamic_obstacle_tracker::DynamicPointDetectorNode)
