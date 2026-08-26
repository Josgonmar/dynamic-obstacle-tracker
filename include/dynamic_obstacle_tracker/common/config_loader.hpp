#pragma once

#include <stdexcept>
#include <string>
#include <utility>

#include "dynamic_obstacle_tracker/common/yaml_config_utils.hpp"
#include "dynamic_obstacle_tracker/detector/dynamic_point_detector.hpp"
#include "dynamic_obstacle_tracker/tracker/obstacle_tracker.hpp"

namespace dynamic_obstacle_tracker {

inline constexpr char kDefaultDynamicCloudTopic[] = "/dynamic_point_detector/dynamic_points";

struct DynamicPointDetectorConfig
{
    std::string                input_cloud_topic    = "deskewed_cloud";
    std::string                dynamic_cloud_topic  = kDefaultDynamicCloudTopic;
    std::string                tracking_frame       = "odom";
    std::string                sensor_frame         = "lidar_link";
    bool                       debug                = false;
    bool                       publish_static_cloud = true;
    double                     tf_timeout           = 0.05;
    DynamicPointDetectorParams detector_params;
};

class DetectorConfigLoader
{
  public:
    explicit DetectorConfigLoader(std::string config_path) : config_path_(std::move(config_path)) {}

    DynamicPointDetectorConfig load() const
    {
        if (config_path_.empty())
            throw std::invalid_argument("The dynamic point detector config path is empty");

        YAML::Node root;
        try {
            root = YAML::LoadFile(config_path_);
        } catch (const YAML::Exception& exception) {
            throw std::runtime_error("Could not load YAML file '" + config_path_ + "': " + exception.what());
        }

        if (!root || !root.IsMap())
            throw std::runtime_error("The dynamic point detector config must contain a YAML map at its root");

        const YAML::Node topics        = config_loader_detail::readSection(root, "topics");
        const YAML::Node frame         = config_loader_detail::readSection(root, "frame");
        const YAML::Node detector      = config_loader_detail::readSection(root, "detector");
        const YAML::Node preprocessing = config_loader_detail::readSection(detector, "preprocessing");
        const YAML::Node voxel_map     = config_loader_detail::readSection(detector, "voxel_map");
        const YAML::Node hmm_mos       = config_loader_detail::readSection(detector, "hmm_mos");

        DynamicPointDetectorConfig config;
        config.input_cloud_topic
                = config_loader_detail::readNonEmptyString(topics, "deskewed_cloud_topic", config.input_cloud_topic);
        config.dynamic_cloud_topic
                = config_loader_detail::readNonEmptyString(topics, "dynamic_cloud_topic", config.dynamic_cloud_topic);
        config.tracking_frame = config_loader_detail::readValue(frame, "tracking_frame", config.tracking_frame);
        config.sensor_frame   = config_loader_detail::readValue(frame, "sensor_frame", config.sensor_frame);
        config.debug          = config_loader_detail::readValue(detector, "debug", config.debug);
        config.publish_static_cloud
                = config_loader_detail::readValue(detector, "publish_static_cloud", config.publish_static_cloud);
        config.tf_timeout = config_loader_detail::readValue(detector, "tf_timeout", config.tf_timeout);

        auto& params         = config.detector_params;
        params.minimum_range = config_loader_detail::readValue(preprocessing, "minimum_range", params.minimum_range);
        params.maximum_range = config_loader_detail::readValue(preprocessing, "maximum_range", params.maximum_range);
        params.remove_ground = config_loader_detail::readValue(preprocessing, "remove_ground", params.remove_ground);
        params.ground_height = config_loader_detail::readValue(preprocessing, "ground_height", params.ground_height);
        params.ground_clearance
                = config_loader_detail::readValue(preprocessing, "ground_clearance", params.ground_clearance);
        params.voxel_size = config_loader_detail::readValue(voxel_map, "voxel_size", params.voxel_size);
        params.voxels_per_block
                = config_loader_detail::readValue(voxel_map, "voxels_per_block", params.voxels_per_block);
        params.active_radius = config_loader_detail::readValue(voxel_map, "active_radius", params.active_radius);
        params.garbage_collection_period = config_loader_detail::readValue(
                voxel_map, "garbage_collection_period", params.garbage_collection_period);
        params.occupancy_sigma  = config_loader_detail::readValue(hmm_mos, "occupancy_sigma", params.occupancy_sigma);
        params.belief_threshold = config_loader_detail::readValue(hmm_mos, "belief_threshold", params.belief_threshold);
        params.transition_epsilon
                = config_loader_detail::readValue(hmm_mos, "transition_epsilon", params.transition_epsilon);
        params.convolution_size = config_loader_detail::readValue(hmm_mos, "convolution_size", params.convolution_size);
        params.local_window_size
                = config_loader_detail::readValue(hmm_mos, "local_window_size", params.local_window_size);
        params.global_window_size
                = config_loader_detail::readValue(hmm_mos, "global_window_size", params.global_window_size);
        params.minimum_otsu_threshold
                = config_loader_detail::readValue(hmm_mos, "minimum_otsu_threshold", params.minimum_otsu_threshold);
        params.dilation_radius_voxels
                = config_loader_detail::readValue(hmm_mos, "dilation_radius_voxels", params.dilation_radius_voxels);
        params.histogram_bins = config_loader_detail::readValue(hmm_mos, "histogram_bins", params.histogram_bins);

        return config;
    }

  private:
    std::string config_path_;
};

struct DynamicObstacleTrackerConfig
{
    std::string           dynamic_cloud_topic = kDefaultDynamicCloudTopic;
    std::string           tracking_frame      = "odom";
    bool                  debug               = false;
    ObstacleTrackerParams tracker_params;
};

class ConfigLoader
{
  public:
    explicit ConfigLoader(std::string config_path) : config_path_(std::move(config_path)) {}

    DynamicObstacleTrackerConfig load() const
    {
        if (config_path_.empty())
            throw std::invalid_argument("The dynamic obstacle tracker config path is empty");

        YAML::Node root;
        try {
            root = YAML::LoadFile(config_path_);
        } catch (const YAML::Exception& exception) {
            throw std::runtime_error("Could not load YAML file '" + config_path_ + "': " + exception.what());
        }

        if (!root || !root.IsMap())
            throw std::runtime_error("The dynamic obstacle tracker config must contain a YAML map at its root");

        const YAML::Node topics     = config_loader_detail::readSection(root, "topics");
        const YAML::Node frame      = config_loader_detail::readSection(root, "frame");
        const YAML::Node tracker    = config_loader_detail::readSection(root, "tracker");
        const YAML::Node clustering = config_loader_detail::readSection(tracker, "clustering");
        const YAML::Node tracking   = config_loader_detail::readSection(tracker, "tracking");
        const YAML::Node prediction = config_loader_detail::readSection(tracker, "prediction");

        DynamicObstacleTrackerConfig config;
        config.dynamic_cloud_topic
                = config_loader_detail::readNonEmptyString(topics, "dynamic_cloud_topic", config.dynamic_cloud_topic);
        config.tracking_frame = config_loader_detail::readValue(frame, "tracking_frame", config.tracking_frame);
        config.debug          = config_loader_detail::readValue(tracker, "debug", config.debug);

        auto& params = config.tracker_params;
        params.cluster_tolerance
                = config_loader_detail::readValue(clustering, "cluster_tolerance", params.cluster_tolerance);
        params.min_cluster_size
                = config_loader_detail::readValue(clustering, "min_cluster_size", params.min_cluster_size);
        params.max_cluster_size
                = config_loader_detail::readValue(clustering, "max_cluster_size", params.max_cluster_size);
        params.cluster_bbox_cutoff_size = config_loader_detail::readValue(
                clustering, "cluster_bbox_cutoff_size", params.cluster_bbox_cutoff_size);
        params.use_adaptive_kf = config_loader_detail::readValue(tracking, "use_adaptive_kf", params.use_adaptive_kf);
        params.adaptive_kf_alpha
                = config_loader_detail::readValue(tracking, "adaptive_kf_alpha", params.adaptive_kf_alpha);
        params.adaptive_kf_dt = config_loader_detail::readValue(tracking, "adaptive_kf_dt", params.adaptive_kf_dt);
        params.time_to_delete_old_obstacles = config_loader_detail::readValue(
                tracking, "time_to_delete_old_obstacles", params.time_to_delete_old_obstacles);
        params.velocity_threshold
                = config_loader_detail::readValue(tracking, "velocity_threshold", params.velocity_threshold);
        params.acceleration_threshold
                = config_loader_detail::readValue(tracking, "acceleration_threshold", params.acceleration_threshold);
        params.max_history_size
                = config_loader_detail::readValue(tracking, "max_history_size", params.max_history_size);
        params.prediction_horizon
                = config_loader_detail::readValue(prediction, "prediction_horizon", params.prediction_horizon);
        params.prediction_dt = config_loader_detail::readValue(prediction, "prediction_dt", params.prediction_dt);
        params.cutoff_length_threshold = config_loader_detail::readValue(
                prediction, "cutoff_length_threshold", params.cutoff_length_threshold);
        params.degree_for_pwp  = config_loader_detail::readValue(prediction, "degree_for_pwp", params.degree_for_pwp);
        params.degree_for_poly = config_loader_detail::readValue(prediction, "degree_for_poly", params.degree_for_poly);
        params.min_observations_for_prediction = config_loader_detail::readValue(
                prediction, "min_observations_for_prediction", params.min_observations_for_prediction);
        params.max_obstacle_velocity
                = config_loader_detail::readValue(prediction, "max_obstacle_velocity", params.max_obstacle_velocity);

        return config;
    }

  private:
    std::string config_path_;
};

} // namespace dynamic_obstacle_tracker
