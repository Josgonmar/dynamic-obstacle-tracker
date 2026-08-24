#pragma once

#include <yaml-cpp/yaml.h>

#include <stdexcept>
#include <string>
#include <utility>

#include "dynamic_obstacle_tracker/tracker/obstacle_tracker.hpp"

namespace dynamic_obstacle_tracker {

namespace config_loader_detail {

template <typename T>
T readValue(const YAML::Node& root, const char* key, const T& fallback)
{
    const YAML::Node value = root[key];
    if (!value)
        return fallback;

    try {
        return value.as<T>();
    } catch (const YAML::Exception& exception) {
        throw std::runtime_error("Invalid value for '" + std::string(key) + "': " + exception.what());
    }
}

inline YAML::Node readSection(const YAML::Node& root, const char* key)
{
    const YAML::Node section = root[key];
    if (!section)
        return {};

    if (!section.IsMap())
        throw std::runtime_error("Config section '" + std::string(key) + "' must contain a YAML map");

    return section;
}

} // namespace config_loader_detail

struct DynamicObstacleTrackerConfig
{
    std::string           predicted_obstacle_topic = "predicted_obstacles";
    std::string           bbox_marker_topic        = "cluster_bounding_boxes";
    std::string           prediction_marker_topic  = "tracked_obstacles";
    std::string           output_frame;
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
        const YAML::Node clustering = config_loader_detail::readSection(root, "clustering");
        const YAML::Node tracking   = config_loader_detail::readSection(root, "tracking");
        const YAML::Node prediction = config_loader_detail::readSection(root, "prediction");

        DynamicObstacleTrackerConfig config;
        config.predicted_obstacle_topic = config_loader_detail::readValue(
                topics, "predicted_obstacle_topic", config.predicted_obstacle_topic);
        config.bbox_marker_topic = config_loader_detail::readValue(
                topics, "bbox_marker_topic", config.bbox_marker_topic);
        config.prediction_marker_topic = config_loader_detail::readValue(
                topics, "prediction_marker_topic", config.prediction_marker_topic);
        config.output_frame = config_loader_detail::readValue(frame, "output_frame", config.output_frame);

        auto& params = config.tracker_params;
        params.cluster_tolerance = config_loader_detail::readValue(
                clustering, "cluster_tolerance", params.cluster_tolerance);
        params.min_cluster_size = config_loader_detail::readValue(
                clustering, "min_cluster_size", params.min_cluster_size);
        params.max_cluster_size = config_loader_detail::readValue(
                clustering, "max_cluster_size", params.max_cluster_size);
        params.cluster_bbox_cutoff_size = config_loader_detail::readValue(
                clustering, "cluster_bbox_cutoff_size", params.cluster_bbox_cutoff_size);
        params.use_adaptive_kf = config_loader_detail::readValue(
                tracking, "use_adaptive_kf", params.use_adaptive_kf);
        params.adaptive_kf_alpha = config_loader_detail::readValue(
                tracking, "adaptive_kf_alpha", params.adaptive_kf_alpha);
        params.adaptive_kf_dt = config_loader_detail::readValue(
                tracking, "adaptive_kf_dt", params.adaptive_kf_dt);
        params.time_to_delete_old_obstacles = config_loader_detail::readValue(
                tracking, "time_to_delete_old_obstacles", params.time_to_delete_old_obstacles);
        params.velocity_threshold = config_loader_detail::readValue(
                tracking, "velocity_threshold", params.velocity_threshold);
        params.acceleration_threshold = config_loader_detail::readValue(
                tracking, "acceleration_threshold", params.acceleration_threshold);
        params.max_history_size = config_loader_detail::readValue(
                tracking, "max_history_size", params.max_history_size);
        params.prediction_horizon = config_loader_detail::readValue(
                prediction, "prediction_horizon", params.prediction_horizon);
        params.prediction_dt = config_loader_detail::readValue(
                prediction, "prediction_dt", params.prediction_dt);
        params.cutoff_length_threshold = config_loader_detail::readValue(
                prediction, "cutoff_length_threshold", params.cutoff_length_threshold);
        params.degree_for_pwp = config_loader_detail::readValue(
                prediction, "degree_for_pwp", params.degree_for_pwp);
        params.degree_for_poly = config_loader_detail::readValue(
                prediction, "degree_for_poly", params.degree_for_poly);
        params.min_observations_for_prediction = config_loader_detail::readValue(
                prediction, "min_observations_for_prediction", params.min_observations_for_prediction);
        params.max_obstacle_velocity = config_loader_detail::readValue(
                prediction, "max_obstacle_velocity", params.max_obstacle_velocity);

        return config;
    }

  private:
    std::string config_path_;
};

} // namespace dynamic_obstacle_tracker
