#include "dynamic_obstacle_tracker/common/config_loader.hpp"

#include <yaml-cpp/yaml.h>

#include <stdexcept>
#include <utility>

namespace dynamic_obstacle_tracker {
namespace {

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

YAML::Node readSection(const YAML::Node& root, const char* key)
{
    const YAML::Node section = root[key];
    if (!section)
        return {};

    if (!section.IsMap())
        throw std::runtime_error("Config section '" + std::string(key) + "' must contain a YAML map");

    return section;
}

} // namespace

ConfigLoader::ConfigLoader(std::string config_path) : config_path_(std::move(config_path)) {}

DynamicObstacleTrackerConfig ConfigLoader::load() const
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

    const YAML::Node topics     = readSection(root, "topics");
    const YAML::Node frame      = readSection(root, "frame");
    const YAML::Node clustering = readSection(root, "clustering");
    const YAML::Node tracking   = readSection(root, "tracking");
    const YAML::Node prediction = readSection(root, "prediction");

    DynamicObstacleTrackerConfig config;
    config.predicted_obstacle_topic = readValue(topics, "predicted_obstacle_topic", config.predicted_obstacle_topic);
    config.bbox_marker_topic        = readValue(topics, "bbox_marker_topic", config.bbox_marker_topic);
    config.prediction_marker_topic  = readValue(topics, "prediction_marker_topic", config.prediction_marker_topic);
    config.output_frame             = readValue(frame, "output_frame", config.output_frame);

    auto& params              = config.tracker_params;
    params.cluster_tolerance  = readValue(clustering, "cluster_tolerance", params.cluster_tolerance);
    params.min_cluster_size   = readValue(clustering, "min_cluster_size", params.min_cluster_size);
    params.max_cluster_size   = readValue(clustering, "max_cluster_size", params.max_cluster_size);
    params.cluster_bbox_cutoff_size = readValue(
            clustering, "cluster_bbox_cutoff_size", params.cluster_bbox_cutoff_size);
    params.use_adaptive_kf   = readValue(tracking, "use_adaptive_kf", params.use_adaptive_kf);
    params.adaptive_kf_alpha = readValue(tracking, "adaptive_kf_alpha", params.adaptive_kf_alpha);
    params.adaptive_kf_dt    = readValue(tracking, "adaptive_kf_dt", params.adaptive_kf_dt);
    params.time_to_delete_old_obstacles
            = readValue(tracking, "time_to_delete_old_obstacles", params.time_to_delete_old_obstacles);
    params.velocity_threshold     = readValue(tracking, "velocity_threshold", params.velocity_threshold);
    params.acceleration_threshold = readValue(tracking, "acceleration_threshold", params.acceleration_threshold);
    params.max_history_size       = readValue(tracking, "max_history_size", params.max_history_size);
    params.prediction_horizon    = readValue(prediction, "prediction_horizon", params.prediction_horizon);
    params.prediction_dt         = readValue(prediction, "prediction_dt", params.prediction_dt);
    params.cutoff_length_threshold
            = readValue(prediction, "cutoff_length_threshold", params.cutoff_length_threshold);
    params.degree_for_pwp = readValue(prediction, "degree_for_pwp", params.degree_for_pwp);
    params.degree_for_poly = readValue(prediction, "degree_for_poly", params.degree_for_poly);
    params.min_observations_for_prediction
            = readValue(prediction, "min_observations_for_prediction", params.min_observations_for_prediction);
    params.max_obstacle_velocity = readValue(prediction, "max_obstacle_velocity", params.max_obstacle_velocity);

    return config;
}

} // namespace dynamic_obstacle_tracker
