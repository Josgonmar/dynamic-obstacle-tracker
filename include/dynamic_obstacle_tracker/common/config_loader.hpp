#pragma once

#include <string>

#include "dynamic_obstacle_tracker/tracker/obstacle_tracker.hpp"

namespace dynamic_obstacle_tracker {

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
    explicit ConfigLoader(std::string config_path);

    DynamicObstacleTrackerConfig load() const;

  private:
    std::string config_path_;
};

} // namespace dynamic_obstacle_tracker
