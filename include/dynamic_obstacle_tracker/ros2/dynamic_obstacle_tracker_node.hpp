#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <string>

#include "dynamic_obstacle_tracker/tracker/obstacle_tracker.hpp"

namespace dynamic_obstacle_tracker {

class DynamicObstacleTrackerNode final : public rclcpp::Node
{
  public:
    DynamicObstacleTrackerNode();

  private:
    void cloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg);

    std::string input_cloud_topic_;
    std::string predicted_obstacle_topic_;
    std::string bbox_marker_topic_;
    std::string prediction_marker_topic_;
    std::string output_frame_;

    std::unique_ptr<ObstacleTracker>                                   tracker_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr     cloud_sub_;
    rclcpp::Publisher<msg::DynamicObstacleTrajectory>::SharedPtr       predicted_obstacle_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr bbox_marker_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr prediction_marker_pub_;
};

} // namespace dynamic_obstacle_tracker
