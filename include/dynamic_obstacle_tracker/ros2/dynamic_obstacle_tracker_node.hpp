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
    explicit DynamicObstacleTrackerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

  private:
    void cloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg);

    std::string  input_cloud_topic_;
    std::string  tracking_frame_;
    bool         debug_ = false;
    rclcpp::Time last_cloud_stamp_{0, 0, RCL_ROS_TIME};

    std::unique_ptr<ObstacleTracker>                                   tracker_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr     cloud_sub_;
    rclcpp::Publisher<msg::DynamicObstacleTrajectory>::SharedPtr       predicted_obstacle_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr bbox_marker_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr prediction_marker_pub_;
};

} // namespace dynamic_obstacle_tracker
