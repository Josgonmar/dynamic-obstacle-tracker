#pragma once

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <string>

#include "dynamic_obstacle_tracker/detector/dynamic_point_detector.hpp"

namespace dynamic_obstacle_tracker {

class DynamicPointDetectorNode final : public rclcpp::Node
{
  public:
    explicit DynamicPointDetectorNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

  private:
    void cloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg);

    std::string  input_cloud_topic_;
    std::string  dynamic_cloud_topic_;
    std::string  static_cloud_topic_;
    std::string  tracking_frame_;
    std::string  sensor_frame_;
    bool         publish_static_cloud_ = true;
    double       tf_timeout_           = 0.05;
    rclcpp::Time last_cloud_stamp_{0, 0, RCL_ROS_TIME};

    std::unique_ptr<DynamicPointDetector>       detector_;
    std::shared_ptr<tf2_ros::Buffer>            tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr    dynamic_cloud_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr    static_cloud_pub_;
};

} // namespace dynamic_obstacle_tracker
