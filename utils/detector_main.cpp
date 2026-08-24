#include "dynamic_obstacle_tracker/ros2/dynamic_point_detector_node.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<dynamic_obstacle_tracker::DynamicPointDetectorNode>());
    rclcpp::shutdown();
    return 0;
}
