#include "dynamic_obstacle_tracker/ros2/dynamic_obstacle_tracker_node.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<dynamic_obstacle_tracker::DynamicObstacleTrackerNode>());
    rclcpp::shutdown();
    return 0;
}
