#include "rclcpp/rclcpp.hpp"
#include "laser_transformation.cpp"
#include "ldlidar/ldlidar_node.h"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

    auto cut_node_front = std::make_shared<LidarTransformer>("front_scan_cut", rclcpp::NodeOptions().namespace_("front"));
    auto cut_node_back = std::make_shared<LidarTransformer>("back_scan_cut", rclcpp::NodeOptions().namespace_("back"));

    auto node_front = std::make_shared<LD06>("front_lidar", rclcpp::NodeOptions().namespace_("front"));
    auto node_back = std::make_shared<LD06>("back_lidar", rclcpp::NodeOptions().namespace_("back"));

    executor->add_node(cut_node_front);
    executor->add_node(cut_node_back);

    executor->add_node(node_front);
    executor->add_node(node_back);

    executor->spin();

    rclcpp::shutdown();
    return 0;
}
