#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include "ldlidar_node.h" // LD06ノードのヘッダーを想定
#include "laser_transformation.cpp" // LidarTransformerノードのヘッダーを想定

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();

  auto node_ld06 = std::make_shared<LD06>();
  auto node_lidar_transformer = std::make_shared<LidarTransformer>();

  executor->add_node(node_ld06);
  executor->add_node(node_lidar_transformer);

  executor->spin();

  rclcpp::shutdown();
  return 0;
}
