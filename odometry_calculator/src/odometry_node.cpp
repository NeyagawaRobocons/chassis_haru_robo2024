#include <rclcpp/rclcpp.hpp>
#include "odometer_pub_sub.hpp"
#include "nucleo_agent/msg/odometer_data.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomPubSub>());
  rclcpp::shutdown();
  return 0;
}