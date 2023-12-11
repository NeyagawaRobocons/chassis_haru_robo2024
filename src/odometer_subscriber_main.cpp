#include <rclcpp/rclcpp.hpp>
#include "odometer_subscriber.hpp"

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomDataSub>());
  rclcpp::shutdown();
  return 0;
}