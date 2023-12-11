#include <rclcpp/rclcpp.hpp>
#include "nucleo_agent/msg/odometer_data.hpp"

class OdomDataSub : public rclcpp::Node{
private:
  rclcpp::Subscription<nucleo_agent::msg::OdometerData>::SharedPtr _subscription;
  void _topic_callback(const nucleo_agent::msg::OdometerData::SharedPtr msg);
public:
  OdomDataSub(
    const rclcpp::NodeOptions& options = rclcpp::NodeOptions()
  );
  OdomDataSub(
    const std::string& name_space, 
    const rclcpp::NodeOptions& options = rclcpp::NodeOptions()
  );
};