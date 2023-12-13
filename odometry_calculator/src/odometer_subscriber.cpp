#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include "nucleo_agent/msg/odometer_data.hpp"  // メッセージ型のインクルード
#include "odometer_subscriber.hpp"

void OdomDataSub::_topic_callback(const nucleo_agent::msg::OdometerData::SharedPtr msg) {
  // メッセージの内容をログに出力
  RCLCPP_INFO(this->get_logger(), "Rotation: [%f, %f, %f], Angular Vel: [%f, %f, %f]",
              msg->rotation[0], msg->rotation[1], msg->rotation[2],
              msg->angular_vel[0], msg->angular_vel[1], msg->angular_vel[2]);
}

OdomDataSub::OdomDataSub(
  const rclcpp::NodeOptions& options
): OdomDataSub("", options) {}

OdomDataSub::OdomDataSub(
  const std::string& name_space, 
  const rclcpp::NodeOptions& options
): Node("odom_data_sub", name_space, options) {
  _subscription = this->create_subscription<nucleo_agent::msg::OdometerData>(
    "odometer_3wheel", // トピック名
    rclcpp::QoS(10),
    std::bind(&OdomDataSub::_topic_callback, this, std::placeholders::_1)
  );
}