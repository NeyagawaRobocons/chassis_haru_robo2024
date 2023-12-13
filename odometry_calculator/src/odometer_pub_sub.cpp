#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "nucleo_agent/msg/odometer_data.hpp"  // メッセージ型のインクルード
#include "odometer_pub_sub.hpp"

OdomPubSub::OdomPubSub(
  const rclcpp::NodeOptions& options
): OdomPubSub("", options) {}

OdomPubSub::OdomPubSub(
  const std::string& name_space, 
  const rclcpp::NodeOptions& options
): Node("odometry_node", name_space, options), count_(0) {
  publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
    "odometry_pose", // トピック名
    rclcpp::QoS(10)
  );
  _subscription = this->create_subscription<nucleo_agent::msg::OdometerData>(
    "odometer_3wheel", // トピック名
    rclcpp::QoS(10),
    std::bind(&OdomPubSub::_topic_callback, this, std::placeholders::_1)
  );
}