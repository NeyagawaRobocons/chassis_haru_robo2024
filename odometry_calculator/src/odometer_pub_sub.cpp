#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include "nucleo_agent/msg/odometer_data.hpp"  // メッセージ型のインクルード
#include "odometer_pub_sub.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/quaternion.h>

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
  initial_subscription = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "initialpose", // トピック名
    rclcpp::QoS(10),
    std::bind(&OdomPubSub::_initial_callback, this, std::placeholders::_1)
  );
  // declare parameters: radius, length
  this->declare_parameter("radius", 0.02504);
  this->declare_parameter("length", 0.1689);
  this->get_parameter("radius", radius);
  this->get_parameter("length", length);

  odom = Odometry(radius, length, 0.0, 0.0, 0.0);

  RCLCPP_INFO(this->get_logger(), "odometry_node has been started");
}

void OdomPubSub::_topic_callback(const nucleo_agent::msg::OdometerData::SharedPtr msg) {
  // メッセージの内容をログに出力
  RCLCPP_INFO(this->get_logger(), "receive odometer data");
  double tire_angle[3] = {msg->rotation[0], msg->rotation[1], msg->rotation[2]};
  // 位置の更新
  odom.update_pose(tire_angle);
  // 位置をパブリッシュ
  auto msg_pose = std::make_shared<geometry_msgs::msg::PoseStamped>();
  auto _pose = odom.get_pose();
  tf2::Quaternion q;
  q.setRPY(0, 0, _pose.theta);  // ロール、ピッチは0、ヨーを設定
  // Headerの設定
  msg_pose->header.stamp = rclcpp::Clock().now();
  msg_pose->header.frame_id = "map";  // フレームIDの設定
  msg_pose->pose.position.x = _pose.x;
  msg_pose->pose.position.y = _pose.y;
  msg_pose->pose.position.z = 0.0;
  msg_pose->pose.orientation = tf2::toMsg(q);

  RCLCPP_INFO(this->get_logger(), "publish odometry pose");
  publisher_->publish(*msg_pose);
}

void OdomPubSub::_initial_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
  // メッセージの内容をログに出力
  RCLCPP_INFO(this->get_logger(), "receive initial pose");

  // 初期位置の設定
  pose2D initial_pose;
  initial_pose.x = msg->pose.pose.position.x;
  initial_pose.y = msg->pose.pose.position.y;

  // クォータニオンからヨー角を取得
  tf2::Quaternion quat;
  tf2::fromMsg(msg->pose.pose.orientation, quat);
  double roll, pitch, yaw;
  tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
  initial_pose.theta = yaw;

  odom.set_pose(initial_pose);
}