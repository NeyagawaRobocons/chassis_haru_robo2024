#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "odometry_publisher.hpp"

using namespace std::chrono_literals;

OdomPub::OdomPub(
  const std::string& name_space, 
  const rclcpp::NodeOptions& options
): Node("odometry_node", name_space,options), count_(0){
  publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
    "odometry_pose", // トピック名
    rclcpp::QoS(10)
  );
  timer_ = this->create_wall_timer(
    500ms, // 0.5秒ごとにpublish
    [this](){
      auto msg = std::make_shared<geometry_msgs::msg::PoseStamped>(); // メッセージの型を指定
      // msg->data = "Hello " + std::to_string(count_++);
      RCLCPP_INFO(this->get_logger(), "publish odometry pose");
      publisher_->publish(*msg);
    }
  );
}
