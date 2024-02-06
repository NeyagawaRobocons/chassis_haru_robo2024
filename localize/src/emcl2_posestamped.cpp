// description: MCLの姿勢を受け取り、ロボットの姿勢をパブリッシュする仮のノード
#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/pose.hpp>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include <limits>
#include <cmath>

using std::placeholders::_1;

class LocalizeNode : public rclcpp::Node
{
public:
  LocalizeNode() : Node("localize_node")
  {
    RCLCPP_INFO(this->get_logger(), "Localize node is initializing...");

    // MCLポーズのサブスクライバーの初期化
    mcl_pose_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/mcl_pose", 10, std::bind(&LocalizeNode::mcl_pose_callback, this, _1));

    // ロボットポーズのパブリッシャーの初期化
    robot_pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/robot_pose", 10);

    RCLCPP_INFO(this->get_logger(), "Localize node has been initialized");
  }

private:
  void mcl_pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Received MCL pose");

    geometry_msgs::msg::PoseStamped robot_pose;
    robot_pose.header = msg->header;
    robot_pose.pose = msg->pose.pose;
    robot_pose_publisher_->publish(robot_pose);
  }

  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr mcl_pose_subscriber_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr robot_pose_publisher_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LocalizeNode>());
  rclcpp::shutdown();
  return 0;
}