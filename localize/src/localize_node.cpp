// 特筆事項：
// オドメトリは-43.18 mmだけy軸方向にずれている
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include <list>
#include <limits>
#include <cmath>

using std::placeholders::_1;

class LocalizeNode : public rclcpp::Node
{
public:
  LocalizeNode() : Node("localize_node")
  {
    // オドメトリポーズのサブスクライバーの初期化
    odometry_pose_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/odometry_pose", 10, std::bind(&LocalizeNode::odometry_pose_callback, this, _1));

    // MCLポーズのサブスクライバーの初期化
    mcl_pose_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/mcl_pose", 10, std::bind(&LocalizeNode::mcl_pose_callback, this, _1));

    // ロボットポーズのパブリッシャーの初期化
    robot_pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/robot_pose", 10);

    RCLCPP_INFO(this->get_logger(), "Localize node has been initialized");
  }

private:
  void odometry_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    // RCLCPP_INFO(this->get_logger(), "Received odometry pose");
    for (auto it = robot_poses_.begin(); it != robot_poses_.end(); ++it)
    {
      RCLCPP_INFO(this->get_logger(), "robot_poses_ x: %f, y: %f", it->pose.position.x, it->pose.position.y);
    }
    if (!current_odometry_pose_) {
      // 最初のodometry_poseを受信した場合
      current_odometry_pose_ = msg;
      current_robot_pose_->header = msg->header;
      current_robot_pose_->pose.position.x = msg->pose.position.x;
      current_robot_pose_->pose.position.y = msg->pose.position.y - 0.04318; // オドメトリのy軸方向のずれを補正
      current_robot_pose_->pose.position.z = msg->pose.position.z;
      current_robot_pose_->pose.orientation = msg->pose.orientation;
      robot_poses_.push_back(*current_robot_pose_); // オドメトリポーズをリストに追加
      return;
    }

    // 差分の計算
    double dx = msg->pose.position.x - current_odometry_pose_->pose.position.x;
    double dy = msg->pose.position.y - current_odometry_pose_->pose.position.y;

    // 現在のrobot_poseに差分を適用し，更新
    if (current_robot_pose_) {
      current_robot_pose_->pose.position.x += dx;
      current_robot_pose_->pose.position.y += dy;
      robot_pose_publisher_->publish(*current_robot_pose_);
    }

    // 現在のodometry_poseを更新
    current_odometry_pose_ = msg;
    robot_poses_.push_back(*current_robot_pose_); // オドメトリポーズをリストに追加
  }

  void mcl_pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Received MCL pose");
    if (robot_poses_.empty())
      return;

    // 最もタイムスタンプが近いrobot_poseを探索
    auto nearest_it = robot_poses_.begin();
    auto latest_it = --robot_poses_.end(); // 最新のrobot_poseを指すイテレータ
    double min_time_diff = std::numeric_limits<double>::max();
    for (auto it = robot_poses_.begin(); it != robot_poses_.end(); ++it)
    {
      double time_diff = std::fabs((rclcpp::Time(it->header.stamp) - rclcpp::Time(msg->header.stamp)).seconds());
      if (time_diff < min_time_diff)
      {
        nearest_it = it;
        min_time_diff = time_diff;
      }
    }

    // 最新のodometry_poseと置き換えられるodometry_poseの差分を計算
    double dx = latest_it->pose.position.x - nearest_it->pose.position.x;
    double dy = latest_it->pose.position.y - nearest_it->pose.position.y;

    // 差分をmcl_poseに適用して新しいrobot_poseを生成
    current_robot_pose_->header.stamp = this->get_clock()->now();
    current_robot_pose_->header.frame_id = "map";
    current_robot_pose_->pose.position.x = msg->pose.pose.position.x + dx;
    current_robot_pose_->pose.position.y = msg->pose.pose.position.y + dy;
    current_robot_pose_->pose.orientation = msg->pose.pose.orientation; // オリエンテーションの差分の適用は省略

    // robot_poseを更新
    robot_pose_publisher_->publish(*current_robot_pose_);

    // robot_poses_リストをクリア
    robot_poses_.clear();
  }

  std::list<geometry_msgs::msg::PoseStamped> robot_poses_;
  geometry_msgs::msg::PoseStamped::SharedPtr current_odometry_pose_;
  geometry_msgs::msg::PoseStamped::SharedPtr current_robot_pose_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr odometry_pose_subscriber_;
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