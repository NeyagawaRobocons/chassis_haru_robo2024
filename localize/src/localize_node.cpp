// description: オドメトリとMCLのポーズを統合してロボットのポーズを推定するノード
// 特筆事項：
// オドメトリは-43.18 mmだけy軸方向にずれている
#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/pose.hpp>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <tf2_eigen/tf2_eigen.hpp>
#include <list>
#include <limits>
#include <cmath>

using std::placeholders::_1;

class LocalizeNode : public rclcpp::Node
{
public:
  LocalizeNode() : Node("localize_node")
  {
    RCLCPP_INFO(this->get_logger(), "Localize node is initializing...");
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
    if (!current_odometry_pose_) {
      RCLCPP_INFO(this->get_logger(), "最初のodometry_poseを受信した");
      // 最初のodometry_poseを受信した場合
      current_odometry_pose_ = msg;
      if (!current_robot_pose_) {
        current_robot_pose_ = std::make_shared<geometry_msgs::msg::PoseStamped>();
      }
      current_robot_pose_->header = msg->header;
      current_robot_pose_->pose.position.x = msg->pose.position.x;
      current_robot_pose_->pose.position.y = msg->pose.position.y;
      current_robot_pose_->pose.position.z = msg->pose.position.z;
      current_robot_pose_->pose.orientation = msg->pose.orientation;
      robot_poses_.push_back(*current_robot_pose_); // オドメトリポーズをリストに追加
      return;
    }

    if (robot_poses_.empty()) {
      RCLCPP_ERROR(this->get_logger(), "robot_poses_ is empty");
      return;
    }

    // 差分の計算
    double dx = msg->pose.position.x - current_odometry_pose_->pose.position.x;
    double dy = msg->pose.position.y - current_odometry_pose_->pose.position.y;
    double dtheta = getYawFromPose(msg->pose) - getYawFromPose(current_odometry_pose_->pose);
    auto current_yaw = getYawFromPose(current_robot_pose_->pose) + dtheta;
    auto quaternion = getQuaternionFromYaw(current_yaw);

    // 現在のrobot_poseに差分を適用し，更新
    if (current_robot_pose_) {
      current_robot_pose_->pose.position.x += dx;
      current_robot_pose_->pose.position.y += dy;
      current_robot_pose_->pose.orientation.x = quaternion.x();
      current_robot_pose_->pose.orientation.y = quaternion.y();
      current_robot_pose_->pose.orientation.z = quaternion.z();
      current_robot_pose_->pose.orientation.w = quaternion.w();
      robot_pose_publisher_->publish(*current_robot_pose_);
      RCLCPP_INFO(this->get_logger(), "robot_pose has been updated");
    } else {
      RCLCPP_ERROR(this->get_logger(), "current_robot_pose_ is nullptr");
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
    RCLCPP_INFO(this->get_logger(), "finding nearest pose");
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
    RCLCPP_INFO(this->get_logger(), "nearest robot_pose has been found");

    // 最新のodometry_poseと置き換えられるodometry_poseの差分を計算
    double dx = latest_it->pose.position.x - nearest_it->pose.position.x;
    double dy = latest_it->pose.position.y - nearest_it->pose.position.y;
    double dtheta = getYawFromPose(latest_it->pose) - getYawFromPose(nearest_it->pose);

    // 差分をmcl_poseに適用して新しいrobot_poseを生成
    current_robot_pose_->header.stamp = this->get_clock()->now();
    current_robot_pose_->header.frame_id = "map";
    current_robot_pose_->pose.position.x = msg->pose.pose.position.x + dx;
    current_robot_pose_->pose.position.y = msg->pose.pose.position.y + dy;
    auto current_yaw = getYawFromPose(msg->pose.pose) + dtheta;
    auto quaternion = getQuaternionFromYaw(current_yaw);
    current_robot_pose_->pose.orientation.x = quaternion.x();
    current_robot_pose_->pose.orientation.y = quaternion.y();
    current_robot_pose_->pose.orientation.z = quaternion.z();
    current_robot_pose_->pose.orientation.w = quaternion.w();

    // robot_poseを更新
    robot_pose_publisher_->publish(*current_robot_pose_);

    // robot_poses_リストをクリア
    robot_poses_.clear();
    robot_poses_.push_back(*current_robot_pose_); // オドメトリポーズをリストに追加
  }

  double getYawFromPose(const geometry_msgs::msg::Pose& pose) {
      Eigen::Quaterniond quaternion(
          pose.orientation.w,
          pose.orientation.x,
          pose.orientation.y,
          pose.orientation.z);
      auto euler = quaternion.toRotationMatrix().eulerAngles(0, 1, 2);
      return euler[2]; // Yaw
  }

  // Yaw角から四元数を生成する関数
  tf2::Quaternion getQuaternionFromYaw(double yaw) {
      tf2::Quaternion quaternion;
      quaternion.setRPY(0, 0, yaw); // RollとPitchは0、Yawは引数から
      return quaternion;
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