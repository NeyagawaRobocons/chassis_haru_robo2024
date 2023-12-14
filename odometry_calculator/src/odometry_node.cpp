#include <rclcpp/rclcpp.hpp>
#include "odometer_pub_sub.hpp"
#include "nucleo_agent/msg/odometer_data.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "odometry_class.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/quaternion.h>

Odometry odom(0.02504, 0.16218, 0.0, 0.0, 0.0);

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

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomPubSub>());
  rclcpp::shutdown();
  return 0;
}