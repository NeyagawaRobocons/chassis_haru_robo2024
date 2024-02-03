#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

class TimestampPublisherNode : public rclcpp::Node
{
public:
    TimestampPublisherNode() : Node("timestamp_publisher_node")
    {
        // mcl_poseパブリッシャー
        mcl_pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/mcl_pose", 10);

        // タイマーを使用して定期的にメッセージをパブリッシュ
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(5000),  // 5秒ごと
            std::bind(&TimestampPublisherNode::publish_messages, this));
    }

private:
    void publish_messages()
    {
        auto now = this->get_clock()->now();

        // mcl_poseメッセージのパブリッシュ
        auto mcl_pose_msg = geometry_msgs::msg::PoseWithCovarianceStamped();
        mcl_pose_msg.header.stamp = now;
        mcl_pose_msg.header.frame_id = "map";
        mcl_pose_msg.pose.pose.position.x = 1.5;
        mcl_pose_msg.pose.pose.position.y = 2.5;
        mcl_pose_msg.pose.pose.orientation.w = 1.0;
        // 共分散は例示のために全て0を設定
        for (auto& value : mcl_pose_msg.pose.covariance) {
            value = 0.0;
        }
        mcl_pose_publisher_->publish(mcl_pose_msg);
    }

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr odometry_pose_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr mcl_pose_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TimestampPublisherNode>());
    rclcpp::shutdown();
    return 0;
}