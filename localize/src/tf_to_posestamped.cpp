
/*
- description: publish the pose of robot with regularly observing the tf
- input: `/tf` (topic: tf2_msgs/TFMessage)
    - parent_frame_id: `map`
    - child_frame_id: `base_link`
- output: `/robot_pose` (topic: geometry_msgs/PoseStamped)
*/

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <list>
#include <limits>
#include <cmath>

using std::placeholders::_1;
using namespace std::chrono_literals;

class TFToPoseStamped : public rclcpp::Node
{
public:
  TFToPoseStamped() : Node("tf_to_posestamped")
  {
    RCLCPP_INFO(this->get_logger(), "tf_to_posestamped node is initializing...");
    // declare parameters
    this->declare_parameter<std::string>("parent_frame_id", "map");
    this->declare_parameter<std::string>("child_frame_id", "base_link");
    this->declare_parameter<std::string>("output_topic", "robot_pose");
    this->declare_parameter<double>("publish_rate", 20.0);
    parent_frame_id = this->get_parameter("parent_frame_id").as_string();
    child_frame_id = this->get_parameter("child_frame_id").as_string();
    auto publish_rate = this->get_parameter("publish_rate").as_double();
    auto output_topic = this->get_parameter("output_topic").as_string();

    // display parameters
    RCLCPP_INFO(this->get_logger(), "parent_frame_id: %s", parent_frame_id.c_str());
    RCLCPP_INFO(this->get_logger(), "child_frame_id: %s", child_frame_id.c_str());
    RCLCPP_INFO(this->get_logger(), "output_topic: %s", output_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "publish_rate: %f", publish_rate);

    pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(output_topic, 10);

    // tfのサブスクライバーの初期化
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    timer_ = this->create_wall_timer(
        std::chrono::duration<double>(1.0 / publish_rate), 
        std::bind(&TFToPoseStamped::publishPose, this));

    RCLCPP_INFO(this->get_logger(), "duration: %f", 1.0 / publish_rate);

    RCLCPP_INFO(this->get_logger(), "tf_to_posestamped node has been initialized");
  }

private:
  void publishPose () {
    geometry_msgs::msg::TransformStamped transform;
    try
    {
      transform = tf_buffer_->lookupTransform(parent_frame_id, child_frame_id, tf2::TimePointZero);
    }
    catch (tf2::TransformException &ex)
    {
      RCLCPP_ERROR(this->get_logger(), "TransformException: %s", ex.what());
      return;
    }
    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = transform.header.stamp;
    pose.header.frame_id = transform.header.frame_id;
    pose.pose.position.x = transform.transform.translation.x;
    pose.pose.position.y = transform.transform.translation.y;
    pose.pose.position.z = transform.transform.translation.z;
    pose.pose.orientation = transform.transform.rotation;
    pose_publisher_->publish(pose);
  }

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> transform_listener_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::string parent_frame_id;
  std::string child_frame_id;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TFToPoseStamped>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}