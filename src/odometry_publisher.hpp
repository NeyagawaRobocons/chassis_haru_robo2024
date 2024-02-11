#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

class OdomPub : public rclcpp::Node{
private:
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  size_t count_;

public:
  OdomPub(
    const std::string& name_space="", 
    const rclcpp::NodeOptions& options = rclcpp::NodeOptions()
  );
};