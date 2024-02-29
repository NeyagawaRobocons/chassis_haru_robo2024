#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include "nucleo_agent/msg/odometer_data.hpp"
#include "odometry_class.hpp"

class OdomPubSub : public rclcpp::Node{
private:
    rclcpp::Subscription<nucleo_agent::msg::OdometerData>::SharedPtr _subscription;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_subscription;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr get_mcl_pose_sub;
    void _topic_callback(const nucleo_agent::msg::OdometerData::SharedPtr msg);
    void _initial_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
    void _get_mcl_pose_callback(const std_msgs::msg::Bool::SharedPtr msg);
    size_t count_;
    double radius;
    double length;
    Odometry odom;
    bool get_initialpose;
    bool get_mcl_pose;

public:
    OdomPubSub(
        const rclcpp::NodeOptions& options = rclcpp::NodeOptions()
    );
    OdomPubSub(
        const std::string& name_space, 
        const rclcpp::NodeOptions& options = rclcpp::NodeOptions()
    );
};