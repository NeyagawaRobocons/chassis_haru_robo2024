#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

using std::placeholders::_1;

class PoseRateChange : public rclcpp::Node
{
public:
    PoseRateChange() : Node("pose_rate_change")
    {
        RCLCPP_INFO(this->get_logger(), "Pose rate change node is initializing...");
        // declare parameters: input/output topic name
        this->declare_parameter("input_topic", "/corrected_pose");
        this->declare_parameter("output_topic", "/robot_pose");
        // declare parameters: rate change
        this->declare_parameter("pub_rate", 10.0);
        // get parameters
        std::string input_topic = this->get_parameter("input_topic").as_string();
        std::string output_topic = this->get_parameter("output_topic").as_string();
        this->pub_rate = this->get_parameter("pub_rate").as_double();
        RCLCPP_INFO(this->get_logger(), "input_topic: %s", input_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "output_topic: %s", output_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "pub_rate: %f", pub_rate);
    
        // pose サブスクライバーの初期化
        pose_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            input_topic, 10, std::bind(&PoseRateChange::pose_callback, this, _1));
    
        // pose_rate_changed パブリッシャーの初期化
        pose_rate_changed_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            output_topic, 10);
    
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds((int) (1000.0 / pub_rate)), std::bind(&PoseRateChange::timer_callback, this));
        RCLCPP_INFO(this->get_logger(), "Pose rate change node has been initialized");
    }

private:
    void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        robot_pose_ = msg;
    }

    void timer_callback()
    {
        if (robot_pose_ != nullptr)
        {
            pose_rate_changed_publisher_->publish(*robot_pose_);
        }
    }

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_rate_changed_publisher_;
    geometry_msgs::msg::PoseStamped::SharedPtr robot_pose_;
    rclcpp::TimerBase::SharedPtr timer_;
    double pub_rate;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PoseRateChange>());
    rclcpp::shutdown();
    return 0;
}