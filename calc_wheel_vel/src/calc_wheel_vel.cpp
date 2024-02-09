#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

using std::placeholders::_1;

class CalcWheelVel : public rclcpp::Node
{
public:
  CalcWheelVel() : Node("calc_wheel_vel")
  {
    RCLCPP_INFO(this->get_logger(), "Calc wheel vel node is initializing...");
    // declare parameters: input/output topic name
    this->declare_parameter("input_topic", "/robot_vel");
    this->declare_parameter("output_topic", "/input_vel");
    // declare parameters: tire radius, tire distance (= length)
    this->declare_parameter("radius", 0.0508);

    this->declare_parameter("length", 0.160); // ToDo: check this value

    // declare parameters: number of wheels (3, 4)
    this->declare_parameter("num_wheels", 4);
    // get parameters
    std::string input_topic = this->get_parameter("input_topic").as_string();
    std::string output_topic = this->get_parameter("output_topic").as_string();
    this->radius = this->get_parameter("radius").as_double();
    this->length = this->get_parameter("length").as_double();
    RCLCPP_INFO(this->get_logger(), "input_topic: %s", input_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "output_topic: %s", output_topic.c_str());

    // robot_vel サブスクライバーの初期化
    robot_vel_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/robot_vel", 10, std::bind(&CalcWheelVel::robot_vel_callback, this, _1));

    // input_vel パブリッシャーの初期化
    input_vel_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/input_vel", 10);

    RCLCPP_INFO(this->get_logger(), "Calc wheel vel node has been initialized");
  }

private:
  void robot_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    double v_x = msg->linear.x;
    double v_y = msg->linear.y;
    double omega = msg->angular.z;

    std_msgs::msg::Float64MultiArray input_vel_msg;
    input_vel_msg = calc_wheel_vel(radius, length, v_x, v_y, omega);
    input_vel_publisher_->publish(input_vel_msg);
  }

  std_msgs::msg::Float64MultiArray calc_wheel_vel(double radius, double length, double v_x, double v_y, double omega)
  {
    std_msgs::msg::Float64MultiArray input_vel;
    // get parameters: num_wheels
    const int num_wheels = this->get_parameter("num_wheels").as_int();
    if (num_wheels == 4) { // default
      // ToDo: make this calculation correct
      input_vel.data.resize(4);
      input_vel.data[0] = (v_x - v_y + length * omega) / radius; // left front
      input_vel.data[1] = (v_x + v_y + length * omega) / radius; // right front
      input_vel.data[2] = (v_x + v_y + length * omega) / radius; // right rear
      input_vel.data[3] = (v_x - v_y + length * omega) / radius; // left rear
      return input_vel;
    }
    else if (num_wheels == 3) {
      input_vel.data.resize(3);
      input_vel.data[0] = (-0.5 * v_x + 0.8660254037844386 * v_y + length * omega) / radius;
      input_vel.data[1] = (-0.5 * v_x - 0.8660254037844386 * v_y + length * omega) / radius;
      input_vel.data[2] = (v_x + length * omega) / radius;
      return input_vel;
    }
    else {
      RCLCPP_ERROR(this->get_logger(), "Invalid num_wheels: %d", num_wheels);
      return input_vel;
    }
  }

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr robot_vel_subscriber_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr input_vel_publisher_;
  double radius;
  double length;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CalcWheelVel>());
  rclcpp::shutdown();
  return 0;
}