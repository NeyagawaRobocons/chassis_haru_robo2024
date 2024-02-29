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
    this->declare_parameter("length", 0.400); // ToDo: check this value

    // declare parameters: number of wheels (3, 4)
    this->declare_parameter("num_wheels", 4);
    this->declare_parameter("max_input", 50.0); // [rad/sec]
    // get parameters
    std::string input_topic = this->get_parameter("input_topic").as_string();
    std::string output_topic = this->get_parameter("output_topic").as_string();
    this->radius = this->get_parameter("radius").as_double();
    this->length = this->get_parameter("length").as_double();
    // get parameters: num_wheels
    this->num_wheels = this->get_parameter("num_wheels").as_int();
    this->max_input = this->get_parameter("max_input").as_double();
    RCLCPP_INFO(this->get_logger(), "input_topic: %s", input_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "output_topic: %s", output_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "radius: %f", radius);
    RCLCPP_INFO(this->get_logger(), "length: %f", length);
    RCLCPP_INFO(this->get_logger(), "num_wheels: %d", num_wheels);
    RCLCPP_INFO(this->get_logger(), "max_input: %f", max_input);

    if (this->num_wheels == 3) {
      // robot_vel サブスクライバーの初期化
      robot_vel_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
          "/robot_vel", 10, std::bind(&CalcWheelVel::robot_vel_callback_3, this, _1));
    }
    else if (this->num_wheels == 4) {
      // robot_vel サブスクライバーの初期化
      robot_vel_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
          "/robot_vel", 10, std::bind(&CalcWheelVel::robot_vel_callback_4, this, _1));
    }
    else {
      RCLCPP_ERROR(this->get_logger(), "Invalid number of wheels");
    }

    // input_vel パブリッシャーの初期化
    input_vel_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/input_vel", 10);

    RCLCPP_INFO(this->get_logger(), "Calc wheel vel node has been initialized");
    
    timeout_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(2000), std::bind(&CalcWheelVel::timeout_callback, this));
  }

private:
  void robot_vel_callback_3(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    double v_x = msg->linear.x;
    double v_y = msg->linear.y;
    double omega = msg->angular.z;

    std_msgs::msg::Float64MultiArray input_vel_msg;
    input_vel_msg = calc_3wheel_vel(radius, length, v_x, v_y, omega);
    input_vel_msg = filter_max_vel(input_vel_msg, max_input);
    input_vel_publisher_->publish(input_vel_msg);
  }

  void robot_vel_callback_4(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    double v_x = msg->linear.x;
    double v_y = msg->linear.y;
    double omega = msg->angular.z;

    std_msgs::msg::Float64MultiArray input_vel_msg;
    input_vel_msg = calc_4wheel_vel(radius, length, v_x, v_y, omega);
    input_vel_msg = filter_max_vel(input_vel_msg, max_input);
    input_vel_publisher_->publish(input_vel_msg);
  }

  std_msgs::msg::Float64MultiArray calc_3wheel_vel(double radius, double length, double v_x, double v_y, double omega)
  {
    is_received = true;
    std_msgs::msg::Float64MultiArray input_vel;
    input_vel.data.resize(3);
    input_vel.data[0] = (-0.5 * v_x + 0.8660254037844386 * v_y + length * omega) / radius;
    input_vel.data[1] = (-0.5 * v_x - 0.8660254037844386 * v_y + length * omega) / radius;
    input_vel.data[2] = (v_x + length * omega) / radius;
    return input_vel;
  }

  std_msgs::msg::Float64MultiArray calc_4wheel_vel(double radius, double length, double v_x, double v_y, double omega)
  {
    is_received = true;
    std_msgs::msg::Float64MultiArray input_vel;
    input_vel.data.resize(4);
    input_vel.data[0] = ((-v_x + v_y) / 1.4142135623730951 + length * omega) / radius; // left front
    input_vel.data[1] = ((-v_x - v_y) / 1.4142135623730951 + length * omega) / radius; // right front
    input_vel.data[2] = ((v_x - v_y) / 1.4142135623730951 + length * omega) / radius; // right rear
    input_vel.data[3] = ((v_x + v_y) / 1.4142135623730951 + length * omega) / radius; // left rear
    return input_vel;
  }

  std_msgs::msg::Float64MultiArray filter_max_vel(std_msgs::msg::Float64MultiArray input_vel, double max_input)
  {
    for (int i = 0; i < input_vel.data.size(); i++)
    {
      if (input_vel.data[i] > max_input)
      {
        input_vel.data[i] = max_input;
      }
      else if (input_vel.data[i] < -max_input)
      {
        input_vel.data[i] = -max_input;
      }
    }
    return input_vel;
  }

  void timeout_callback()
  {
    if (!is_received)
    {
      std_msgs::msg::Float64MultiArray input_vel;
      input_vel.data.resize(num_wheels);
      for (int i = 0; i < num_wheels; i++)
      {
        input_vel.data[i] = 0.0;
      }
      input_vel_publisher_->publish(input_vel);
    }
    is_received = false;
  }

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr robot_vel_subscriber_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr input_vel_publisher_;
  double radius;
  double length;
  int num_wheels;
  double max_input;
  rclcpp::TimerBase::SharedPtr timeout_timer_;
  bool is_received = false;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CalcWheelVel>());
  rclcpp::shutdown();
  return 0;
}