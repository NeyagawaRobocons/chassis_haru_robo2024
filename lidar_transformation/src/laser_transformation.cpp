/*
- description: 
  - this node subscribes /scan and transforms the data to /scan_transformed
  - transformation is read from tf (front/laser -> back/laser)
- input:
  - /scan (sensor_msgs/LaserScan)
  - tf (front/laser -> back/laser)
- output:
  - /scan_transformed (sensor_msgs/LaserScan)
- parameters:
  - ~frame_id (string, default: "front/laser")
  - ~child_frame_id (string, default: "back/laser")
  - ~input_topic (string, default: "/scan")
  - ~output_topic (string, default: "/scan_transformed")
- memo
  - sensor_msgs/LaserScan
    - header (std_msgs/Header)
      - frame_id (string)
    - angle_min (float32): start angle of the scan [rad]
    - angle_max (float32): end angle of the scan [rad]
    - angle_increment (float32): angular distance between measurements [rad]
    - time_increment (float32): time between measurements [seconds]
    - scan_time (float32): time between scans [seconds]
    - range_min (float32): minimum range value [m]
    - range_max (float32): maximum range value [m]
    - ranges (float32[]): range data [m]
    - intensities (float32[]): intensity data
*/

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_eigen/tf2_eigen.hpp>

class LidarTransformer : public rclcpp::Node
{
public:
  LidarTransformer() : Node("lidar_transformer")
  {
    this->declare_parameter<std::string>("frame_id", "front/laser");
    this->declare_parameter<std::string>("child_frame_id", "back/laser");
    this->declare_parameter<std::string>("input_topic", "/scan");
    this->declare_parameter<std::string>("output_topic", "/scan_transformed");
    this->declare_parameter<double>("angle_min_deg", 0.0);
    this->declare_parameter<double>("angle_max_deg", 360.0);
    this->frame_id = this->get_parameter("frame_id").as_string();
    auto input_topic = this->get_parameter("input_topic").as_string();
    auto output_topic = this->get_parameter("output_topic").as_string();
    angle_min = this->get_parameter("angle_min_deg").as_double() * M_PI / 180.0;
    angle_max = this->get_parameter("angle_max_deg").as_double() * M_PI / 180.0;
    if (angle_min > angle_max)
    {
      RCLCPP_ERROR(this->get_logger(), "angle_min_deg must be less than angle_max_deg");
    }
    RCLCPP_INFO(this->get_logger(), "frame_id: %s", frame_id.c_str());
    RCLCPP_INFO(this->get_logger(), "input_topic: %s", input_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "output_topic: %s", output_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "angle_min_deg: %f", angle_min * 180.0 / M_PI);
    RCLCPP_INFO(this->get_logger(), "angle_max_deg: %f", angle_max * 180.0 / M_PI);
    // initialize pub/sub
    sub = this->create_subscription<sensor_msgs::msg::LaserScan>(
      input_topic, 10, std::bind(&LidarTransformer::scan_callback, this, std::placeholders::_1));
    pub = this->create_publisher<sensor_msgs::msg::LaserScan>(output_topic, 10);
    // initialize tf
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    RCLCPP_INFO(this->get_logger(), "lidar_transformer node has been initialized");
  }

private:
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    // RCLCPP_INFO(this->get_logger(), "scan_callback");
    auto transformed_msg = transform_laser_scan(msg);
    transformed_msg->header.stamp = this->now();
    transformed_msg->header.frame_id = frame_id;
    pub->publish(*transformed_msg);
  }

  sensor_msgs::msg::LaserScan::SharedPtr transform_laser_scan(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    std::vector<std::vector<double>> points, transformed_points;
    std::vector<size_t> selected_indices; // 関数の最初で宣言してスコープを関数全体にする

    std::tie(transformed_points, selected_indices) = cutted_points_from_laser_scan(msg);
    auto transformed_msg = laser_scan_from_points(transformed_points, msg, selected_indices);
    return transformed_msg;
  }

  std::pair<std::vector<std::vector<double>>, std::vector<size_t>> cutted_points_from_laser_scan(const sensor_msgs::msg::LaserScan::SharedPtr &msg)
  {
    std::vector<std::vector<double>> points;
    std::vector<size_t> selected_indices; // 選択されたインデックスを格納する配列

    for (size_t i = 0; i < msg->ranges.size(); i++)
    {
      double angle = msg->angle_min + msg->angle_increment * i;
      if (angle < angle_min || angle > angle_max)
      {
          continue;
      }
      double x = msg->ranges[i] * cos(angle);
      double y = msg->ranges[i] * sin(angle);
      points.push_back({x, y});
      selected_indices.push_back(i); // 選択されたポイントのインデックスを保存
    }
    return {points, selected_indices}; // 点と選択されたインデックスのペアを返す
  }

  sensor_msgs::msg::LaserScan::SharedPtr laser_scan_from_points(
    const std::vector<std::vector<double>> &points,
    const sensor_msgs::msg::LaserScan::SharedPtr &original_msg,
    const std::vector<size_t> &selected_indices) // 変換後のポイントに対応する元のインデックス
  {
    auto transformed_msg = std::make_shared<sensor_msgs::msg::LaserScan>();
    // ヘッダー情報、角度、時間のインクリメント等を設定
    transformed_msg->angle_min = angle_min;
    transformed_msg->angle_max = angle_max;
    transformed_msg->angle_increment = original_msg->angle_increment;
    transformed_msg->time_increment = original_msg->time_increment;
    transformed_msg->scan_time = original_msg->scan_time;
    transformed_msg->range_min = original_msg->range_min;
    transformed_msg->range_max = original_msg->range_max;
    // `ranges`の設定
    transformed_msg->ranges.resize(points.size());
    transformed_msg->intensities.resize(points.size());
    for (size_t i = 0; i < points.size(); i++) {
      transformed_msg->ranges[i] = sqrt(pow(points[i][0], 2) + pow(points[i][1], 2));
      // 対応する`intensity`値を`intensities`配列に設定
      transformed_msg->intensities[i] = original_msg->intensities[selected_indices[i]];
    }
    return transformed_msg;
  }

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> transform_listener_;
  std::string frame_id;
  double angle_min;
  double angle_max;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LidarTransformer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
