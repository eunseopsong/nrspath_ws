#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>

#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <array>
#include <iostream>
#include <memory>
#include <chrono>

using PointStamped = geometry_msgs::msg::PointStamped;
using namespace std::chrono_literals;

class ClickedPointRegionPublisher : public rclcpp::Node
{
public:
  ClickedPointRegionPublisher()
  : Node("clicked_point_region_publisher"), index_(0)
  {
    pub_ = this->create_publisher<PointStamped>("/clicked_point", 10);

    // 기본 파라미터
    this->declare_parameter<std::string>("waypoint_dir", "cross_corner_waypoints");
    this->declare_parameter<std::string>("frame_id", "base_link");
    this->declare_parameter<double>("publish_rate_hz", 1.0);

    waypoint_dir_ = this->get_parameter("waypoint_dir").as_string();
    frame_id_ = this->get_parameter("frame_id").as_string();
    publish_rate_hz_ = this->get_parameter("publish_rate_hz").as_double();

    if (!select_and_load_region()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to load selected region waypoint file.");
      rclcpp::shutdown();
      return;
    }

    const auto period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      std::bind(&ClickedPointRegionPublisher::on_timer, this)
    );

    RCLCPP_INFO(this->get_logger(), "Publishing loaded waypoints to /clicked_point");
    RCLCPP_INFO(this->get_logger(), "Frame ID: %s", frame_id_.c_str());
    RCLCPP_INFO(this->get_logger(), "Loaded %zu waypoints", waypoints_.size());
  }

private:
  bool select_and_load_region()
  {
    int region_id;
    std::cout << "\nSelect region to publish (1 / 2 / 3 / 4): ";
    std::cin >> region_id;

    if (region_id < 1 || region_id > 4) {
      std::cerr << "Invalid region id. Must be one of 1, 2, 3, 4." << std::endl;
      return false;
    }

    selected_region_ = region_id;
    const std::string file_path = waypoint_dir_ + "/" + std::to_string(region_id) + ".txt";

    std::ifstream file(file_path);
    if (!file.is_open()) {
      std::cerr << "Failed to open waypoint file: " << file_path << std::endl;
      return false;
    }

    std::string line;
    size_t line_count = 0;

    while (std::getline(file, line)) {
      ++line_count;

      // 빈 줄 skip
      if (line.empty()) {
        continue;
      }

      // 주석 줄 skip (# ...)
      if (line[0] == '#') {
        continue;
      }

      std::istringstream iss(line);
      std::array<double, 3> pt;

      if (!(iss >> pt[0] >> pt[1] >> pt[2])) {
        std::cerr << "Warning: failed to parse line " << line_count
                  << " in file " << file_path << std::endl;
        continue;
      }

      waypoints_.push_back(pt);
    }

    file.close();

    if (waypoints_.empty()) {
      std::cerr << "No valid waypoints found in file: " << file_path << std::endl;
      return false;
    }

    std::cout << "Loaded region " << selected_region_
              << " from file: " << file_path << std::endl;
    std::cout << "Number of valid waypoints: " << waypoints_.size() << std::endl;

    return true;
  }

  void on_timer()
  {
    if (index_ >= waypoints_.size()) {
      RCLCPP_INFO(this->get_logger(), "All waypoints from region %d have been published.", selected_region_);
      rclcpp::shutdown();
      return;
    }

    const auto & pt = waypoints_[index_];

    PointStamped msg;
    msg.header.stamp = this->get_clock()->now();
    msg.header.frame_id = frame_id_;
    msg.point.x = pt[0];
    msg.point.y = pt[1];
    msg.point.z = pt[2];

    pub_->publish(msg);

    RCLCPP_INFO(
      this->get_logger(),
      "Published region %d waypoint %zu/%zu: [%.6f, %.6f, %.6f]",
      selected_region_,
      index_ + 1,
      waypoints_.size(),
      pt[0], pt[1], pt[2]
    );

    ++index_;
  }

private:
  rclcpp::Publisher<PointStamped>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::vector<std::array<double, 3>> waypoints_;
  size_t index_;
  int selected_region_;

  std::string waypoint_dir_;
  std::string frame_id_;
  double publish_rate_hz_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ClickedPointRegionPublisher>());
  rclcpp::shutdown();
  return 0;
}