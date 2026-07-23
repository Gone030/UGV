#pragma once

#include <cstdint>
#include <memory>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "ugv/serial_bridge.hpp"
#include "ugv/track_kinematics.hpp"

namespace ugv {

class ControlNode final : public rclcpp::Node {
 public:
  explicit ControlNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

 private:
  void on_cmd_vel(const geometry_msgs::msg::Twist::ConstSharedPtr message);

  std::string cmd_vel_topic_;
  bool enable_mcu_commands_{};
  TrackKinematics kinematics_;
  std::unique_ptr<SerialBridge> serial_bridge_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscription_;
  std::uint32_t sequence_{};
};

}  // namespace ugv
