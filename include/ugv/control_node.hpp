#pragma once

#include <cstdint>
#include <memory>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "ugv/serial_bridge.hpp"
#include "ugv/track_kinematics.hpp"

namespace ugv {

class ControlNode final : public rclcpp::Node {
 public:
  explicit ControlNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~ControlNode() override;

 private:
  void on_cmd_vel(const geometry_msgs::msg::Twist::ConstSharedPtr message);
  void on_serial_poll();
  void on_command_timeout();
  void send_disable_command();

  std::string cmd_vel_topic_;
  std::string mcu_state_topic_;
  bool enable_mcu_commands_{};
  std::uint32_t cmd_vel_timeout_ms_{};
  TrackKinematics kinematics_;
  std::unique_ptr<SerialBridge> serial_bridge_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscription_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mcu_state_publisher_;
  rclcpp::TimerBase::SharedPtr serial_poll_timer_;
  rclcpp::TimerBase::SharedPtr command_timeout_timer_;
  rclcpp::Time last_cmd_vel_time_;
  bool command_active_{};
  std::uint32_t sequence_{};
};

}  // namespace ugv
