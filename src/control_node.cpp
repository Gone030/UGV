#include "ugv/control_node.hpp"

#include <functional>
#include <utility>

namespace ugv {

ControlNode::ControlNode(const rclcpp::NodeOptions& options)
    : Node("ugv_control_node", options),
      cmd_vel_topic_(declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel")),
      enable_mcu_commands_(declare_parameter<bool>("enable_mcu_commands", false)),
      kinematics_(declare_parameter<double>("track_width_m", 0.0),
                  declare_parameter<double>("drive_sprocket_radius_m", 0.0)),
      serial_bridge_(make_null_serial_bridge()) {
  cmd_vel_subscription_ = create_subscription<geometry_msgs::msg::Twist>(
      cmd_vel_topic_, rclcpp::QoS(10),
      std::bind(&ControlNode::on_cmd_vel, this, std::placeholders::_1));

  if (!kinematics_.configured()) {
    RCLCPP_WARN(get_logger(),
                "Track dimensions are unset. Set positive track_width_m and "
                "drive_sprocket_radius_m before wheel targets can be produced.");
  }
  if (!enable_mcu_commands_) {
    RCLCPP_INFO(get_logger(),
                "MCU commands are disabled by parameter; serial backend is the safe null backend.");
  }
}

void ControlNode::on_cmd_vel(const geometry_msgs::msg::Twist::ConstSharedPtr message) {
  const auto targets = kinematics_.calculate(message->linear.x, message->angular.z);
  if (!targets) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                         "Rejected cmd_vel: track dimensions are unset or input is non-finite.");
    return;
  }

  WheelVelocityCommand command{};
  command.sequence = ++sequence_;
  command.enable = enable_mcu_commands_;
  command.left_target_rad_s = targets->left_rad_s;
  command.right_target_rad_s = targets->right_rad_s;

  if (!serial_bridge_->send_command(command)) {
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Wheel targets computed but not sent: USB CDC backend is not implemented/connected.");
  }

  // The real backend will return parsed STATE packets here. Feedback is for
  // monitoring/odometry consumers, never for the ROS-side motor control loop.
  static_cast<void>(serial_bridge_->poll_status());
}

}  // namespace ugv

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ugv::ControlNode>());
  rclcpp::shutdown();
  return 0;
}
